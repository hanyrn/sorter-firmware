#!/usr/bin/env python3
"""
signal_model.py - pure-stdlib Python mirror of signal_model.h.

`signal_model.h` is the bench-test waveform: a train of randomly parameterised
pulses on a DC baseline with dither and impulse noise.  The M4 runs it to drive the
two DACs (A12 / A13) in the DAC -> AD7606 loopback.  This module mirrors that model
exactly - same algorithm, same documented draw order, same xorshift32 PRNG - so the
host tools can work out what the board is (or should be) generating without a board
attached:

  * test/plot_signals.py  --source dac   the offline SVG chart
  * test/live_plot.py     --sim          the live plot window, no hardware

Same seed in, same pulse train out.  The firmware computes in float32 and this
mirror in float64, so a drawn height/width can differ by one count in the last
digit - irrelevant for a plot, and every *decision* (how many pulses, where they
land, which are glitches) is integer-identical.

Run it directly for a statistical self-check, which is the Python twin of
test/test_signal_model.cpp:

    py -3 test/signal_model.py              # PASS/FAIL summary of the statistics
    py -3 test/signal_model.py --dump 3     # print the train drawn for 3 cycles
"""

import argparse
import math
import sys

MASK32 = 0xFFFFFFFF
LUT_SIZE = 256

# Waveform shape constants, mirrored from signal_model.h / config.h (SIGNALGEN_*).
DEFAULTS = dict(
    cycle_samples=20000,        # SIGNALGEN_CYCLE_SAMPLES  (2.0 s at 10 kSPS)
    max_pulses=48,              # SIGNALGEN_MAX_PULSES     (a packed cycle holds ~20)
    cycle_margin=600,           # SIGNALGEN_CYCLE_MARGIN
    gap_min=30,                 # SIGNALGEN_GAP_MIN        0.3 ms of quiet: neighbours
                                #                          crowd each other
    gap_max=900,                # SIGNALGEN_GAP_MAX        ... 90 ms at the most, so a
                                #                          cycle is packed end to end
    merge_pct=80,               # SIGNALGEN_MERGE_PCT        % of pulses that fuse
                                #                            with the previous one
    merged_span_min=1.50,       # SIGNALGEN_MERGED_SPAN_MIN  x width_mean
    merged_span_max=2.00,       # SIGNALGEN_MERGED_SPAN_MAX  x width_mean
    merge_min_overlap=0.12,     # SIGNALGEN_MERGE_MIN_OVERLAP  of the first width
    width_mean=1200.0,          # SIGNALGEN_WIDTH_MEAN       (120 ms at 10 kSPS)
    width_sd=260.0,             # SIGNALGEN_WIDTH_SD
    width_min=30,               # SIGNALGEN_WIDTH_MIN        3.0 ms: the narrowest
                                #                            event that still has to
                                #                            be measured
    width_max=3000,             # SIGNALGEN_WIDTH_MAX
    width_span_pct=40,          # SIGNALGEN_WIDTH_SPAN_PCT   % of peaks drawn
                                #                            log-uniform over
                                #                            [width_min, width_mean]
    height_mean=1800.0,         # SIGNALGEN_HEIGHT_MEAN       nominal peak (counts)
    height_sd=600.0,            # SIGNALGEN_HEIGHT_SD
    height_span_pct=30,         # SIGNALGEN_HEIGHT_SPAN_PCT   % of peaks drawn
                                #                             log-uniform instead
    height_min=320,             # SIGNALGEN_HEIGHT_MIN        counts (~1.3*5 sigma)
    height_max=40000,           # SIGNALGEN_HEIGHT_MAX        counts (past the clamp)
    frontier_pct=50,            # SIGNALGEN_FRONTIER_PCT      % of peaks that take
                                #                             the small-peak style in
                                #                             the band below
    frontier_width_min=200,     # SIGNALGEN_FRONTIER_WIDTH_MIN  the band the slice
    frontier_width_max=800,     # SIGNALGEN_FRONTIER_WIDTH_MAX  applies in: 20..80 ms
    frontier_trigger=200,       # SIGNALGEN_FRONTIER_TRIGGER  counts (k_on * sigma)
    frontier_rise=20,           # SIGNALGEN_FRONTIER_RISE     DET_MIN_RISE_SAMPLES
    frontier_margin_min=2.5,    # SIGNALGEN_FRONTIER_MARGIN_MIN  apex = floor(width)
    frontier_margin_max=5.0,    # SIGNALGEN_FRONTIER_MARGIN_MAX  x this..that
    ladder=0,                   # SIGNALGEN_LADDER         1 = draw the floor ladder
                                #   ... the ladder block below mirrors config.h's
                                #   SIGNALGEN_LADDER_* and signal_model.h::Config (checked
                                #   against config.h by test_signal_model.cpp's
                                #   defaultsMatchConfig - keep all three in step)
                                #                          instead of the random train
    ladder_width=600,           # SIGNALGEN_LADDER_WIDTH   samples (60 ms): every rung
    ladder_min=40,              # SIGNALGEN_LADDER_MIN     counts: bottom rung's apex,
                                #                          inside the +/-84-count dither
    ladder_step=80,             # SIGNALGEN_LADDER_STEP    counts between rungs
    ladder_count=12,            # SIGNALGEN_LADDER_COUNT   rungs: 40, 120 ... 920 counts
    glitch_pct=40,              # SIGNALGEN_GLITCH_PCT
    glitch_width_max=6,         # SIGNALGEN_GLITCH_WIDTH_MAX
    glitch_height_mean=3600.0,  # SIGNALGEN_GLITCH_HEIGHT_MEAN
    glitch_height_sd=2200.0,    # SIGNALGEN_GLITCH_HEIGHT_SD
    glitch_height_min=240,      # SIGNALGEN_GLITCH_HEIGHT_MIN
    glitch_height_max=40000,    # SIGNALGEN_GLITCH_HEIGHT_MAX
    baseline_code=2048,         # SIGNALGEN_BASELINE_CODE  (~1.65 V)
    noise_code=16,              # SIGNALGEN_NOISE_CODE     (~13 mV)
    noise_alpha=1.0,            # SIGNALGEN_NOISE_ALPHA    (1.0 = white)
    baseline_wave_code=150,     # SIGNALGEN_BASELINE_WAVE_CODE    slow wander of that
                                #                                 DC level, +/- codes
    baseline_wave_samples=300000,  # SIGNALGEN_BASELINE_WAVE_SAMPLES  30 s period
    spike_pct=0.05,             # SIGNALGEN_SPIKE_PCT      (% of samples with a spike)
    spike_max=400,              # SIGNALGEN_SPIKE_MAX      (DAC codes, uniform +/-)
    min_code=250,               # SIGNALGEN_MIN_CODE
    max_code=3850,              # SIGNALGEN_MAX_CODE
    adc_per_dac=5.28,           # SIGNALGEN_ADC_PER_DAC    (counts per DAC code)
    seed=0x2F6E2B1,             # SIGNALGEN_SEED
)

RAISED_COSINE = 'cos'
RECTANGLE = 'rect'


def clamp_i32(value, lo, hi):
    return lo if value < lo else (hi if value > hi else value)


def frontier_apex(trigger, rise_samples, width):
    """Mirror of signal_model.h::frontierApex().

    The smallest apex a raised cosine of `width` samples can have and still show
    `rise_samples` samples of leading edge above a trigger of `trigger` counts - the
    apex the detector's rise test can just accept:

        rise(a) = (width / 2) * (1 - acos(1 - 2*trigger/a) / pi)   samples
        rise(a) >= rise_samples   <=>   a >= trigger / cos^2(rise_samples*pi/width)

    which has a solution only for width > 2*rise_samples.  0 means no apex can.
    """
    if trigger <= 0 or rise_samples <= 0 or width <= 2 * rise_samples:
        return 0
    c = math.cos(rise_samples * math.pi / float(width))
    if c <= 0.0:
        return 0
    return int(math.ceil(trigger / (c * c)))


def baseline_wave_code(amplitude, period, total):
    """The slow baseline wander of signal_model.h::baselineWaveCode().

    A pure function of the *global* sample index, so it is continuous across the
    pattern-cycle boundary and reproducible from any point in a capture.  The modulo
    is taken on the integer index, never on a running phase.
    """
    if not amplitude or not period:
        return 0.0
    return amplitude * math.sin(2.0 * math.pi * (total % period) / float(period))


class SignalModel:
    """Mirror of sorter::SignalModel (signal_model.h)."""

    def __init__(self, **overrides):
        cfg = dict(DEFAULTS)
        cfg.update(overrides)
        self.cfg = cfg

        for key, value in cfg.items():
            setattr(self, key, value)

        self.lut = [0.5 * (1.0 - math.cos(2.0 * math.pi * i / LUT_SIZE))
                    for i in range(LUT_SIZE)]
        self.begin()

    # ---- PRNG (xorshift32, identical to the firmware) -----------------------
    def _next_u32(self):
        r = self.rng
        r = (r ^ ((r << 13) & MASK32)) & MASK32
        r = (r ^ (r >> 17)) & MASK32
        r = (r ^ ((r << 5) & MASK32)) & MASK32
        self.rng = r
        return r

    def _uniform01(self):
        """[0, 1) from the top 24 bits - exactly what the firmware's float gets."""
        return (self._next_u32() >> 8) / 16777216.0

    def _white(self):
        """[-1, 1) dither sample."""
        value = self._next_u32()
        if value & 0x80000000:
            value -= 0x100000000
        return value / 2147483648.0

    def _gauss_pair(self):
        """Box-Muller: two uniforms in, two independent standard normals out."""
        u1 = self._uniform01()
        u2 = self._uniform01()
        radius = math.sqrt(-2.0 * math.log(u1 if u1 > 0.0 else 1e-7))
        angle = 2.0 * math.pi * u2
        return radius * math.cos(angle), radius * math.sin(angle)

    # ---- lifecycle ----------------------------------------------------------
    def begin(self):
        """Seed the PRNG, draw cycle 0's train, put the baseline on the outputs."""
        self.rng = self.seed & MASK32
        self.pos = 0
        self.cycles = 0
        self.total = 0                 # samples since begin(): the wander's clock
        self.train_cycle = 0
        self.dither = 0.0
        self.spike = 0.0
        self.impulses = 0
        self.build_train()
        self.code = self.sample_code(self.pos)

    def tick(self):
        """Advance one sample; returns the DAC code the M4 would write."""
        if self.train_cycle != self.cycles:
            self.build_train()          # a new cycle draws a new train
            self.train_cycle = self.cycles

        self.dither += self.noise_alpha * (self._white() - self.dither)

        # Impulse noise: a single-sample spike, uniform in +/- spike_max, hundreds of
        # AD7606 counts tall - the "noise spikes" that look like peaks.  It is added
        # on top of the pulses (and inside the DAC clamp), so it also lands on real
        # peaks and shows up in their reported max.  The floor ladder draws none - and
        # does not spend the draws, so the two mirrors stay in step (signal_model.h).
        self.spike = 0.0
        if not self.ladder and (self._uniform01() * 100.0) < self.spike_pct:
            self.spike = self._white() * self.spike_max
            self.impulses += 1

        self.code = self.sample_code(self.pos)

        self.total += 1                # the wander never restarts with the pattern
        self.pos += 1
        if self.pos >= self.cycle_samples:
            self.pos = 0
            self.cycles += 1
        return self.code

    # ---- waveform -----------------------------------------------------------
    def build_train(self):
        """Draw one cycle's pulse train (mirror of SignalModel::buildTrain).

        The draw order is fixed and shared with the firmware: per pulse,
        gap/overlap distance, kind, overlap decision, u1, u2 (the Box-Muller pair),
        glitch width, log-uniform height, height style, width gate and width style.
        """
        self.pulses_list = []
        if self.ladder:
            self.build_ladder()
            return
        cursor = 0                       # end of the previous pulse, in samples
        prev_start = 0                   # start of the previous pulse
        prev_w = 0                       # width of the previous pulse
        prev_merged = False              # was the previous pulse merged into its own?

        while len(self.pulses_list) < self.max_pulses:
            u_dist = self._uniform01()   # gap, or the merged pair's span
            u_kind = self._uniform01()   # glitch or peak
            u_ov   = self._uniform01()   # fuse with the previous pulse?
            z0, z1 = self._gauss_pair()  # consumes u1 and u2, in that order
            u_w    = self._uniform01()   # glitch width
            u_hgt  = self._uniform01()   # log-uniform height
            u_span = self._uniform01()   # ... for this share of the peaks
            u_wgt  = self._uniform01()   # is this peak's width log-uniform?
            u_wid  = self._uniform01()   # ... the log-uniform width itself

            glitch = (u_kind * 100.0) < self.glitch_pct

            if glitch:
                width = 1 + int(u_w * self.glitch_width_max)
                amplitude = clamp_i32(round(self.glitch_height_mean +
                                            self.glitch_height_sd * z0),
                                      self.glitch_height_min, self.glitch_height_max)
                kind = RECTANGLE
            else:
                width = clamp_i32(round(self.width_mean + self.width_sd * z0),
                                  self.width_min, self.width_max)
                # The u_span draw is this pulse's *height style*: the small-peak
                # frontier slice, the log-uniform slice, or the nominal Gaussian.
                # The slice only applies inside the width band where the documented
                # envelope is the whole story; a pulse outside it keeps the ordinary
                # draws.  The width itself is drawn as always, whatever the style.
                style = u_span * 100.0
                if (style < self.frontier_pct and
                        self.frontier_width_min <= width <= self.frontier_width_max):
                    # A small peak drawn from the envelope the detector's own bounds
                    # allow at this width - the corner the independent height/width
                    # draws above never reach (signal_model.h::frontierApex).
                    floor_apex = frontier_apex(self.frontier_trigger,
                                               self.frontier_rise, width)
                    margin = self.frontier_margin_min + u_hgt * (
                        self.frontier_margin_max - self.frontier_margin_min)
                    amplitude = clamp_i32(round(floor_apex * margin),
                                          self.height_min, self.height_max)
                else:
                    if (u_wgt * 100.0) < self.width_span_pct:
                        # Log-uniform over [width_min, width_mean]: every octave
                        # equally likely, so the narrow end of the range is present
                        # in *every* cycle - a 3 ms needle next to a nominal-width
                        # peak.  It stops at width_mean on purpose: a single pulse
                        # must never be as wide as a merged pair, or the event
                        # widths stop having two modes.
                        width = clamp_i32(round(self.width_min *
                                                (self.width_mean /
                                                 float(self.width_min)) ** u_wid),
                                          self.width_min, self.width_max)
                    if (self.frontier_pct <= style <
                            self.frontier_pct + self.height_span_pct):
                        # Log-uniform over the whole span: every octave equally
                        # likely, so a cycle always contains both peaks that barely
                        # clear the noise band and peaks that clip the DAC
                        # (saturated, flat-topped ones).
                        amplitude = clamp_i32(
                            round(self.height_min *
                                  (float(self.height_max) /
                                   float(self.height_min)) ** u_hgt),
                            self.height_min, self.height_max)
                    else:
                        amplitude = clamp_i32(
                            round(self.height_mean + self.height_sd * z1),
                            self.height_min, self.height_max)
                kind = RAISED_COSINE

            # Where does this pulse start?  Normally after the previous pulse plus a
            # random gap, but merge_pct % of them are placed *inside* the previous
            # one, far enough in that the two shapes add up into one two-humped event
            # spanning merged_span_min..max nominal widths - clearly wider than any
            # single pulse, which is what makes the width distribution double peaked.
            # Merges come in pairs and only when the two really overlap enough to
            # fuse; otherwise the pulse is placed separated, like any other.
            start = None
            merged = False
            if not prev_merged and ((u_ov * 100.0) < self.merge_pct) and prev_w:
                span = int(round((self.merged_span_min + u_dist *
                                  (self.merged_span_max -
                                   self.merged_span_min)) * self.width_mean))
                need = int(self.merge_min_overlap * prev_w)
                # span > width keeps the order (never before the first pulse);
                # span + need <= prev_w + width is the overlap requirement.
                if span > width and span + need <= prev_w + width:
                    start = prev_start + span - width
                    merged = True
            if not merged:
                gap = self.gap_min + int(u_dist * (self.gap_max - self.gap_min + 1))
                start = cursor + gap

            # Keep the whole pulse (plus a margin) inside the cycle.
            if start + width + self.cycle_margin > self.cycle_samples:
                break

            self.pulses_list.append({'start': start, 'width': width,
                                     'amplitude': amplitude, 'kind': kind})
            cursor = max(cursor, start + width)
            prev_start = start
            prev_w = width
            prev_merged = merged

    def build_ladder(self):
        """The floor ladder (mirror of SignalModel::buildLadder).

        Deterministic - no PRNG draw at all - and the *only* content of the pattern: a
        staircase of rungs, one centred in each slot of the cycle, apexes stepping by
        `ladder_step` from `ladder_min`.  A raised cosine is exactly zero outside its
        own width, so a whole (slot - width) samples of nothing but dither and wander
        separate two rungs: that is what makes each one a peak *near the baseline*
        instead of a shoulder on the next one.
        """
        count = self.ladder_count or 1
        slot = self.cycle_samples // count
        if not self.ladder_width or slot <= self.ladder_width:
            return                      # the rungs would touch: nothing to draw
        for k in range(count):
            if len(self.pulses_list) >= self.max_pulses:
                break
            start = slot * k + (slot - self.ladder_width) // 2
            if start + self.ladder_width > self.cycle_samples:
                break                   # the slot arithmetic already keeps the last rung
                                        # clear of the boundary (a rung never straddles a
                                        # cycle the way a random pulse could)
            self.pulses_list.append({'start': start, 'width': self.ladder_width,
                                     'amplitude': self.ladder_min + k * self.ladder_step,
                                     'kind': RAISED_COSINE})

    def sample_code(self, index):
        """The DAC code for sample `index` of the current cycle."""
        value = (float(self.baseline_code) + self.dither * self.noise_code + self.spike +
                 baseline_wave_code(self.baseline_wave_code,
                                    self.baseline_wave_samples, self.total))

        for p in self.pulses_list:
            if index < p['start'] or index >= p['start'] + p['width']:
                continue
            amp = p['amplitude'] / self.adc_per_dac      # counts -> DAC codes
            if p['kind'] == RECTANGLE:
                value += amp
            else:
                u = ((index - p['start']) * LUT_SIZE) // p['width']
                if u >= LUT_SIZE:
                    u = LUT_SIZE - 1
                value += amp * self.lut[u]

        if value < self.min_code:
            value = float(self.min_code)
        if value > self.max_code:
            value = float(self.max_code)
        return int(math.floor(value + 0.5))      # like lroundf() for positive values

    # ---- accessors ----------------------------------------------------------
    @property
    def pulses(self):
        """The train in use, as a list of dicts (start/width/amplitude/kind)."""
        return self.pulses_list

    @property
    def pulse_count(self):
        return len(self.pulses_list)

    @property
    def position(self):
        """Sample index inside the current cycle (mirrors SignalModel::position())."""
        return self.pos

    @property
    def samples(self):
        """Samples generated since begin() (mirrors SignalModel::samples())."""
        return self.total

    @property
    def pulse_span(self):
        """(first sample, last sample) covered by the train, or (0, 0)."""
        if not self.pulses_list:
            return (0, 0)
        first = self.pulses_list[0]['start']
        last = self.pulses_list[-1]['start'] + self.pulses_list[-1]['width']
        return (first, last)


def event_spans(pulses):
    """The train's pulses grouped the way a detector would see them.

    A pulse that starts before the previous one ends overlaps it in the rendered
    waveform (both shapes add up, so the signal never returns to the noise band in
    between) and the detector times ONE event covering both.  Returns one
    (start, end, pulse_count) triple per event.
    """
    spans = []
    start = end = None
    pulses_in = 0
    for p in pulses:
        p_start = p['start']
        p_end = p_start + p['width']
        if end is None or p_start >= end:
            if end is not None:
                spans.append((start, end, pulses_in))
            start, end, pulses_in = p_start, p_end, 1
        else:
            end = max(end, p_end)
            pulses_in += 1
    if end is not None:
        spans.append((start, end, pulses_in))
    return spans


def build_dac_codes(total_samples, model=None):
    """The DAC codes the M4 would write for `total_samples` samples."""
    m = model if model is not None else SignalModel()
    m.begin()
    return [m.tick() for _ in range(total_samples)]


def build_cycles(cycles, model=None):
    """Like build_dac_codes(), but for whole pattern cycles.

    Returns (model, codes) - `model.pulses` is then the train that was in use for
    the *last* cycle, i.e. exactly the pulses those codes were built from.
    """
    m = model if model is not None else SignalModel()
    codes = build_dac_codes(cycles * m.cycle_samples, m)
    return m, codes


def describe_train(model, label='cycle'):
    """One line per pulse - for the --dump mode and for readable test output."""
    lo, hi = model.pulse_span
    lines = ['%s: %d pulse(s), samples %d..%d' % (label, model.pulse_count, lo, hi)]
    end = None
    for n, p in enumerate(model.pulses):
        merged = (end is not None) and (p['start'] < end)
        lines.append('  #%d  start=%6d  width=%5d smp  amp=%5d counts  %-4s%s'
                     % (n, p['start'], p['width'], p['amplitude'], p['kind'],
                        '  <- overlaps the previous pulse' if merged else ''))
        end = p['start'] + p['width']
    return lines


# ===========================================================================
# Self-check - the Python twin of test/test_signal_model.cpp
# ===========================================================================
def _stats(values):
    n = len(values)
    mean = sum(values) / float(n)
    variance = sum((v - mean) ** 2 for v in values) / float(n)
    return mean, math.sqrt(variance)


def _median(values):
    ordered = sorted(values)
    return ordered[len(ordered) // 2]


def _modes_and_valley(values, lo, hi, buckets):
    """(mode1, mode2, valley) of a histogram - the shape of a double-peaked draw.

    mode1 is the fullest bucket, the valley is where the histogram stops falling on
    one side of it, and mode2 is the fullest bucket beyond that valley (the taller of
    the two sides).
    """
    counts = [0] * buckets
    span = float(hi - lo) or 1.0
    for v in values:
        b = int((v - lo) * buckets / span)
        counts[min(max(b, 0), buckets - 1)] += 1

    m1 = counts.index(max(counts))

    right_dip = m1
    b = m1
    while b + 1 < buckets and counts[b + 1] < counts[b]:
        b += 1
        right_dip = b
    right = counts[right_dip + 1:]
    right_peak = max(right) if right else 0

    left_dip = m1
    b = m1
    while b > 0 and counts[b - 1] < counts[b]:
        b -= 1
        left_dip = b
    left = counts[:left_dip]
    left_peak = max(left) if left else 0

    if left_peak > right_peak:
        return counts[m1], left_peak, counts[left_dip]
    return counts[m1], right_peak, counts[right_dip]


def _hist_lines(values, lo, hi, buckets, width=40):
    """One line per bucket, so the shape of a distribution is visible in the output."""
    counts = [0] * buckets
    span = float(hi - lo) or 1.0
    for v in values:
        b = int((v - lo) * buckets / span)
        counts[min(max(b, 0), buckets - 1)] += 1
    top = max(counts) or 1
    return ['%6d %s %d' % (lo + span * b / buckets,
                           '#' * (width * counts[b] // top), counts[b])
            for b in range(buckets)]


def _peaks_short_of_amplitude(cfg, codes, trains):
    """The pulses whose rendered apex does not reach baseline + their amplitude.

    Neighbours and impulses can only *add* to a sample, so every pulse has to reach
    at least its own drawn amplitude - unless that amplitude is past the DAC rail,
    in which case the rail is as far as the source can go.  The baseline wander is
    why this is an exact statement and not an approximate one: the level a pulse
    sits on is the wander *at that sample*, not the nominal DC code.
    """
    short = []
    for cy, train in enumerate(trains):
        lo = cy * cfg['cycle_samples']
        for p in train:
            first = lo + p['start']
            last = first + p['width']
            at = max(range(first, last), key=lambda g: codes[g])
            want = (cfg['baseline_code'] +
                    baseline_wave_code(cfg['baseline_wave_code'],
                                       cfg['baseline_wave_samples'], at) +
                    p['amplitude'] / float(cfg['adc_per_dac']))
            if codes[at] < min(want, cfg['max_code']) - cfg['noise_code'] - 2:
                short.append(p)
    return short


def self_check(cycles=120, verbose=True):
    """Verify the model's invariants and the requested distributions."""
    failures = []
    cfg = DEFAULTS

    def check(name, ok, detail=''):
        if verbose:
            print('  %-4s %s%s' % ('ok' if ok else 'FAIL', name,
                                   ('   (%s)' % detail) if detail else ''))
        if not ok:
            failures.append(name)

    # ---- 1. determinism -----------------------------------------------------
    m_a = SignalModel()
    m_b = SignalModel()
    m_a.begin()
    m_b.begin()
    check('same seed -> same pulse train', m_a.pulses == m_b.pulses,
          '%d pulses in cycle 0' % m_a.pulse_count)
    check('same seed -> identical samples',
          build_dac_codes(4000) == build_dac_codes(4000))
    check('different seed -> different samples',
          build_dac_codes(4000) != build_dac_codes(4000, SignalModel(seed=0xC0FFEE)))

    # ---- 2. whole cycles through tick() (the firmware's own path) -----------
    k_cycles = 8                              # 160 000 samples
    model = SignalModel()
    model.begin()
    codes, trains = [], []
    for _ in range(k_cycles):
        for _ in range(model.cycle_samples):
            codes.append(model.tick())
        trains.append(list(model.pulses))

    check('%d cycles of %d samples' % (k_cycles, model.cycle_samples),
          len(codes) == k_cycles * model.cycle_samples and model.cycles == k_cycles
          and model.position == 0,
          'cycles=%d position=%d' % (model.cycles, model.position))
    check('every DAC code inside [%d, %d]' % (model.min_code, model.max_code),
          min(codes) >= model.min_code and max(codes) <= model.max_code,
          'min=%d max=%d' % (min(codes), max(codes)))
    check('a new train is drawn every cycle',
          trains[0] != trains[1] and trains[1] != trains[2])
    check('every cycle has 1..%d pulses' % model.max_pulses,
          all(0 < len(t) <= model.max_pulses for t in trains),
          'pulses per cycle: %s' % [len(t) for t in trains])

    # ---- 2b. the slow baseline wander ---------------------------------------
    # The DC level the pulses sit on drifts, and it has to drift in a way that no
    # peak bound can mistake for a signal:
    #   * one sine of +/-baseline_wave_code codes over baseline_wave_samples;
    #   * continuous across the pattern-cycle boundary (its clock is the *global*
    #     sample index, never the position inside the cycle);
    #   * slower than anything the detector's bounds are written for: at 150 codes
    #     over 300 000 samples it moves at most 0.004 codes per sample, i.e. four
    #     orders of magnitude below DET_MAX_SLOPE;
    #   * and present in the rendered codes exactly as the shared function says -
    #     an idle sample is the DC level plus the wander plus the dither, nothing
    #     else.  That is the property the host harness's ground truth leans on.
    amp = cfg['baseline_wave_code']
    per = cfg['baseline_wave_samples']
    at_quarter = baseline_wave_code(amp, per, per // 4)
    check('the wander is a sine of +/-%d codes over %d samples (%.0f s)'
          % (amp, per, per / 10000.0),
          abs(baseline_wave_code(amp, per, 0)) < 1e-9 and
          abs(baseline_wave_code(amp, per, per)) < 1e-9 and
          abs(at_quarter - amp) < 1e-6 and
          abs(baseline_wave_code(amp, per, 3 * per // 4) + amp) < 1e-6,
          '0 / %+.0f / 0 / %+.0f codes at 0, 1/4, 1/2, 3/4 of the period'
          % (at_quarter, baseline_wave_code(amp, per, 3 * per // 4)))
    check('the wander is continuous across the pattern-cycle boundary',
          abs(baseline_wave_code(amp, per, cfg['cycle_samples']) -
              baseline_wave_code(amp, per, cfg['cycle_samples'] - 1)) < 0.01,
          '%.4f -> %.4f codes at sample %d'
          % (baseline_wave_code(amp, per, cfg['cycle_samples'] - 1),
             baseline_wave_code(amp, per, cfg['cycle_samples']),
             cfg['cycle_samples']))
    fastest = max(abs(baseline_wave_code(amp, per, i + 1) -
                      baseline_wave_code(amp, per, i))
                  for i in range(0, per, 997))
    check('the wander moves far slower than any peak bound',
          fastest < 0.01, 'fastest step %.4f codes/sample (bound is 100)' % fastest)

    # ... and it is in the rendered codes exactly as the shared function says.  That
    # has to be measured on a run with the impulses switched off: an impulse is a
    # +/-400-code sample that lands on an idle sample as happily as on a peak, so it
    # is the one term that is not the DC level.  With it out of the way an idle
    # sample IS the DC level plus the wander plus the dither - nothing else moves the
    # DC level, which is the property the harness's ground truth leans on.
    quiet = SignalModel(spike_pct=0.0)
    quiet.begin()
    q_codes, q_trains = [], []
    for _ in range(8):                        # 16 s: past the 7.5 s peak and the
        for _ in range(cfg['cycle_samples']):  # zero crossing at 15 s
            q_codes.append(quiet.tick())
        q_trains.append(list(quiet.pulses))
    idle = []
    for i in range(0, len(q_codes), 37):
        cy, loc = divmod(i, cfg['cycle_samples'])
        if any(p['start'] <= loc < p['start'] + p['width'] for p in q_trains[cy]):
            continue
        idle.append((i, q_codes[i]))
    drift_err = max(abs(c - cfg['baseline_code'] - baseline_wave_code(amp, per, i))
                    for i, c in idle)
    check('an idle sample is exactly baseline + wander + dither',
          bool(idle) and drift_err <= cfg['noise_code'] + 1,
          'worst |idle - (baseline + wander)| = %d codes over %d idle samples'
          % (drift_err, len(idle)))
    lifted = max(c for _, c in idle) - cfg['baseline_code']
    dipped = min(c for _, c in idle) - cfg['baseline_code']
    check('the idle level really does wander up and down',
          lifted > 0.9 * amp and dipped < 0, '+%d / %d codes over %d idle samples'
          % (lifted, dipped, len(idle)))

    # ---- 3. per-pulse invariants over many cycles ---------------------------
    # Drawn straight through build_train() (the same PRNG stream and the same code
    # the firmware runs, just without the 20 000 ticks in between).
    gaps, peak_w, peak_h, glitch_w, glitch_h = [], [], [], [], []
    merged_span = []                  # samples from the first to the end of the second
    overlaps = []                     # (overlap / first width, first width)
    spans = []                        # (event span, pulses in it) as a detector sees it
    counters = {'gap': 0, 'span': 0, 'merged': 0}

    def collect(train):
        end = 0                       # end of the furthest pulse so far
        prev_start = 0                # start of the immediately previous pulse
        prev_w = 0
        for p in train:
            if prev_w and p['start'] < prev_start + prev_w:
                # This pulse is placed inside the previous one: a merged pair.
                counters['merged'] += 1
                merged_span.append(p['start'] + p['width'] - prev_start)
                overlaps.append(((prev_start + prev_w - p['start']) / float(prev_w),
                                 prev_w))
            else:
                gap = p['start'] - end
                if not (cfg['gap_min'] <= gap <= cfg['gap_max']):
                    counters['gap'] += 1
                gaps.append(gap)
            if p['start'] + p['width'] + cfg['cycle_margin'] > cfg['cycle_samples']:
                counters['span'] += 1
            if p['kind'] == RECTANGLE:
                glitch_w.append(p['width'])
                glitch_h.append(p['amplitude'])
            else:
                peak_w.append(p['width'])
                peak_h.append(p['amplitude'])
            end = max(end, p['start'] + p['width'])
            prev_start = p['start']
            prev_w = p['width']
        for (s, e, n) in event_spans(train):
            glitchy = any(p['kind'] == RECTANGLE and s <= p['start'] < e
                          for p in train)
            spans.append((e - s, n, glitchy))

    for train in trains:
        collect(train)
    model2 = SignalModel()
    for _ in range(cycles):
        model2.build_train()
        collect(list(model2.pulses))

    n_pulses = len(peak_h) + len(glitch_h)
    check('separated pulses keep gaps within [%d, %d]' % (cfg['gap_min'], cfg['gap_max']),
          counters['gap'] == 0, 'min=%d max=%d' % (min(gaps), max(gaps)))
    check('pulses stay clear of the cycle boundary', counters['span'] == 0)
    check('peak widths within [%d, %d]' % (cfg['width_min'], cfg['width_max']),
          all(cfg['width_min'] <= w <= cfg['width_max'] for w in peak_w))
    check('peak heights within [%d, %d]' % (cfg['height_min'], cfg['height_max']),
          all(cfg['height_min'] <= h <= cfg['height_max'] for h in peak_h))
    check('glitch widths within 1..%d' % cfg['glitch_width_max'],
          all(1 <= w <= cfg['glitch_width_max'] for w in glitch_w))

    # ---- 3c. the small-peak slice -------------------------------------------
    # The height draw is independent of the width, so on its own it almost never
    # puts a small apex on a pulse wide enough for the rise test (DET_MIN_RISE_SAMPLES)
    # to be the bound that decides.  The frontier slice exists to present that corner,
    # so the train has to hold such peaks - at least one per cycle on average, this
    # signal is the detector's small-peak test case - and every one of them has to sit
    # at or above the apex the rise test can accept at its own width: a "small peak"
    # drawn below that would be a test case for the wrong thing.
    frontier_per_cycle, frontier_below = [], 0
    frontier_widths, frontier_apexes = [], []
    for train in trains:
        seen = 0
        for p in train:
            if p['kind'] != RAISED_COSINE:
                continue
            if not (cfg['frontier_width_min'] <= p['width'] <=
                    cfg['frontier_width_max']):
                continue        # outside the band the slice does not apply
            floor_apex = frontier_apex(cfg['frontier_trigger'], cfg['frontier_rise'],
                                       p['width'])
            if floor_apex <= 0:
                continue
            if (floor_apex * cfg['frontier_margin_min'] - 1 <= p['amplitude'] <=
                    floor_apex * cfg['frontier_margin_max'] + 1):
                seen += 1
                frontier_widths.append(p['width'])
                frontier_apexes.append(p['amplitude'])
                if p['amplitude'] < floor_apex:
                    frontier_below += 1
        frontier_per_cycle.append(seen)
    check('the train presents small peaks on the detector\'s frontier',
          2 * len(frontier_apexes) >= len(trains),
          '%d peaks over %d cycles (as few as %d in one), %.1f..%.1f ms wide, '
          'apex %d..%d counts'
          % (len(frontier_apexes), len(frontier_per_cycle),
             min(frontier_per_cycle), 0.1 * min(frontier_widths),
             0.1 * max(frontier_widths), min(frontier_apexes),
             max(frontier_apexes)))
    check('no frontier peak is drawn under the apex the rise test needs',
          frontier_below == 0, '%d of %d below it' % (frontier_below,
                                                     len(frontier_apexes)))

    # ---- 3b. the pulses that merge ------------------------------------------
    # A merged pulse is *placed* inside the previous one so the pair spans a fixed
    # fraction of width_mean.  How many of the merge_pct draws survive depends on the
    # widths that happen to be drawn (a pair of very narrow pulses has no room), so
    # the share is only bounded - but every rule of the mechanism is checked.
    merged_pct = 100.0 * counters['merged'] / float(n_pulses)
    check('a fair share of the pulses merge with the previous one',
          5.0 < merged_pct < cfg['merge_pct'],
          '%.0f%% merged (%d of %d pulses, at most the drawn %d%%)'
          % (merged_pct, counters['merged'], n_pulses, cfg['merge_pct']))

    span_lo = int(round(cfg['merged_span_min'] * cfg['width_mean']))
    span_hi = int(round(cfg['merged_span_max'] * cfg['width_mean']))
    check('a merged pair spans %.0f..%.0f samples (%.2f..%.2f x width_mean)'
          % (span_lo, span_hi, cfg['merged_span_min'], cfg['merged_span_max']),
          min(merged_span) >= span_lo - 1 and max(merged_span) <= span_hi + 1,
          'saw %d..%d over %d pairs' % (min(merged_span), max(merged_span),
                                        len(merged_span)))
    wide = [r for (r, w) in overlaps if w >= 100]    # glitches are only 1..6 wide
    check('its pulses overlap by at least %.0f%% of the first one'
          % (100.0 * cfg['merge_min_overlap']),
          min(wide) >= 0.9 * cfg['merge_min_overlap'],
          'saw %.2f..%.2f (peaks only)' % (min(wide), max(wide)))
    check('a merged event covers exactly two pulses',
          all(n == 2 for (s, n, g) in spans if n > 1),
          'event sizes seen: %s' % sorted(set(n for (s, n, g) in spans)))

    # ---- 4. the requested distributions ------------------------------------
    g_mean, g_sd = _stats(gaps)
    expected_gap = 0.5 * (cfg['gap_min'] + cfg['gap_max'])
    expected_gap_sd = (cfg['gap_max'] - cfg['gap_min'] + 1) / math.sqrt(12.0)
    check('gap mean ~ %.0f (uniform)' % expected_gap,
          abs(g_mean - expected_gap) < 0.15 * expected_gap_sd,
          '%.0f, sd %.0f (uniform sd %.0f)' % (g_mean, g_sd, expected_gap_sd))

    # Peak width is a two-part draw as well: width_span_pct % of the peaks are
    # log-uniform over [width_min, width_mean] (the 3 ms needles), the rest is the
    # nominal Gaussian.  The Gaussian's own moments are therefore measured on a run
    # with the slice switched off, and the mixed run is checked for what the slice
    # exists to guarantee: the narrow end is really represented, in the share the
    # log-uniform draw implies.
    gauss = SignalModel(width_span_pct=0, frontier_pct=0)
    gauss_w = []
    for _ in range(cycles):
        gauss.build_train()
        gauss_w += [p['width'] for p in gauss.pulses if p['kind'] == RAISED_COSINE]
    gw_mean, gw_sd = _stats(gauss_w)
    check('peak width mean ~ %.0f (the Gaussian draw alone)' % cfg['width_mean'],
          abs(gw_mean - cfg['width_mean']) < 50.0,
          '%.0f over %d peaks' % (gw_mean, len(gauss_w)))
    check('peak width sd   ~ %.0f (the Gaussian draw alone)' % cfg['width_sd'],
          abs(gw_sd - cfg['width_sd']) < 50.0, '%.0f' % gw_sd)

    needle_cut = cfg['width_mean'] / 4.0
    needles = 100.0 * len([w for w in peak_w if w < needle_cut]) / float(len(peak_w))
    check('the narrow end of the width span is really drawn (needles)',
          min(peak_w) <= 1.25 * cfg['width_min'],
          'narrowest of %d peaks: %d samples (width_min %d)'
          % (len(peak_w), min(peak_w), cfg['width_min']))
    check('needles ~ width_span_pct of the peaks below %.0f samples' % needle_cut,
          12.0 < needles < 38.0,
          '%.0f%% below %.0f samples (%d of %d peaks)'
          % (needles, needle_cut, len([w for w in peak_w if w < needle_cut]),
             len(peak_w)))
    check('a single pulse still never reaches the merged band',
          max(peak_w) <= cfg['width_max'] and
          min(merged_span) >= cfg['merged_span_min'] * cfg['width_mean'] - 1,
          'widest single %d samples vs closest pair %d'
          % (max(peak_w), min(merged_span)))

    # Peak height is a two-part draw: height_span_pct % of the peaks are log-uniform
    # over the whole span (the extremes), the rest is the nominal Gaussian - and the
    # small-peak slice above puts a deliberately small apex on its own share on top of
    # that, which is its whole point.  So the *nominal* median is measured with that
    # slice out of the way, and the mixed run is checked for the shape of the mix: the
    # small peaks are a minority of it.
    nominal = SignalModel(frontier_pct=0)
    nominal_h = []
    for _ in range(cycles):
        nominal.build_train()
        nominal_h += [p['amplitude'] for p in nominal.pulses
                      if p['kind'] == RAISED_COSINE]
    n_median = _median(nominal_h)
    check('peak height median ~ %.0f (the nominal draws alone)' % cfg['height_mean'],
          abs(n_median - cfg['height_mean']) < 240.0,
          '%.0f over %d peaks' % (n_median, len(nominal_h)))
    small_pct = 100.0 * len([h for h in peak_h if h < 1000]) / float(len(peak_h))
    check('the small-peak slice is a minority of the peaks',
          5.0 < small_pct < 40.0,
          '%.0f%% of %d peaks are under 1000 counts (the slice is %d%%)'
          % (small_pct, len(peak_h), cfg['frontier_pct']))

    total = len(glitch_w) + len(peak_w)
    share = 100.0 * len(glitch_w) / float(total)
    check('glitch share ~ %d%% of the pulses (false peaks)' % cfg['glitch_pct'],
          len(glitch_w) > 0 and abs(share - cfg['glitch_pct']) < 8.0,
          '%.0f%% (%d glitches / %d pulses)' % (share, len(glitch_w), total))

    # ---- 4b. event widths: one mode per pulse, a second one for the merges ----
    # Glitch events (a lone 1..6-sample rectangle) are not "widths" and would just
    # pile up at zero, so they are reported separately and left out of the modes.
    glitch_events = [s for (s, n, g) in spans if g]
    peak_spans = [s for (s, n, g) in spans if not g]
    single = sorted(s for (s, n, g) in spans if n == 1 and not g)
    multi = sorted(s for (s, n, g) in spans if n > 1 and not g)
    check('merged pairs are a fair share of the peak events',
          len(multi) > 0 and 100.0 * len(multi) / len(peak_spans) > 10.0,
          '%d of %d peak events cover 2 pulses (%d glitch events ignored)'
          % (len(multi), len(peak_spans), len(glitch_events)))
    check('merged events are much wider (the second mode)',
          _median(multi) > _median(single) + 0.4 * cfg['width_mean'],
          'median %d vs %d samples (%d vs %d events)'
          % (_median(multi), _median(single), len(multi), len(single)))

    # ... and the two modes must really be two *peaks*: a histogram of the peak
    # widths has to fall into a dip between them.
    bucket = 150
    mode1, mode2, valley = _modes_and_valley(peak_spans, 0, 3000, 3000 // bucket)
    check('the width distribution is double peaked (a dip between the modes)',
          valley < 0.5 * min(mode1, mode2),
          'modes %d and %d events wide, dip %d (%d-sample buckets)'
          % (mode1, mode2, valley, bucket))
    if verbose:
        print('       peak widths (glitch events excluded) - bimodal on purpose:')
        for line in _hist_lines(peak_spans, 0, 3000, 20):
            print('         ' + line)

    # ---- 5. the rendered waveform: impulses, noise band, saturation ---------
    sample_count = len(codes)
    want = cfg['spike_pct'] * sample_count / 100.0
    check('impulses hit ~%.2f%% of the samples' % cfg['spike_pct'],
          abs(model.impulses - want) < 4.0 * math.sqrt(max(want, 1.0)),
          '%d impulses in %d samples (expected %.0f)'
          % (model.impulses, sample_count, want))
    check('impulses pull the trace below the baseline too',
          min(codes) < cfg['baseline_code'] - cfg['noise_code'],
          'lowest code %d, baseline %d %s%d'
          % (min(codes), cfg['baseline_code'], '-/+', cfg['noise_code']))

    clip = (cfg['max_code'] - cfg['baseline_code']) * cfg['adc_per_dac']
    check('the DAC rail is reached (peaks saturate the source)',
          max(codes) == cfg['max_code'],
          'highest code %d = rail; rail is %.0f counts above the baseline'
          % (max(codes), clip))
    check('every drawn peak reaches its amplitude (or the rail)',
          sum(1 for bad in _peaks_short_of_amplitude(cfg, codes, trains) if bad) == 0,
          '%d of %d pulses short'
          % (len(_peaks_short_of_amplitude(cfg, codes, trains)), n_pulses))

    low_share = 100.0 * len([h for h in peak_h if h < 2 * cfg['height_min']]) / \
        float(len(peak_h))
    clip_share = 100.0 * len([h for h in peak_h if h > clip]) / float(len(peak_h))
    check('peaks span from the noise floor to past the rail',
          _median(peak_h) < clip and low_share > 3.0 and clip_share > 3.0,
          '%.0f%% below %d counts (2x floor), %.0f%% past the rail'
          % (low_share, 2 * cfg['height_min'], clip_share))

    # ---- 6. an isolated pulse reaches baseline + its drawn amplitude ---------
    # With the random extras switched off (no overlap, no impulses, no glitches, no
    # needles, no baseline wander), the tallest peak of the train must land exactly
    # on baseline + amplitude / adc_per_dac - which is what makes the printed max=
    # comparable with the drawn amplitude at all.
    plain = SignalModel(merge_pct=0, spike_pct=0.0, glitch_pct=0,
                        height_span_pct=0, width_span_pct=0, frontier_pct=0,
                        baseline_wave_code=0)
    plain.begin()
    plain_codes = [plain.tick() for _ in range(plain.cycle_samples)]
    peak = max((p for p in plain.pulses if p['kind'] == RAISED_COSINE),
               key=lambda q: q['amplitude'])
    apex = max(plain_codes[peak['start']:peak['start'] + peak['width']])
    expected = cfg['baseline_code'] + peak['amplitude'] / float(cfg['adc_per_dac'])
    check('an isolated peak apex ~ baseline + amplitude',
          abs(apex - expected) <= cfg['noise_code'] + 2,
          'apex=%d, expected %.0f (amp %d counts)'
          % (apex, expected, peak['amplitude']))

    # ---- 7. the floor ladder -------------------------------------------------
    # `ladder` = 1 replaces the train with a deterministic staircase (see
    # signal_model.h::buildLadder): one rung per slot, ascending apexes, and no
    # impulses, no glitches and no random pulses at all.  The property that matters to
    # the plot is measured here: between two rungs the code is back on the wandering DC
    # level within the dither - that is what makes a rung a peak *near the baseline*
    # instead of a shoulder on its neighbour (config.h's SIGNALGEN_LADDER_* block).
    lad = SignalModel(ladder=1)
    lad.begin()
    slot = lad.cycle_samples // lad.ladder_count
    check('the ladder draws ladder_count rungs', len(lad.pulses) == lad.ladder_count,
          '%d rungs' % len(lad.pulses))
    shape = all(p['kind'] == RAISED_COSINE and p['width'] == lad.ladder_width and
                p['amplitude'] == lad.ladder_min + k * lad.ladder_step
                for k, p in enumerate(lad.pulses))
    check('every rung is a raised cosine of the ladder width, stepped by ladder_step',
          shape, '%d rungs of %d smp' % (len(lad.pulses), lad.ladder_width))
    fit = all(p['start'] == slot * k + (slot - lad.ladder_width) // 2 and
              p['start'] + p['width'] <= lad.cycle_samples
              for k, p in enumerate(lad.pulses))
    check('the rungs are centred in their slots and inside the cycle', fit,
          'slot %d smp, width %d smp' % (slot, lad.ladder_width))

    worst = 0
    for i in range(lad.cycle_samples):
        code = lad.tick()
        if any(p['start'] <= i < p['start'] + p['width'] for p in lad.pulses):
            continue
        want = lad.baseline_code + baseline_wave_code(lad.baseline_wave_code,
                                                      lad.baseline_wave_samples, i)
        worst = max(worst, int(round(abs(code - want))))
    check('between two rungs the trace is back at the baseline',
          worst <= lad.noise_code + 1,
          'worst |code - the wandering level| in a gap %d codes (dither +/-%d)'
          % (worst, lad.noise_code))
    check('the ladder draws no impulse noise', lad.impulses == 0,
          '%d impulses' % lad.impulses)

    return not failures


def main():
    parser = argparse.ArgumentParser(
        description='Mirror + self-check of the randomized bench waveform '
                    '(signal_model.h).')
    parser.add_argument('--cycles', type=int, default=120,
                        help='cycles drawn for the statistics (default 120)')
    parser.add_argument('--dump', type=int, default=0, metavar='N',
                        help='print the train drawn for N cycles, then exit')
    parser.add_argument('--quiet', action='store_true', help='only the summary')
    args = parser.parse_args()

    if args.dump:
        model = SignalModel()
        model.begin()
        for n in range(args.dump):
            if n:
                for _ in range(model.cycle_samples):
                    model.tick()
                model.tick()      # the first sample of a cycle draws its train
            print('\n'.join(describe_train(model, 'cycle %d' % n)))
        return 0

    print('signal_model.py - randomized bench waveform (mirror of signal_model.h)')
    print('  %d-sample cycle (%.1f s @ 10 kSPS), %d cycles for the statistics'
          % (DEFAULTS['cycle_samples'], DEFAULTS['cycle_samples'] / 10000.0,
             args.cycles))
    ok = self_check(cycles=args.cycles, verbose=not args.quiet)
    print('')
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())



