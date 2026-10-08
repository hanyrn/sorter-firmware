#!/usr/bin/env python3
"""
signal_model.py - pure-stdlib Python mirror of signal_model.h.

`signal_model.h` is the bench-test waveform: a train of randomly parameterised
pulses on a DC baseline with dither.  The M4 runs it to drive the two DACs (A12 /
A13) in the DAC -> AD7606 loopback.  This module mirrors that model exactly - same
algorithm, same documented draw order, same xorshift32 PRNG - so the host tools can
work out what the board is (or should be) generating without a board attached:

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
    max_pulses=16,              # SIGNALGEN_MAX_PULSES
    cycle_margin=600,           # SIGNALGEN_CYCLE_MARGIN
    gap_min=700,                # SIGNALGEN_GAP_MIN
    gap_max=3200,               # SIGNALGEN_GAP_MAX
    width_mean=1200.0,          # SIGNALGEN_WIDTH_MEAN
    width_sd=260.0,             # SIGNALGEN_WIDTH_SD
    width_min=300,              # SIGNALGEN_WIDTH_MIN
    width_max=3000,             # SIGNALGEN_WIDTH_MAX
    height_mean=900.0,          # SIGNALGEN_HEIGHT_MEAN
    height_sd=300.0,            # SIGNALGEN_HEIGHT_SD
    height_min=150,             # SIGNALGEN_HEIGHT_MIN
    height_max=3500,            # SIGNALGEN_HEIGHT_MAX
    glitch_pct=25,              # SIGNALGEN_GLITCH_PCT
    glitch_width_max=6,         # SIGNALGEN_GLITCH_WIDTH_MAX
    glitch_height_mean=1200.0,  # SIGNALGEN_GLITCH_HEIGHT_MEAN
    glitch_height_sd=600.0,     # SIGNALGEN_GLITCH_HEIGHT_SD
    glitch_height_min=200,      # SIGNALGEN_GLITCH_HEIGHT_MIN
    glitch_height_max=3500,     # SIGNALGEN_GLITCH_HEIGHT_MAX
    baseline_code=2048,         # SIGNALGEN_BASELINE_CODE  (~1.65 V)
    noise_code=6,               # SIGNALGEN_NOISE_CODE     (~4.8 mV)
    noise_alpha=1.0,            # SIGNALGEN_NOISE_ALPHA    (1.0 = white)
    min_code=250,               # SIGNALGEN_MIN_CODE
    max_code=3850,              # SIGNALGEN_MAX_CODE
    adc_per_dac=2.64,           # SIGNALGEN_ADC_PER_DAC    (counts per DAC code)
    seed=0x2F6E2B1,             # SIGNALGEN_SEED
)

RAISED_COSINE = 'cos'
RECTANGLE = 'rect'


def clamp_i32(value, lo, hi):
    return lo if value < lo else (hi if value > hi else value)


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
        self.train_cycle = 0
        self.dither = 0.0
        self.build_train()
        self.code = self.sample_code(self.pos)

    def tick(self):
        """Advance one sample; returns the DAC code the M4 would write."""
        if self.train_cycle != self.cycles:
            self.build_train()          # a new cycle draws a new train
            self.train_cycle = self.cycles

        self.dither += self.noise_alpha * (self._white() - self.dither)
        self.code = self.sample_code(self.pos)

        self.pos += 1
        if self.pos >= self.cycle_samples:
            self.pos = 0
            self.cycles += 1
        return self.code

    # ---- waveform -----------------------------------------------------------
    def build_train(self):
        """Draw one cycle's pulse train (mirror of SignalModel::buildTrain).

        The draw order is fixed and shared with the firmware: per pulse,
        gap, kind, u1, u2 (the Box-Muller pair), glitch width.
        """
        self.pulses_list = []
        cursor = 0                       # end of the previous pulse, in samples

        while len(self.pulses_list) < self.max_pulses:
            u_gap = self._uniform01()
            u_kind = self._uniform01()
            z0, z1 = self._gauss_pair()  # consumes u1 and u2, in that order
            u_w = self._uniform01()

            gap = self.gap_min + int(u_gap * (self.gap_max - self.gap_min + 1))
            start = cursor + gap
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
                amplitude = clamp_i32(round(self.height_mean + self.height_sd * z1),
                                      self.height_min, self.height_max)
                kind = RAISED_COSINE

            # Keep the whole pulse (plus a margin) inside the cycle.
            if start + width + self.cycle_margin > self.cycle_samples:
                break

            self.pulses_list.append({'start': start, 'width': width,
                                     'amplitude': amplitude, 'kind': kind})
            cursor = start + width

    def sample_code(self, index):
        """The DAC code for sample `index` of the current cycle."""
        value = float(self.baseline_code) + self.dither * self.noise_code

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
    def pulse_span(self):
        """(first sample, last sample) covered by the train, or (0, 0)."""
        if not self.pulses_list:
            return (0, 0)
        first = self.pulses_list[0]['start']
        last = self.pulses_list[-1]['start'] + self.pulses_list[-1]['width']
        return (first, last)


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
    for n, p in enumerate(model.pulses):
        lines.append('  #%d  start=%6d  width=%5d smp  amp=%5d counts  %s'
                     % (n, p['start'], p['width'], p['amplitude'], p['kind']))
    return lines


# ===========================================================================
# Self-check - the Python twin of test/test_signal_model.cpp
# ===========================================================================
def _stats(values):
    n = len(values)
    mean = sum(values) / float(n)
    variance = sum((v - mean) ** 2 for v in values) / float(n)
    return mean, math.sqrt(variance)


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

    # ---- 2. three full cycles through tick() (the firmware's own path) ------
    model = SignalModel()
    model.begin()
    codes, trains = [], []
    for _ in range(3):
        for _ in range(model.cycle_samples):
            codes.append(model.tick())
        trains.append(list(model.pulses))

    check('3 cycles of %d samples' % model.cycle_samples,
          len(codes) == 3 * model.cycle_samples and model.cycles == 3
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

    # ---- 3. per-pulse invariants over many cycles ---------------------------
    # Drawn straight through build_train() (the same PRNG stream and the same code
    # the firmware runs, just without the 20 000 ticks in between).
    gaps, peak_w, peak_h, glitch_w, glitch_h = [], [], [], [], []
    counters = {'gap': 0, 'span': 0}

    def collect(train):
        end = 0
        for p in train:
            gap = p['start'] - end
            if not (cfg['gap_min'] <= gap <= cfg['gap_max']):
                counters['gap'] += 1
            if p['start'] + p['width'] + cfg['cycle_margin'] > cfg['cycle_samples']:
                counters['span'] += 1
            gaps.append(gap)
            if p['kind'] == RECTANGLE:
                glitch_w.append(p['width'])
                glitch_h.append(p['amplitude'])
            else:
                peak_w.append(p['width'])
                peak_h.append(p['amplitude'])
            end = p['start'] + p['width']

    for train in trains:
        collect(train)
    model2 = SignalModel()
    for _ in range(cycles):
        model2.build_train()
        collect(model2.pulses)

    check('gaps are uniform in [%d, %d]' % (cfg['gap_min'], cfg['gap_max']),
          counters['gap'] == 0, 'min=%d max=%d' % (min(gaps), max(gaps)))
    check('pulses stay clear of the cycle boundary', counters['span'] == 0)
    check('peak widths within [%d, %d]' % (cfg['width_min'], cfg['width_max']),
          all(cfg['width_min'] <= w <= cfg['width_max'] for w in peak_w))
    check('peak heights within [%d, %d]' % (cfg['height_min'], cfg['height_max']),
          all(cfg['height_min'] <= h <= cfg['height_max'] for h in peak_h))
    check('glitch widths within 1..%d' % cfg['glitch_width_max'],
          all(1 <= w <= cfg['glitch_width_max'] for w in glitch_w))

    # ---- 4. the requested distributions ------------------------------------
    g_mean, g_sd = _stats(gaps)
    expected_gap = 0.5 * (cfg['gap_min'] + cfg['gap_max'])
    expected_gap_sd = (cfg['gap_max'] - cfg['gap_min'] + 1) / math.sqrt(12.0)
    check('gap mean ~ %.0f (uniform)' % expected_gap,
          abs(g_mean - expected_gap) < 0.15 * expected_gap_sd,
          '%.0f, sd %.0f (uniform sd %.0f)' % (g_mean, g_sd, expected_gap_sd))

    h_mean, h_sd = _stats(peak_h)
    w_mean, w_sd = _stats(peak_w)
    check('peak height mean ~ %.0f (Gaussian)' % cfg['height_mean'],
          abs(h_mean - cfg['height_mean']) < 50.0,
          '%.0f over %d peaks' % (h_mean, len(peak_h)))
    check('peak height sd   ~ %.0f (Gaussian)' % cfg['height_sd'],
          abs(h_sd - cfg['height_sd']) < 50.0, '%.0f' % h_sd)
    check('peak width  mean ~ %.0f (Gaussian)' % cfg['width_mean'],
          abs(w_mean - cfg['width_mean']) < 50.0,
          '%.0f over %d peaks' % (w_mean, len(peak_w)))
    check('peak width  sd   ~ %.0f (Gaussian)' % cfg['width_sd'],
          abs(w_sd - cfg['width_sd']) < 50.0, '%.0f' % w_sd)

    total = len(glitch_w) + len(peak_w)
    share = 100.0 * len(glitch_w) / float(total)
    check('glitch share ~ %d%% of the pulses (false peaks)' % cfg['glitch_pct'],
          len(glitch_w) > 0 and abs(share - cfg['glitch_pct']) < 8.0,
          '%.0f%% (%d glitches / %d pulses)' % (share, len(glitch_w), total))

    # ---- 5. a peak really reaches baseline + its drawn amplitude -----------
    peak = max((p for p in trains[0] if p['kind'] == RAISED_COSINE),
               key=lambda q: q['amplitude'])
    apex = max(codes[peak['start']:peak['start'] + peak['width']])
    expected = cfg['baseline_code'] + peak['amplitude'] / float(cfg['adc_per_dac'])
    check('peak apex ~ baseline + amplitude',
          abs(apex - expected) <= cfg['noise_code'] + 2,
          'apex=%d, expected %.0f (amp %d counts)' % (apex, expected, peak['amplitude']))

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



