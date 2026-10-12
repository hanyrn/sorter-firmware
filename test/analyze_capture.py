#!/usr/bin/env python3
"""
Measure a capture: where a detector window sits on its pulse, whether a channel moves
in steps, and whether one word reads another word with a bit held.

test/live_plot.py draws the stream and overlays the decisions the firmware made, and
an eye on that page is how the bench is judged.  Three of those judgements are
questions of *numbers*, not of shape, and the page cannot answer any of them:

  * is a detector window where the pulse is?  The window is printed in microseconds
    (the M4's clock) while the samples are printed as loop indices, and the page maps
    one onto the other with `us * 10 kSPS / 1e6`.  That identity is exact only when
    the stamps start at 0 with sample 0 - which is how test/test_bench_detect.cpp
    stamps its runs (`i * kDtUs`) and what sorter-firmware.ino's sampleStampUs() now
    does too.  A board built before that change stamps micros(), i.e. *uptime*, so
    every window is drawn `(uptime at the loop's first sample) / 100` samples to the
    right of the peak it describes - a constant shift, at both ends of every window.
    This tool measures the shift and says whether it is constant (an origin, and the
    firmware's) or scattered (a detector that really is late on some pulses);

  * does a channel deviate from the code written for it, and how?  A *step* that
    holds for several points while the code moves on - the level shift an eye reads
    as a glitch mid-pulse - has one of two signatures, and they point at different
    places:
      - a step of +/- 2^k counts (or a small sum of them) is *one data line*: module
        0's DB lines are latched first, closest to the RD edge and furthest up the
        ribbon, so a marginal line (D22..D37) or too short an AD7606_TACC_NS shows up
        on channel 0 of module 0 before it shows anywhere else;
      - a smooth deviation that grows with the code's slope, and reverses when the
        pulse comes back down, is *analog*: the input network (or a DAC output) not
        settling.  No cable fixes that one.
    The two are told apart by comparing the step against the powers of two, so this
    tool prints that comparison rather than a verdict about the wiring.

  * does one channel's word read another channel's word with one bit *held*?  The
    bench loopback writes the same code to both DACs, so the two columns carry the
    same input and their words have to match; a difference of exactly 2^k counts that
    appears whenever the shared input's bit k is *clear*, and vanishes when it is
    set, is not a voltage offset - it is one data line that never follows its input
    (an open pin, a ribbon contact, a module header).  A fixed offset of the same
    size does *not* track the bit, which is how the two are separated here; and the
    bit number names the wire:

        module m, DBk -> the entry m*16 + k of config.h's AD7606_DB_LINES[]

    which this tool resolves to the GIGA pin, because "DB12" is not a pin anyone can
    put a probe on.  (test/verify_adc_pinmap.ps1 checks that table itself.)

The counts a fit measures are also compared against the ones the AD7606 strap
(`--range`) and the firmware's SIGNALGEN_ADC_PER_DAC should produce.  They are a
factor of two apart between the straps, so a build and a board that disagree about
the RANGE pin make every absolute count bound - DET_MIN_MAX_VALUE, DET_MIN_AREA,
DET_MAX_SLOPE - mean something else than the firmware's author intended.

    py -3 test/live_plot.py --port COM6 --log run.txt     (Ctrl+C when done)
    py -3 test/analyze_capture.py run.txt
    py -3 test/analyze_capture.py run.txt --channel 0 --events 12

The capture is the file --log wrote - the board's own text, nothing added - so the
same run can be handed to someone else, or replayed with `--stdin < run.txt`.

With no board at all, --selftest builds a synthetic capture with defects planted in
it (a known step, a known one-point lag, a known stamp origin, a known held bit) and
checks that the measurements find them: that is what keeps this file honest.

    py -3 test/analyze_capture.py --selftest
"""

import argparse
import bisect
import math
import os
import random
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live_plot                                  # noqa: E402  - the page's own parser


# ===========================================================================
# The capture, whole
# ===========================================================================
class Capture:
    """Everything a capture's lines carry, with nothing dropped.

    live_plot.StreamState keeps the page's window and forgets what scrolls out of it;
    this drives the very same StreamParser over the very same text, but the questions
    here are about a whole run, so every point and every event is kept.
    """

    def __init__(self, window_sec=1.0, keep_sec=1.0):
        self.window_sec = window_sec
        self.keep_sec   = keep_sec
        self.idx    = []          # sample index of every point, ascending
        self.gen    = []          # the DAC code written for that sample
        self.cols   = []          # per column: the channel's counts, aligned with idx
        self.channels = []        # column -> AD7606 channel, from the STREAM header
        self.meta   = None
        self.events = []
        self.lines  = []

    # -- StreamState's surface, as StreamParser uses it ----------------------
    def set_meta(self, **kw):
        self.meta     = kw
        self.channels = list(kw.get('channels') or [])

    def add_batch(self, first_index, gen, adc, step=live_plot.DECIM):
        for k, code in enumerate(gen):
            self.idx.append(first_index + k * step)
            self.gen.append(code)
        while len(self.cols) < len(adc):
            self.cols.append([])
        for c, values in enumerate(adc):
            self.cols[c].extend(values)

    def add_event(self, ev):
        self.events.append(ev)

    def log(self, line):
        self.lines.append(line)

    def publish(self, obj):
        pass                      # nothing subscribes: the analysis reads the state

    # -- addressing ----------------------------------------------------------
    def column_of(self, channel):
        """The column carrying AD7606 channel `channel`, or None if it is not streamed."""
        try:
            return self.channels.index(channel)
        except ValueError:
            return None

    def span(self, i0, i1):
        """[lo, hi) into the point arrays for the sample indices i0..i1 inclusive."""
        return (bisect.bisect_left(self.idx, i0), bisect.bisect_right(self.idx, i1))

    @property
    def step(self):
        """Samples between two points: the decimation the header announced."""
        if len(self.idx) > 1:
            return self.idx[1] - self.idx[0]
        return live_plot.DECIM

    def rate(self):
        """Points per second of stream."""
        return live_plot.SAMPLE_RATE_HZ / max(self.step, 1)

    def baseline_code(self):
        """The code the generator rests on: the trace's own middle, in codes.

        The median, not the most common value: the baseline wanders (SIGNALGEN_
        BASELINE_WAVE_CODE) and the *clamp* is a single code that every tall pulse
        repeats, so a mode picks the clamp the moment enough pulses flat-top - which is
        exactly the capture this tool exists to read.
        """
        return int(statistics.median(self.gen)) if self.gen else 0

    def clamp_code(self):
        """The top code the capture reaches: the DAC's clamp, where tall peaks flat-top."""
        return max(self.gen) if self.gen else 0


def load(path, range_v):
    """Feed a capture file through the page's own parser."""
    cap    = Capture()
    parser = live_plot.StreamParser(cap, path, range_v, echo=False)
    with open(path, 'rb') as fh:
        while True:
            data = fh.read(1 << 16)
            if not data:
                break
            parser.feed_bytes(data)
    parser.flush()
    return cap


# ===========================================================================
# The measurements
# ===========================================================================
def fit_line(xs, ys):
    """Least-squares `a*x + b`, returned as (a, b); `a` is counts per DAC code."""
    n  = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx if sxx else 0.0
    return a, my - a * mx


def fit_robust(xs, ys, clip=4.0, passes=2):
    """`fit_line` with the outliers taken out, then refitted.

    A plain fit is *not* good enough here and the self-test is what showed it: one
    1024-count spike among 600 points tilts the slope by ~0.06 counts per code, and
    since only the pulse points sit far from the baseline that is ~18 counts of
    apparent residual on the next pulse's flanks - a phantom step.  Real captures
    carry spikes on purpose (the generator feeds impulses the detector must reject),
    so the fit drops whatever sits more than `clip` robust sigmas off it and fits
    again.  Returns (a, b).
    """
    a, b = fit_line(xs, ys)
    for _ in range(passes):
        res   = [y - (a * x + b) for x, y in zip(xs, ys)]
        med   = statistics.median(res)
        sigma = max(robust_sigma(res), 1e-9)
        keep  = [(x, y) for x, y, r in zip(xs, ys, res)
                 if abs(r - med) <= clip * sigma]
        if len(keep) < 8 or len(keep) == len(xs):
            break
        xs, ys = [k[0] for k in keep], [k[1] for k in keep]
        a, b = fit_line(xs, ys)
    return a, b


def follow_lag(cap, col, max_lag):
    """How many points a channel follows the code by, and the fit taken there.

    The generator's code is the truth about what the input was, so a channel is a
    linear map of it: `counts = a*code + b`.  Comparing the code's point `i` with the
    channel's point `i + lag` leaves a residual that is smallest at the lag the sample
    path really has - and it is a *time* measurement, so it is read as such:

        lag > 0   the channel is behind the code by lag points
                  (lag * step samples, lag / rate() seconds; +1 point = 1 ms)
        lag == 0  the channel is on the code

    The lag is picked on the *robust* spread of the residuals, not on their RMS: a
    spike is a large error at every lag and must not decide the alignment.  `a` is
    taken at that alignment, so the gain quoted is the one the channel actually has
    (the page draws the expected trace with config.h's nominal 2.64 counts per code,
    which is what makes a *fit* worth printing here).
    """
    best = None
    for lag in range(-max_lag, max_lag + 1):
        xs, ys = [], []
        for i, code in enumerate(cap.gen):
            j = i + lag
            if 0 <= j < len(cap.cols[col]):
                xs.append(code)
                ys.append(cap.cols[col][j])
        if len(xs) < 8:
            continue
        a, b   = fit_robust(xs, ys)
        res    = [y - (a * x + b) for x, y in zip(xs, ys)]
        spread = robust_sigma(res)                       # spikes show up here,
        rms    = math.sqrt(sum(r * r for r in res) / len(res))   # ... not here
        # The lag is chosen on the RMS of a *robust* fit: a spike cannot move the fit
        # (so it cannot move the choice either), while an alignment that only the
        # baseline points satisfy - which a MAD score would happily accept - blows up
        # on every pulse point and is rejected.
        if best is None or rms < best['rms']:
            best = {'lag': lag, 'a': a, 'b': b, 'spread': spread, 'rms': rms,
                    'n': len(xs)}
    return best


def residuals(cap, col, fit):
    """`channel - (a*code + b)` at the fitted alignment: (indices, residuals).

    What is left after the code is taken out is the channel's own error: the dither,
    anything the input network did to the edges, and any step - which is exactly what
    this file is looking for.
    """
    out_i, out_r = [], []
    for i, code in enumerate(cap.gen):
        j = i + fit['lag']
        if 0 <= j < len(cap.cols[col]):
            out_i.append(cap.idx[j])
            out_r.append(cap.cols[col][j] - (fit['a'] * code + fit['b']))
    return out_i, out_r


def robust_sigma(values):
    """MAD-based spread: one spike cannot widen it the way a plain std-dev would."""
    med = statistics.median(values)
    return 1.4826 * statistics.median([abs(v - med) for v in values])


def find_steps(idx, res, sigma, k=4.0, min_run=2):
    """Sustained rectangular deviations: a *step* in the trace, not a spike.

    A run of `min_run` points or more that sits `k` robust sigmas off the residual's
    own level, on one side, is a step: the trace jumped and *stayed* while the code
    moved on - which is what an eye reads as a level shift mid-pulse.  A lone point is
    a spike (dither, or a one-sample transient) and is left to the page's own `!max`
    tag rather than reported here.
    """
    if not res:
        return []
    level = statistics.median(res)
    out, i = [], 0
    while i < len(res):
        side = 0
        d = res[i] - level
        if d > k * sigma:
            side = 1
        elif d < -k * sigma:
            side = -1
        if side == 0:
            i += 1
            continue
        j = i
        while j < len(res) and (res[j] - level) * side > (k * sigma) / 2.0:
            j += 1
        run = res[i:j]
        if len(run) >= min_run:
            out.append({'first_index': idx[i], 'last_index': idx[j - 1],
                        'points': len(run), 'level': sum(run) / len(run) - level})
        i = j if j > i else i + 1
    return out


def power_of_two(level, tol=0.10):
    """`(k, 2**k)` when |level| is within `tol` of a power of two, else None.

    One mis-read data line moves a word by exactly 2^k counts (2^0 = the LSB, 2^15 =
    the sign bit), so the *size* of a step is the first thing that says whether a
    cable is involved at all.  `tol` absorbs the dither the channel is measured
    through; a deviation that fits no power of two is a shape, not a bit.
    """
    mag = abs(level)
    if mag < 1.0:
        return None
    for k in range(0, 16):
        if abs(mag - (1 << k)) <= tol * (1 << k):
            return k, (1 << k)
    return None


def pulse_span(cap, i0, i1, base):
    """The pulse's own span, at half its height, around the samples i0..i1.

    The code is what was *written*, so it is the pulse the detector was asked to find:
    comparing the window's centre with this span's centre is independent of how wide
    the window came out (a window is the excursion above the noise band, and so wider
    than any single level).  None when the window holds no pulse at all.
    """
    lo, hi = cap.span(i0, i1)
    if hi - lo < 2:
        return None
    apex = max(cap.gen[lo:hi])
    if apex - base <= 0:
        return None
    half = base + (apex - base) / 2.0
    peak = lo + max(range(hi - lo), key=lambda k: cap.gen[lo + k])
    left, right = peak, peak
    while left > 0 and cap.gen[left - 1] > half:
        left -= 1
    while right + 1 < len(cap.gen) and cap.gen[right + 1] > half:
        right += 1
    return {'left': cap.idx[left], 'right': cap.idx[right], 'apex': apex,
            'span': cap.idx[right] - cap.idx[left],
            'centre': (cap.idx[left] + cap.idx[right]) / 2.0}


def column_span(cap, col, pulse):
    """A measured channel's own half-height span centre over that pulse.

    The channel is compared at *its* half height (its own trough to its own apex), so
    a gain difference between the two modules cannot look like a delay - what is being
    measured here is time, and nothing else.
    """
    values = cap.cols[col]
    lo, hi = cap.span(pulse['left'] - pulse['span'], pulse['right'] + pulse['span'])
    if hi - lo < 4:
        return None
    window = values[lo:hi]
    low, apex = min(window), max(window)
    half = low + (apex - low) / 2.0
    peak = lo + max(range(hi - lo), key=lambda k: values[lo + k])
    left, right = peak, peak
    while left > lo and values[left - 1] > half:
        left -= 1
    while right + 1 < hi and values[right + 1] > half:
        right += 1
    return (cap.idx[left] + cap.idx[right]) / 2.0


# ===========================================================================
# One word against another: a bit that does not follow its input
# ===========================================================================
CONFIG_H = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                        'config.h')
DB_LINES = 16              # data lines per AD7606 module, DB0 (latched first) .. DB15
HELD_TOL = 32              # counts: under this, two words are "the same word"


def db_line_names(path=CONFIG_H):
    """The GIGA pin behind each AD7606_DB_LINES[] entry, in table order.

    Entry m * 16 + k is module m's DBk line, which is the entry that names the wire
    for a bit the firmware read from module m.  Returns [] if config.h is not where
    this file expects it, in which case the tool still reports the bit and the module
    - it just cannot put a pin name on it.
    """
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as handle:
            text = handle.read()
    except OSError:
        return []
    block = re.search(r'AD7606_DB_LINES\s*\[[^\]]*\]\s*=\s*\{(.*?)\};', text, re.S)
    if not block:
        return []
    return [entry.group(1) for entry in re.finditer(r'\{\s*(D\d+)\s*,', block.group(1))]


def db_pin(module, bit, names=None):
    """The Arduino pin module `module`'s DB`bit` is wired to, or None."""
    names = db_line_names() if names is None else names
    index = module * DB_LINES + bit
    return names[index] if 0 <= bit < DB_LINES and index < len(names) else None


def bit_fractions(cap, col):
    """For each bit of one column's word: the fraction of points that read 1.

    A held bit shows up here as a bit that does not move with the input, and that is
    what makes the pair search cheap: only a bit whose two columns disagree about how
    often it is set can be a held one.  (A bit that never moves in *either* word is
    one this capture cannot judge at all - the run never exercises it.)
    """
    values = cap.cols[col]
    n      = len(values)
    if not n:
        return [0.0] * DB_LINES
    ones = [0] * DB_LINES
    for value in values:
        word = value
        for k in range(DB_LINES):
            if word & 1:
                ones[k] += 1
            word >>= 1
    return [count / float(n) for count in ones]


def held_line(cap, ca, cb, tol=HELD_TOL, min_points=10, fixed=0.02):
    """Does column `ca` read column `cb`'s word with one bit held?  Best bit, or None.

    Every candidate bit is modelled as "cb's word with bit k forced to v", which is
    what one data line stuck at a level does to a word, and the model is judged in the
    two regimes *separately*: the points where cb's bit k already is v (where the model
    changes nothing) and the points where it is not (where the model adds or removes
    2^k).  That is the whole difference between a line and a voltage: a fixed offset of
    2^k counts explains one regime and fails the other, and the plain word comparison
    already explains a pair that matches anyway.  So a bit is only called held when
    *both* regimes are explained better than the plain comparison explains them.

    Only meaningful when the two columns carry the same input, which is what the
    caller has to know (the bench loopback, or one DAC split two ways) - two different
    signals can always be read as one word with a bit held.
    """
    a, b = cap.cols[ca], cap.cols[cb]
    n    = min(len(a), len(b))
    if n < 2 * min_points:
        return None
    fa, fb = bit_fractions(cap, ca), bit_fractions(cap, cb)
    best   = None
    for k in range(DB_LINES):
        bit = 1 << k
        # A held bit *is* constant in the word that has it - that is what held means -
        # so a bit that moves with the input in `ca` cannot be one, and this is the
        # cheap half of the search.  (A line that flaps reads constant with a little
        # slack, `fixed`.)  The reference word then has to *take* that bit both ways,
        # or there is nothing to judge the held one against.
        if bit <= tol or min(fa[k], 1.0 - fa[k]) > fixed:
            continue
        if fb[k] * n < min_points or (1.0 - fb[k]) * n < min_points:
            continue
        for value in (0, 1):
            ok, seen, same = [0, 0], [0, 0], [0, 0]
            for i in range(n):
                known = (b[i] >> k) & 1
                seen[known] += 1
                if -tol <= a[i] - b[i] <= tol:
                    same[known] += 1
                if -tol <= a[i] - ((b[i] & ~bit) | (value << k)) <= tol:
                    ok[known] += 1
            if min(seen) < min_points:
                continue                  # one regime is empty: nothing to rule out
            worst = min(ok[0] / seen[0], ok[1] / seen[1])
            base  = min(same[0] / seen[0], same[1] / seen[1])
            if best is None or worst - base > best['gain']:
                best = {'col': ca, 'other': cb, 'bit': k, 'value': value,
                        'worst': worst, 'base': base, 'gain': worst - base,
                        'points': n}
    if best is None or best['gain'] < 0.5 or best['worst'] < 0.8:
        return None
    k, value = best['bit'], best['value']
    high = low = follows = 0
    for i in range(n):
        mine   = (a[i] >> k) & 1
        theirs = (b[i] >> k) & 1
        if theirs == value:
            follows += 1
        elif mine == value:
            high += 1
        else:
            low += 1
    best.update({'held_high': high, 'held_low': low, 'follows': follows})
    return best


def fixed_gap(diffs, tol=HELD_TOL):
    """The power of two two words differ by *always*, if they differ by one at all.

    A gap that clean does not follow the input's bits, so it is a level - an input
    network, or a channel's own offset - and not a data line.  That is the other half
    of a pair's verdict, and the reason its `held` answer is worth believing.
    """
    if not diffs:
        return None
    middle = statistics.median(diffs)
    spread = robust_sigma([d - middle for d in diffs])
    if abs(middle) <= tol or spread > max(tol, abs(middle) * 0.01):
        return None
    for k in range(DB_LINES):
        bit = 1 << k
        if bit > tol and abs(abs(middle) - bit) <= max(tol, bit * 0.01):
            return (k, middle)
    return None


def pair_check(cap, ca, cb, fractions, tol=HELD_TOL):
    """What two columns' words are to each other: one word, one bit apart, or a gap.

    Also names the bits this capture cannot judge, because neither word moves them.
    """
    a, b  = cap.cols[ca], cap.cols[cb]
    n     = min(len(a), len(b))
    if not n:
        return {'pair': (ca, cb), 'agreement': 0.0, 'spread': 0.0, 'held': None,
                'gap': None, 'quiet': []}
    diffs = [a[i] - b[i] for i in range(n)]
    ways  = [h for h in (held_line(cap, ca, cb, tol), held_line(cap, cb, ca, tol)) if h]
    ways.sort(key=lambda h: -h['gain'])
    fa, fb = fractions[ca], fractions[cb]
    quiet  = [k for k in range(DB_LINES)
              if (1 << k) > tol
              and min(fa[k], 1.0 - fa[k]) < 0.02 and min(fb[k], 1.0 - fb[k]) < 0.02]
    return {'pair': (ca, cb),
            'agreement': (sum(1 for d in diffs if abs(d) <= tol) / float(n)) if n else 0.0,
            'spread': robust_sigma(diffs),
            'held': ways[0] if ways else None,
            'gap': fixed_gap(diffs, tol),
            'quiet': quiet}


def analyse(cap, cols, max_lag, min_run=2):
    """Every measurement this file makes, over one capture."""
    base  = cap.baseline_code()
    clamp = cap.clamp_code()
    fits  = {c: follow_lag(cap, c, max_lag) for c in cols}
    res   = {c: residuals(cap, c, fits[c]) for c in cols}
    sigma = {c: robust_sigma(res[c][1]) for c in cols}
    steps = {c: find_steps(res[c][0], res[c][1], max(sigma[c], 1.0), min_run=min_run)
             for c in cols}

    # The channel's mean error where the code is *clamped*: a step there is the
    # signature of the DAC's output stage or of the peak being read at a different
    # gain, and it is the one place the eye reads as "the middle of the peak moved".
    code_at = dict(zip(cap.idx, cap.gen))
    top = {}
    for c in cols:
        if clamp > base:
            vals = [r for i, r in zip(res[c][0], res[c][1]) if code_at.get(i, 0) >= clamp]
            top[c] = (sum(vals) / len(vals), len(vals)) if vals else None
        else:
            top[c] = None

    rows = []
    for ev in cap.events:
        i0, i1 = ev['start'], ev['end']
        pulse = pulse_span(cap, i0, i1, base)
        if pulse is None:
            continue
        row = {'event': ev['index'], 'start': i0, 'end': i1, 'window_span': i1 - i0,
               'width': ev['width_smp'], 'valid': ev['valid'], 'primary': ev['channel'],
               'apex': pulse['apex'], 'pulse_span': pulse['span'],
               'offset': (i0 + i1) / 2.0 - pulse['centre'],      # + = the window is late
               'chs': {}}
        for c in cols:
            centre = column_span(cap, c, pulse)
            if centre is not None:
                row['chs'][c] = centre - pulse['centre']
        rows.append(row)

    # One word against another, for every pair of columns asked about: the bench
    # loopback is the run where this means something, and it costs nothing to try.
    fractions = {c: bit_fractions(cap, c) for c in cols}
    pairs     = [pair_check(cap, cols[i], cols[j], fractions)
                 for i in range(len(cols)) for j in range(i + 1, len(cols))]

    return {'base': base, 'clamp': clamp, 'fits': fits, 'sigma': sigma, 'steps': steps,
            'top': top, 'rows': rows, 'pairs': pairs}


def ms(samples):
    """Samples as milliseconds of stream."""
    return samples / live_plot.SAMPLE_RATE_HZ * 1000.0


def channel_name(cap, col):
    """The AD7606 channel a column carries, as the header named it."""
    return cap.channels[col] if col < len(cap.channels) else col


def scale_lines(cap, found, cols, range_v):
    """What the fits measured, against the counts per code the strap should give.

    A DAC code is a fixed fraction of 3.3 V and a count is a fixed fraction of the
    AD7606's full scale, so the two only overlay at one counts-per-code - and the
    straps are a factor of two apart.  A build and a board that disagree about the
    RANGE pin therefore mean different things by the same absolute count bound.
    """
    want  = live_plot.gen_scale_counts(range_v)
    other = 5.0 if abs(range_v - 10.0) < 2.5 else 10.0
    there = live_plot.gen_scale_counts(other)
    out   = ['  scale  : the counts per DAC code the fit measured, against the strap '
             '--range names',
             '           measured %s; --range %g wants %.3f (SIGNALGEN_ADC_PER_DAC)'
             % (', '.join('ch%d %.3f' % (channel_name(cap, c), found['fits'][c]['a'])
                          for c in cols), range_v, want)]
    gains = [found['fits'][c]['a'] for c in cols]
    if all(abs(g - want) <= 0.05 * want for g in gains):
        out.append('           the count scale and the strap agree.')
    elif any(abs(g - there) <= 0.05 * there for g in gains):
        out.append('           that is the +/-%g V strap (%.3f), %.1fx the +/-%g V one '
                   '--range' % (other, there, there / want, range_v))
        out.append('           (and the build) name: the modules\' RANGE pin and the build')
        out.append('           disagree.  Every absolute count bound - DET_MIN_MAX_VALUE,')
        out.append('           DET_MIN_AREA, DET_MAX_SLOPE - is a count *at the strap\'s')
        out.append('           scale*, so all of them are off by that factor too.  See')
        out.append('           README, "Levels", on the strap being a package deal.')
    else:
        out.append('           that is neither strap (+/-5 V %.3f, +/-10 V %.3f): either '
                   'the fit is' % (live_plot.gen_scale_counts(5.0),
                                   live_plot.gen_scale_counts(10.0)))
        out.append('           being read across a defect, or this is not the generator '
                   'it came from.')
    if len(gains) > 1 and max(gains) - min(gains) > 0.005 * max(gains):
        out.append('           the columns\' own gains differ by %.1f%%, which a held bit '
                   'does -' % (100.0 * (max(gains) - min(gains)) / max(gains)))
        out.append('           it skews the fit that crosses it, so read them with "lines".')
    return out


def line_lines(cap, found, cols):
    """Does one column's word read another column's word with one bit held?"""
    out = ['  lines  : one column\'s word against another\'s, on one input (the bench',
           '           loopback writes the same code to both DACs).  A bit that does not',
           '           follow that input\'s bit, in the points where the input has it both',
           '           ways, is one data line - an offset of the same size does not track',
           '           the input\'s bit, which is what separates the two here.']
    if len(cols) < 2:
        out.append('           one column in this capture: a held bit needs a second '
                   'column')
        out.append('           carrying the same input before it can be seen at all.')
        return out

    pairs = found['pairs']
    for pair in pairs:
        held = pair['held']
        if not held:
            continue
        ca, cb = pair['pair']
        bit    = held['bit']
        value  = held['value']
        total  = float(held['points'])
        module = cap.channels[held['col']] // 8 if held['col'] < len(cap.channels) else 0
        pin    = db_pin(module, bit)
        out.append('           ch%d vs ch%d: bit %d is held at %d - %d point(s) (%.1f%%) '
                   'read %d counts'
                   % (channel_name(cap, ca), channel_name(cap, cb), bit, value,
                      held['held_high'], 100.0 * held['held_high'] / total, 1 << bit))
        out.append('                       %s where the shared input\'s bit %d is %s, and '
                   'the %d point(s)'
                   % ('high' if value else 'low', bit, 'clear' if value else 'set',
                      held['follows']))
        out.append('                       (%.1f%%) where it is %s follow exactly: 2^%d = '
                   '%d counts.'
                   % (100.0 * held['follows'] / total,
                      'set' if value else 'clear', bit, 1 << bit))
        if held['held_low']:
            out.append('                       the %d point(s) (%.1f%%) where it reads the '
                       'input\'s value instead'
                       % (held['held_low'], 100.0 * held['held_low'] / total))
            out.append('                       say the line is loose, not shorted.')
        if pin:
            wire = ('-> module %d DB%d = GIGA %s (config.h AD7606_DB_LINES[%d])'
                    % (module, bit, pin, module * DB_LINES + bit))
        else:
            wire = '-> module %d DB%d (config.h not found, so no pin name)' % (module, bit)
        out.append('                       %s' % wire)

    if not any(pair['held'] for pair in pairs):
        best   = max(pairs, key=lambda pair: pair['agreement'])
        ca, cb = best['pair']
        out.append('           ch%d vs ch%d: the two words agree (%.1f%% of the points '
                   'within %d counts,'
                   % (channel_name(cap, ca), channel_name(cap, cb),
                      100.0 * best['agreement'], HELD_TOL))
        out.append('           spread %d counts), so no bit is held in either word.'
                   % round(best['spread']))
        if best['gap']:
            out.append('           (their fixed %d-count gap, 2^%d, does not follow the '
                       'input: a level - an' % (round(best['gap'][1]), best['gap'][0]))
            out.append('           input network or an offset - and not a data line.)')
        if len(pairs) > 1:
            out.append('           %d pair(s) checked.' % len(pairs))
        if best['quiet']:
            out.append('           bits this capture never moves in either word, so it '
                       'cannot judge them: %s'
                       % ', '.join(str(k) for k in best['quiet']))
    return out


def report(cap, found, cols, args):
    """Print the measurements and the reading of them.  Returns the lines printed."""
    out = []
    add = out.append
    header = next((l for l in cap.lines if l.startswith('STREAM ')), '')
    step_ms = ms(cap.step)

    add('capture: %s' % args.capture)
    add('         %d point(s), decim %d (1 point = %.2f ms, %.0f points/s), columns %s'
        % (len(cap.idx), cap.step, step_ms, cap.rate(),
           ', '.join('ch%d' % channel_name(cap, c) for c in cols)))
    if header:
        add('         header: %s' % header)
    add('         baseline code %d, top code %d' % (found['base'], found['clamp']))
    add('         %d closed event(s) in the capture, %d with a pulse inside the window'
        % (len(cap.events), len(found['rows'])))
    add('')

    rows = found['rows']
    add('  event  window (samples)      win   pulse   apex   window centre - pulse centre')
    for row in rows[:args.events]:
        add('   #%-4d %8d..%-8d %5d   %5d   %5d   %+8.2f ms'
            % (row['event'], row['start'], row['end'], row['window_span'],
               row['pulse_span'], row['apex'], ms(row['offset'])))
    if len(rows) > args.events:
        add('   ... %d more event(s)' % (len(rows) - args.events))

    # ---- the reading: the overlay -----------------------------------------
    add('')
    if not rows:
        add('  overlay: no window in this capture has a pulse inside it - either nothing')
        add('           was detected, or the windows are stamped for another run.')
    else:
        offs   = [r['offset'] for r in rows]
        med    = statistics.median(offs)
        spread = max(offs) - min(offs)
        wide   = [r for r in rows if abs(r['offset']) > 2.0 * cap.step]
        add('  overlay: window centre - pulse centre: median %+.2f ms over %d event(s), '
            'spread %.2f ms' % (ms(med), len(offs), ms(spread)))
        if abs(med) <= cap.step and not wide:
            add('           every window sits on the pulse it describes (within a point).')
        else:
            # The median says what the *common* case is; the outliers are named below,
            # because a window that swallowed two pulses is not a timing fault - and a
            # constant shift of every window is.
            if abs(med) <= cap.step:
                add('           the median is on the pulse, but %d of %d window(s) are '
                    'more than' % (len(wide), len(rows)))
                add('           two points off - named below, with their spans.')
            elif spread <= max(2.0 * cap.step, 0.25 * abs(med)):
                add('           a *constant* shift, at both ends of every window: an '
                    'origin,')
                add('           not a detector that is late.  The window is stamped with '
                    "the M4's")
                add('           micros() (uptime) while the `S,` lines count samples from '
                    "the")
                add("           acquisition loop's first sample.  Reflash a board built")
                add('           with sampleStampUs() (sorter-firmware.ino) and the two '
                    'agree:')
                add('           %+.2f ms is %.0f samples of uptime the loop began with.'
                    % (ms(med), med))
            else:
                add('           scattered (spread %.2f ms), so the median is not the '
                    'common case -' % ms(spread))
                add('           a late window is the detector\'s own:')
            for r in sorted(rows, key=lambda r: -abs(r['offset']))[:3]:
                add('           event #%-3d %+8.2f ms   (window %d smp, pulse %d smp)'
                    % (r['event'], ms(r['offset']), r['window_span'], r['pulse_span']))
        for c in cols:
            vals = [r['chs'][c] for r in rows if c in r['chs']]
            if vals:
                add('           ch%-3d measured pulse centre - code pulse centre: '
                    'median %+.2f ms over %d pulse(s)'
                    % (channel_name(cap, c), ms(statistics.median(vals)), len(vals)))

    # ---- the reading: what follows what -----------------------------------
    add('')
    add('  lag    : how far each channel follows the code (the fit is taken at that '
        'alignment)')
    for c in cols:
        f = found['fits'][c]
        add('           ch%-3d %+d point(s) = %+7.2f ms   gain %.3f counts/code   '
            'rms %.1f counts (robust %.1f)'
            % (channel_name(cap, c), f['lag'], ms(f['lag'] * cap.step), f['a'],
               f['rms'], f['spread']))
    add('')
    for line in scale_lines(cap, found, cols, args.range_v):
        add(line)

    # ---- the reading: steps ------------------------------------------------
    add('')
    add('  steps  : sustained deviations from the code, >= 2 points (a lone point is a')
    add('           spike, not a step), largest first')
    for c in cols:
        runs = sorted(found['steps'][c], key=lambda s: -s['points'])[:args.steps]
        if not runs:
            add('           ch%-3d none' % channel_name(cap, c))
        for s in runs:
            p2    = power_of_two(s['level'])
            where = ('mid-pulse' if any(r['start'] <= s['first_index'] <= r['end']
                                       for r in rows) else 'between pulses')
            what  = ('a data line: ~%s%d counts' % ('-' if s['level'] < 0 else '+',
                                                    p2[1])
                     if p2 else 'fits no power of two - a shape, not a bit')
            add('           ch%-3d %d..%d  %d point(s) (%+.1f ms)  %+d counts  %s  (%s)'
                % (channel_name(cap, c), s['first_index'], s['last_index'], s['points'],
                   ms(s['points'] * cap.step), round(s['level']), what, where))

    # ---- the reading: the words, against each other ------------------------
    add('')
    for line in line_lines(cap, found, cols):
        add(line)

    # ---- the reading: the top of a tall peak -------------------------------
    if found['clamp'] > found['base']:
        add('')
        add('  apex   : the code is clamped at %d where it would have gone higher'
            % found['clamp'])
        for c in cols:
            if found['top'][c]:
                mean_err, n = found['top'][c]
                add('           ch%-3d reads %+.0f counts off the fit there (%d point(s)) - '
                    "a DAC's" % (channel_name(cap, c), mean_err, n))
                add('                   top is where its output stage runs out of '
                    'headroom, so this')
                add('                   number is a property of the generator, not of the '
                    'input.')
    return out


# ===========================================================================
# A capture with no board: the self-test
# ===========================================================================
def synthetic_text(pulses, n_points=600, decim=live_plot.DECIM, origin_us=0,
                   baseline=2048, gain=2.64, noise=3.0, planted=None, lag_points=None,
                   held=None):
    """The board's own text for a hand-made waveform, defects and all.

    pulses      [(first point, width points, height codes)] - what the *code* draws,
                a raised cosine per pulse like the generator's peaks
    planted     [(column, first point, points, delta counts)] - a step put in on purpose
    lag_points  {column: points} - that channel reads the code that many points earlier
    held        (column, bit, value) - that column's word reads with that data line
                held at `value`, which is what a stuck or open line does to every point
    origin_us   what the *event* lines are stamped with.  0 is the sample grid, which
                is what sampleStampUs() prints and what test_bench_detect.cpp has
                always printed; anything else is an uptime origin, i.e. a board that
                stamps micros().

    The result is the three line shapes the viewer parses, so these runs go through the
    very parser a board's capture does - which is the point of keeping this here.
    """
    rng  = random.Random(0x5EED)
    code = [baseline] * n_points
    for first, width, height in pulses:
        span = max(width - 1, 1)
        for k in range(width):
            p = first + k
            if 0 <= p < n_points:
                code[p] = baseline + int(round(
                    height * (1.0 - math.cos(2.0 * math.pi * k / span)) / 2.0))

    lag_points = lag_points or {}
    lines = ['STREAM on: decim=%d chans=0,8 wf=1 points=%d minw=35'
             % (decim, live_plot.POINTS)]
    for p in range(n_points):
        cells = []
        for col, ch in enumerate((0, 8)):
            src   = min(max(p - lag_points.get(col, 0), 0), n_points - 1)
            value = int(round(gain * code[src])) + int(round(rng.gauss(0.0, noise)))
            for pc, first, count, delta in (planted or []):
                if pc == col and first <= p < first + count:
                    value += delta
            if held is not None and held[0] == col:
                value = (value & ~(1 << held[1])) | (held[2] << held[1])
            cells.append('%d:%d' % (ch, value))
        lines.append('S,%d,%d,%s' % (p * decim, code[p], ','.join(cells)))

    def stamp(smp):
        return int(round(smp * 1e6 / live_plot.SAMPLE_RATE_HZ)) + origin_us

    for n, (first, width, height) in enumerate(pulses):
        i0   = max(first - 1, 0) * decim                        # the window: the pulse
        i1   = min(first + width, n_points - 1) * decim         # plus a point either side
        apex = max(code[first:first + width] or [baseline])
        lines.append('Event #%d  VALID     reason=0 OK  window=%d..%dus/%dsmp  '
                     'primary=ch8  mode=linked'
                     % (n + 1, stamp(i0), stamp(i1), i1 - i0))
        lines.append('  ch0  max=%d  area=0  width=%dus/%dsmp  base=%d  VALID'
                     % (apex, i1 - i0, i1 - i0, baseline))
        lines.append('  ch8  max=%d  area=0  width=%dus/%dsmp  base=%d  VALID'
                     % (apex, i1 - i0, i1 - i0, baseline))
    return lines


def load_text(lines, range_v):
    """Feed synthetic or captured text through the same parser load() uses."""
    cap    = Capture()
    parser = live_plot.StreamParser(cap, 'selftest', range_v, echo=False)
    for line in lines:
        parser.feed_line(line)
    parser.flush()
    return cap


def selftest():
    """A capture with defects planted in it, so the measurements themselves are checked."""
    pulses = [(40, 12, 900), (120, 30, 3000), (220, 8, 1500)]
    checks = []

    def check(cond, what, detail=''):
        checks.append((bool(cond), what, detail))

    def measure_cap(**kw):
        cap = load_text(synthetic_text(pulses, **kw), 10.0)
        return cap, analyse(cap, [0, 1], 16)

    def measure(**kw):
        return measure_cap(**kw)[1]

    # 1. a clean run on the sample grid: nothing to see, so nothing may be reported
    clean = measure()
    off   = statistics.median([r['offset'] for r in clean['rows']])
    check(clean['rows'] and abs(off) <= live_plot.DECIM,
          'a clean capture places its windows on their pulses', 'offset %.1f smp' % off)
    check(all(f['lag'] == 0 for f in clean['fits'].values()),
          'a clean capture reports no channel lag',
          {c: f['lag'] for c, f in clean['fits'].items()})
    check(not any(clean['steps'].values()), 'a clean capture reports no steps',
          clean['steps'])

    # 2. an uptime stamp: every window moves, and they all move together
    origin = measure(origin_us=20000)
    offs   = [r['offset'] for r in origin['rows']]
    check(abs(statistics.median(offs) - 200) <= live_plot.DECIM,
          'an uptime-stamped capture moves the windows by origin/100 samples',
          'median %.1f smp, wanted 200' % statistics.median(offs))
    check(max(offs) - min(offs) <= live_plot.DECIM,
          '... and moves them together, so the shift is read as an origin',
          'spread %.1f smp' % (max(offs) - min(offs)))

    # 3. a four-point step on ch0, mid-pulse: the data-line signature
    step = measure(planted=[(0, 46, 4, -1024)])
    hit  = next((s for s in step['steps'][0] if abs(s['level'] + 1024) <= 100), None)
    check(hit is not None, 'a 4-point step on ch0 is found', step['steps'][0])
    if hit:
        check(power_of_two(hit['level']) == (10, 1024),
              '... its size is matched to the data line it would be', hit['level'])
        check(any(r['start'] <= hit['first_index'] <= r['end'] for r in step['rows']),
              '... and it is placed inside a pulse window')
    check(not step['steps'][1], 'the other channel stays clean', step['steps'][1])

    # 4. one point is a spike, not a step
    spike = measure(planted=[(0, 46, 1, -1024)])
    check(not spike['steps'][0], 'a lone point is not reported as a step',
          spike['steps'][0])

    # 5. a deviation that is no power of two is a shape, not a bit
    shape = measure(planted=[(0, 46, 4, 300)])
    run   = next((s for s in shape['steps'][0] if abs(s['level']) > 100), None)
    check(run is not None and power_of_two(run['level']) is None,
          'a non-power-of-two deviation is reported as a shape', run)

    # 6. a channel that reads the code a point late is measured as that
    late = measure(lag_points={1: 1})
    check(late['fits'][0]['lag'] == 0 and late['fits'][1]['lag'] == 1,
          'a one-point lag is measured on the channel that has it',
          {'ch0': late['fits'][0]['lag'], 'ch8': late['fits'][1]['lag']})

    # 7. one data line held: that word is the other word with one bit forced, and
    #    saying *which* bit is the whole point - an offset of the same size does not
    #    follow the input's bit, so it must not be called a line
    stuck = measure(held=(0, 6, 1))
    bit   = stuck['pairs'][0]['held']
    check(bit is not None, 'a held data line on ch0 is found', stuck['pairs'][0])
    check(bit is not None and (bit['col'], bit['bit'], bit['value']) == (0, 6, 1),
          '... as column 0, bit 6, held at 1',
          bit and (bit['col'], bit['bit'], bit['value']))
    check(bit is not None and bit['held_high'] > 0 and bit['follows'] > 0,
          '... in the points the input takes that bit both ways',
          bit and (bit['held_high'], bit['follows']))

    # 8. and the same the other way round, held low, on the second column
    low = measure(held=(1, 7, 0))
    bit = low['pairs'][0]['held']
    check(bit is not None and (bit['col'], bit['bit'], bit['value']) == (1, 7, 0),
          'a held-low line is found on the column that has it', bit)

    # 9. an offset of exactly one bit's worth is a gap, not a data line
    gap = measure(planted=[(0, 0, 600, 4096)])
    check(gap['pairs'][0]['held'] is None,
          'a fixed 4096-count offset is not called a held line',
          gap['pairs'][0]['held'])
    check(gap['pairs'][0]['gap'] is not None and gap['pairs'][0]['gap'][0] == 12,
          '... it is called a gap of 2^12 counts', gap['pairs'][0]['gap'])

    # 10. a clean pair is not accused of anything
    check(clean['pairs'][0]['held'] is None and clean['pairs'][0]['gap'] is None,
          'a clean capture holds no bit and shows no gap', clean['pairs'][0])

    # 11. a bit names a wire: the pin comes out of config.h's own table
    lines = db_line_names()
    check(len(lines) == 32, 'config.h names all 32 data lines', len(lines))
    check(lines[:1] == ['D22'] and lines[15:16] == ['D37'],
          '... one module per 16 entries, in bit order', (lines[:1], lines[15:16]))

    # 12. the counts per code, against the strap --range names
    cap10, found10 = measure_cap()
    said = ' '.join(scale_lines(cap10, found10, [0, 1], 10.0))
    check('agree' in said, 'a +/-10 V gain reads as the strap --range names', said)
    cap5, found5 = measure_cap(gain=5.28)
    said = ' '.join(scale_lines(cap5, found5, [0, 1], 10.0))
    check('+/-5 V strap' in said, 'a gain of 2x is named as the +/-5 V strap', said)

    failed = [c for c in checks if not c[0]]
    for ok, what, detail in checks:
        print('  %s %s%s' % ('ok  ' if ok else 'FAIL', what,
                             '' if ok or not detail else '  <- %r' % (detail,)))
    print('%s: %d check(s), %d failed'
          % ('FAIL' if failed else 'PASS', len(checks), len(failed)))
    return 1 if failed else 0


# ===========================================================================
# Command line
# ===========================================================================
def main(argv=None):
    p = argparse.ArgumentParser(
        description='Measure a live_plot.py capture: where its windows sit, and what '
                    'its channels do.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='examples:\n'
               '  py -3 test/live_plot.py --port COM6 --log run.txt\n'
               '  py -3 test/analyze_capture.py run.txt\n'
               '  py -3 test/analyze_capture.py run.txt --channel 0 --events 12\n'
               '  py -3 test/analyze_capture.py --selftest')
    p.add_argument('capture', nargs='?',
                   help='the file live_plot.py --log wrote')
    p.add_argument('--selftest', action='store_true',
                   help='no capture: build one with defects planted in it and check the '
                        'measurements against it')
    p.add_argument('--channel', type=int, action='append', metavar='N',
                   help='AD7606 channel to measure (repeatable; default: every streamed '
                        'column)')
    p.add_argument('--events', type=int, default=8,
                   help='event rows to print (default %(default)s)')
    p.add_argument('--steps', type=int, default=3,
                   help='step runs to print per channel (default %(default)s)')
    p.add_argument('--min-run', type=int, default=2, metavar='POINTS',
                   help='points a deviation has to hold for to count as a step rather '
                        'than a spike (default %(default)s; 1 shows every spike too)')
    p.add_argument('--max-lag', type=int, default=16, metavar='POINTS',
                   help='search this many points either side for the channel/code lag '
                        '(default %(default)s)')
    p.add_argument('--range', dest='range_v', type=float, default=10.0,
                   help='AD7606 full-scale input in volts (default %(default)s)')
    args = p.parse_args(argv)

    if args.selftest:
        return selftest()
    if not args.capture:
        p.error('give a capture file, or --selftest')

    cap = load(args.capture, args.range_v)
    if not cap.idx:
        raise SystemExit('no sample lines in %s - was the run logged with --log?  This '
                         "tool reads live_plot.py's own capture, not a monitor's text "
                         'with the event blocks only.' % args.capture)

    cols = list(range(len(cap.cols)))
    if args.channel:
        wanted = [cap.column_of(ch) for ch in args.channel]
        cols   = [c for c in wanted if c is not None]
        if not cols:
            raise SystemExit('this capture streams %s, so --channel %s is not in it'
                             % (', '.join('ch%d' % ch for ch in cap.channels),
                                ', '.join('ch%d' % ch for ch in args.channel)))

    found = analyse(cap, cols, args.max_lag, min_run=max(1, args.min_run))
    for line in report(cap, found, cols, args):
        print(line)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
