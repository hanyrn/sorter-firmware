#!/usr/bin/env python3
"""detector_audit.py - ground-truth audit of the EventDetector against the model.

OUT OF DATE: it drives the detector mirror in model_check.py, which mirrors the window
rules (band breakout, duration) but not the four-phase machine underneath (rise test,
slope-bounded path, suppression), so its scores describe that approximation and not the
firmware.  test/test_bench_detect.cpp does the same audit against the real C++ (and
`--sweep` prints the candidate configurations).

test/model_check.py runs the detector mirror over a *hand-written* signal and checks
a handful of known cases.  This tool does the opposite: it generates the bench
waveform (signal_model.py, the same code the M4 runs), digitizes it the way
test/live_plot.py's simulator does, runs the detector over it, and compares every
closed event with the model's own truth - which pulse was there, and what the
noise-free apex of that pulse was.  That is what turns "it rejected a peak" into a
number, and it is how the shape guards in event_detector.h were tuned.

Reading the report (per candidate configuration):

  genuine spans   the pulses the detector is expected to report (a merged pair of
                  overlapping pulses counts as one span, like the model's own
                  event_spans())
  MISSED          a genuine span whose apex no VALID event covers.  The very first
                  span of a run lands inside the 3000-sample warm-up, so it is always
                  one of these.
  GHOST           a VALID event that covers no genuine pulse at all: the false
                  accepts (a flat rectangular glitch, or a lone impulse).
  fragments       events over a genuine span that do not contain its apex.  A pair of
                  overlapping pulses that really do dip below the off-threshold
                  between the two humps legitimately produces two of them; a rejected
                  one is just log noise.
  apex max err    reported max minus the noise-free apex, on the events that do cover
                  an apex.  This is the "is the maximum the signal's or the spike's?"
                  number: with the input filter off it reaches +1000 counts (an
                  impulse), with it on it is the sampled dither (~+30, i.e. the
                  highest of the noise samples near the apex - which *is* the maximum
                  of the digitized signal).

Usage:
    py -3 test/detector_audit.py [cycles]        # 1 cycle = 2.0 s at 10 kSPS
"""

import collections
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import live_plot as lp             # noqa: E402  (the sim's digitization, kept in sync)
import model_check as mc          # noqa: E402  (the detector mirror)
import signal_model as sm         # noqa: E402  (the waveform mirror)

DT_US    = 100
LUT_SIZE = 256
LUT = [0.5 * (1.0 - math.cos(2.0 * math.pi * i / LUT_SIZE)) for i in range(LUT_SIZE)]

BASE_CODE = sm.DEFAULTS['baseline_code']
MIN_CODE  = sm.DEFAULTS['min_code']
MAX_CODE  = sm.DEFAULTS['max_code']
REASON    = {v: k for k, v in vars(mc).items() if k.startswith('REASON_')}
_spare    = []


def reasons(counter):
    return {REASON.get(k, k): v for k, v in counter.items()}


def apd():
    """AD7606 counts per DAC code for the sim's default range strap."""
    return lp.gen_scale_counts(10.0)


def random_gauss(model):
    """One standard normal per call, from Box-Muller pairs (as SimSource does)."""
    if not _spare:
        _spare.extend(model._gauss_pair())
    return _spare.pop(0)


def clean_code(train, i):
    """The DAC code for cycle sample i with pulses only: no dither, no impulse.

    This is the "actual distribution" behind a sample - the reference the reported
    maximum is compared against.
    """
    value = float(BASE_CODE)
    for p in train:
        if i < p['start'] or i >= p['start'] + p['width']:
            continue
        amp = p['amplitude'] / apd()
        if p['kind'] == sm.RECTANGLE:
            value += amp
        else:
            u = ((i - p['start']) * LUT_SIZE) // p['width']
            if u >= LUT_SIZE:
                u = LUT_SIZE - 1
            value += amp * LUT[u]
    return min(float(MAX_CODE), max(float(MIN_CODE), value))


class Record:
    """One run: the digitized samples plus the truth behind every sample."""

    def __init__(self, cycles, seed=None):
        model = sm.SignalModel(adc_per_dac=apd(), **({'seed': seed} if seed else {}))
        model.begin()
        noise = sm.SignalModel(seed=lp.SIM_SEED)   # only its Gaussian pairs, as SimSource
        self.cycles = cycles
        self.counts, self.clean, self.spike = [], [], []
        self.pulses = []                           # every pulse, global sample index
        self.spans = []                            # grouped pulses (one event each)
        for _ in range(cycles):
            base = len(self.counts)
            train = None
            for i in range(model.cycle_samples):
                code = model.tick()
                if i == 0:
                    train = [dict(p) for p in model.pulses]
                self.spike.append(model.spike * apd())
                self.counts.append(lp.volts_to_counts(code * lp.DAC_VREF / lp.DAC_MAX_CODE,
                                                      10.0)
                                   + int(round(random_gauss(noise) * lp.SIM_ADC_NOISE)))
                self.clean.append(lp.volts_to_counts(clean_code(train, i) * lp.DAC_VREF
                                                     / lp.DAC_MAX_CODE, 10.0))
            for p in train:
                q = dict(p)
                q['start'] += base
                self.pulses.append(q)
            for (s, e, n) in sm.event_spans(train):
                self.spans.append((s + base, e + base, n))

    def span_kind(self, start, end):
        """'peak' / 'glitch' / 'impulse' for a window."""
        inside = [p for p in self.pulses
                  if p['start'] <= end and p['start'] + p['width'] >= start]
        if not inside:
            return 'impulse'
        return 'peak' if any(p['kind'] == sm.RAISED_COSINE for p in inside) else 'glitch'

    def true_peak_of(self, span):
        """(noise-free apex in counts, sample it happens at) of one truth span."""
        s, e, _ = span
        apex = -1
        at = s
        for i in range(s, e + 1):
            v = self.clean[i] - self.clean[0]
            if v > apex:
                apex, at = v, i
        return apex, at

    def apex_in(self, start, end):
        return max(self.clean[start:end + 1]) - self.clean[0]


# ---------------------------------------------------------------------------
# Running and scoring
# ---------------------------------------------------------------------------
class Ev:
    """One closed event, plus the shape features the ladder works on."""

    def __init__(self, start, close, peak, m):
        self.start, self.close, self.peak, self.m = start, close, peak, m

    @property
    def rise(self):
        return self.m['rise_samples']


def run(counts, cfg=None, ladder=None):
    """Run the mirror over `counts`; returns the events and the detector.

    `ladder` re-judges every event with a different shape rule (see verdict()), so
    the report can compare what the guards do - including the rule that used to be
    there - on exactly the same measurements.
    """
    det = mc.EventDetector(cfg if cfg is not None else mc.Config())
    evs = []
    for i, x in enumerate(counts):
        m = det.update(x, i * DT_US, i)
        if m is not None:
            evs.append(Ev(det.ev_start_index, i, m['peak_us'] // DT_US, m))
    if ladder is not None:
        for e in evs:
            e.m['reason'] = verdict(e, det.cfg, *ladder)
            e.m['valid'] = 1 if e.m['reason'] == mc.REASON_OK else 0
    return evs, det

def verdict(e, cfg, shape='shipped', min_rise=None):
    """Re-apply a ladder to a measured event, so variants can be compared.

      shipped    max_value / width_samples > max_slope   (the rule that was there
                 before: judged the *average* slope over the whole width)
      rise_min   rise_samples < min_rise                 (a step is not a peak)
      both       rise_min and max_value / rise > max_slope
    """
    if e.m['max_value'] < cfg.min_max_value:
        return mc.REASON_MAX_TOO_SMALL
    if e.m['width_samples'] < cfg.min_width_samples:
        return mc.REASON_WIDTH_TOO_NARROW
    if e.m['width_samples'] > cfg.max_width_samples:
        return mc.REASON_WIDTH_TOO_WIDE
    if e.m['area'] < cfg.min_area:
        return mc.REASON_AREA_TOO_SMALL
    rise = max(1, e.rise)
    if shape in ('rise_min', 'both') and e.rise < (min_rise or cfg.min_rise_samples):
        return mc.REASON_SHORT_FOR_HEIGHT
    if shape in ('rise_slope', 'both') and e.m['max_value'] / float(rise) > cfg.max_slope:
        return mc.REASON_SHORT_FOR_HEIGHT
    if shape == 'shipped' and \
            e.m['max_value'] / float(max(1, e.m['width_samples'])) > cfg.max_slope:
        return mc.REASON_SHORT_FOR_HEIGHT
    return mc.REASON_OK


def score(rec, label, evs):
    """Score one run: missed peaks, ghost accepts, fragments and max error."""
    apexes = [rec.true_peak_of(s) for s in rec.spans
              if rec.span_kind(s[0], s[1]) == 'peak']
    has_apex = lambda e: any(e.start <= at <= e.close for (_, at) in apexes)

    missed, errs = 0, []
    for (apex, at) in apexes:
        hit = [e for e in evs if e.m['valid'] and e.start <= at <= e.close]
        if hit:
            errs.append(max(e.m['max_value'] for e in hit) - apex)
        else:
            missed += 1

    ghost, frag = collections.Counter(), collections.Counter()
    for e in evs:
        if has_apex(e):
            continue
        kind = rec.span_kind(e.start, e.close)
        if kind == 'peak':
            frag['valid' if e.m['valid'] else 'rejected'] += 1
        elif e.m['valid']:
            ghost[kind] += 1

    s = sorted(errs) if errs else [0]
    print('\n== %-30s == %4d events' % (label, len(evs)))
    print('   genuine spans %d: apex covered %d  MISSED %d'
          % (len(apexes), len(apexes) - missed, missed))
    print('   apex max err (counts): mean %+.1f  p90 %+d  max %+d'
          % (sum(errs) / max(1, len(errs)), s[int(len(s) * 0.9)], s[-1]))
    print('   GHOST (valid, no pulse): %-3d %s' % (sum(ghost.values()), dict(ghost)))
    print('   fragments (no apex): valid %d  rejected %d'
          % (frag['valid'], frag['rejected']))
    return dict(label=label, missed=missed, ghost=sum(ghost.values()),
                frag=frag['valid'], fp=sum(ghost.values()) + frag['valid'])


def shape_stats(rec, evs, src, title):
    """How the two shapes (a pulse, a flat glitch) differ on the edge."""
    groups = collections.defaultdict(list)
    for e in evs:
        groups[rec.span_kind(e.start, e.close)].append(e)
    print('\n-- shape features (%s): rise = apex - event start' % title)
    for k in ('peak', 'glitch', 'impulse'):
        g = groups.get(k)
        if not g:
            continue
        rises = sorted(e.rise for e in g)
        frac = sorted((src[e.start] - src[0]) / float(max(1, e.m['max_value'])) for e in g)
        print('   %-8s n=%-4d rise min/p50/max = %d/%d/%d   first sample / max = %.2f/%.2f/%.2f'
              % (k, len(g), rises[0], rises[len(rises) // 2], rises[-1],
                 frac[int(len(frac) * 0.1)], frac[len(frac) // 2], frac[int(len(frac) * 0.9)]))


def detail(rec, evs, label):
    """Explain the leftover misses / splits, one line per window."""
    print('\n---- detail: %s ----' % label)
    for span in rec.spans:
        s, e, n = span
        if rec.span_kind(s, e) != 'peak':
            continue
        apex, at = rec.true_peak_of(span)
        inside = [ev for ev in evs if ev.start <= e and ev.close >= s]
        hit = [ev for ev in inside if ev.m['valid'] and ev.start <= at <= ev.close]
        if len(hit) == 1 and len(inside) == 1:
            continue
        print('  span %d..%d (%d samples, %d pulse(s)) apex=%d at %d'
              % (s, e, e - s, n, apex, at))
        for ev in inside:
            print('    event %d..%d  reason=%-22s rise=%-4d max=%-6d width=%-5d%s'
                  % (ev.start, ev.close, reasons({ev.m['reason']: 1}).popitem()[0],
                     ev.rise, ev.m['max_value'], ev.m['width_samples'],
                     '  <-- has the apex' if ev.start <= at <= ev.close else ''))


def main():
    cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rec = Record(cycles)
    n_peak = sum(1 for s in rec.spans if rec.span_kind(s[0], s[1]) == 'peak')
    n_glitch = sum(1 for s in rec.spans if rec.span_kind(s[0], s[1]) == 'glitch')
    n_imp = sum(1 for s in rec.spike if s != 0.0)
    print('samples %d (%.0f s at 10 kSPS)  seed 0x%X  %.3f counts/DAC code  baseline %d counts'
          % (len(rec.counts), len(rec.counts) / 10000.0, sm.DEFAULTS['seed'],
             apd(), rec.clean[0]))
    print('truth: %d pulses -> %d spans (%d with a raised cosine, %d glitch-only), '
          '%d impulses (%.3f%% of samples)'
          % (len(rec.pulses), len(rec.spans), n_peak, n_glitch, n_imp,
             100.0 * n_imp / len(rec.spike)))

    cands = [
        # label                              cfg overrides                ladder
        ('old ladder, filter off',            {'impulse_filter_taps': 1},  ('shipped', None)),
        ('old ladder, filter taps=3',         {'impulse_filter_taps': 3},  ('shipped', None)),
        ('taps=7, rise_min=4',                {},                          ('rise_min', 4)),
        ('taps=7, rise_min=6 (shipped now)',  {},                          None),
        ('taps=5, rise_min=6',                {'impulse_filter_taps': 5},  None),
        ('taps=9, rise_min=6',                {'impulse_filter_taps': 9},  None),
        ('taps=7, rise>=4 + slope 100',       {},                          ('both', 4)),
    ]
    summary = []
    for (label, overrides, ladder) in cands:
        cfg = mc.Config()
        for k, v in overrides.items():
            setattr(cfg, k, v)
        evs, _ = run(rec.counts, cfg, ladder)
        summary.append(score(rec, label, evs))

    evs, det = run(rec.counts)
    shape_stats(rec, evs, rec.counts, 'as shipped (taps=7, rise_min=6)')
    print('   baseline %.1f counts, sigma %.2f counts (both of the filtered stream)'
          % (det.mean, det.sigma))
    detail(rec, evs, 'as shipped (taps=7, rise_min=6)')

    print('\n================ summary ================')
    print('%-32s %6s %6s %8s %6s' % ('candidate', 'MISSED', 'GHOST', 'fragVal', 'fp'))
    for r in summary:
        print('%-32s %6d %6d %8d %6d'
              % (r['label'], r['missed'], r['ghost'], r['frag'], r['fp']))


if __name__ == '__main__':
    main()


