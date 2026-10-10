#!/usr/bin/env python3
"""
Graph the voltage waveform that the AD7606 modules will digitize - before the
hardware is on the bench.

Two selectable sources:

  dac    (default) the exact waveform the M4 drives out of its two 12-bit DACs
         in the bench loopback (SORTER_SIGNAL_GEN = 1).  This is a mirror of
         signal_model.h - test/signal_model.py is that mirror: the same randomly
         drawn pulse train, the same 256-entry raised-cosine LUT and the same
         xorshift dither.  In the loopback those DAC pins are wired straight into
         the AD7606 inputs, so this is literally the voltage the modules will
         sample.  Two pattern cycles are run and the second is plotted, so the
         detector is past its 3000-sample warmup; a fresh train is drawn for every
         2.0 s cycle, and the one the plotted cycle uses is listed on stdout.

  model  the synthetic detector cross-check signal from model_check.py
         (baseline + Gaussian noise + three broad peaks + two glitches).

For either source the EventDetector mirror in model_check.py is run over the
waveform and the detector's own verdicts (VALID, or rejected + reason) are
shaded on the plot.  So you can see the signal, the slowly tracked baseline, the
k_on threshold and which excursions the firmware will accept - all with no
hardware and no third-party Python packages.

The result is a single self-contained HTML file (inline SVG, no external assets)
that opens in any browser or in VS Code (right-click -> Open with Live Server,
or Simple Browser).

Run:  py -3 test/plot_signals.py
      py -3 test/plot_signals.py --source model
      py -3 test/plot_signals.py --ladder            # the floor ladder, rung by rung
      py -3 test/plot_signals.py --start live        # pulses through the quiet period
      py -3 test/plot_signals.py --range 5           # AD7606 RANGE strap = +/-5 V
      py -3 test/plot_signals.py --out test/signals.html

The run prints a short text summary of the detected events and then opens the
generated file in your default browser (a terminal cannot draw the chart itself).
Pass --no-open to skip the launch.
"""

import argparse
import math
import os
import sys
import webbrowser

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import model_check as mc           # noqa: E402  (detector mirror + reason codes)
import signal_model as sm          # noqa: E402  (waveform mirror of signal_model.h)

# --- constants mirrored from config.h ---------------------------------------
SAMPLE_RATE_HZ = 10000             # SORTER_SAMPLE_RATE_HZ
DT_US          = 1000000 // SAMPLE_RATE_HZ

# The waveform itself (all the SIGNALGEN_* knobs in config.h) lives in exactly one
# place: test/signal_model.py, the Python mirror of signal_model.h.  Only the cycle
# length and the model_check.py signal length are needed here.
CYCLE_SAMPLES  = sm.DEFAULTS['cycle_samples']   # SIGNALGEN_CYCLE_SAMPLES
MODEL_SAMPLES  = 26000             # model_check.py main() length (covers its peak @ 20000)

# The GIGA's 12-bit DAC: 0..4095 over the ~3.3 V VDDA rail.
DAC_MAX_CODE   = 4095
DAC_VREF       = 3.3

# AD7606 is 16-bit signed: +/-32768 counts across the selected full scale.
AD7606_FULL_SCALE_COUNTS = 32768

REASON_NAME = {
    mc.REASON_OK:               'VALID',
    mc.REASON_MAX_TOO_SMALL:    'rejected: max too small',
    mc.REASON_WIDTH_TOO_NARROW: 'rejected: width too narrow',
    mc.REASON_WIDTH_TOO_WIDE:   'rejected: width too wide',
    mc.REASON_AREA_TOO_SMALL:   'rejected: area too small',
    mc.REASON_SHORT_FOR_HEIGHT: 'rejected: no leading edge (a step)',
}
REASON_SHORT = {
    mc.REASON_OK:               'VALID',
    mc.REASON_MAX_TOO_SMALL:    'max too small',
    mc.REASON_WIDTH_TOO_NARROW: 'too narrow',
    mc.REASON_WIDTH_TOO_WIDE:   'too wide',
    mc.REASON_AREA_TOO_SMALL:   'area too small',
    mc.REASON_SHORT_FOR_HEIGHT: 'no leading edge',
}

# (The fixed `kPattern` table that used to live here is gone: the train is drawn at
# run time by SignalModel::buildTrain(), so test/signal_model.py is the only place
# the waveform is described in Python.)


def adc_per_dac(range_v):
    """AD7606 counts per DAC code for this RANGE strap.

    This is the number the firmware's `SIGNALGEN_ADC_PER_DAC` has to be built with
    (2.64 at +/-10 V, 5.28 at +/-5 V - see config.h): the model works in counts, so
    it divides an amplitude by this to get the DAC code.  Get it wrong for the strap
    and every peak is measured at the wrong height.
    """
    return (DAC_VREF / DAC_MAX_CODE) * (AD7606_FULL_SCALE_COUNTS / range_v)


def build_dac_codes(cycles=2, range_v=10.0, ladder=False):
    """The DAC codes (0..4095) the M4 writes for `cycles` pattern cycles.

    Returns (model, codes); `model.pulses` is the train used for the last cycle.
    `ladder` draws the floor ladder instead of the random train (SIGNALGEN_LADDER_*):
    isolated pulses of one width, apexes stepping up from below the detector's floor.

    This is the *raw* model with no harness rule applied - see build_loopback(), which is
    what main() uses.
    """
    return sm.build_cycles(cycles, sm.SignalModel(adc_per_dac=adc_per_dac(range_v),
                                                  ladder=1 if ladder else 0))


def pulse_code(model, train, index):
    """The DAC-code contribution of `train` at sample `index`, added exactly the way
    SignalModel.sample_code() adds it (counts -> codes, same LUT)."""
    value = 0.0
    for p in train:
        if index < p['start'] or index >= p['start'] + p['width']:
            continue
        amp = p['amplitude'] / float(model.adc_per_dac)
        if p['kind'] == sm.RECTANGLE:
            value += amp
        else:
            u = ((index - p['start']) * sm.LUT_SIZE) // p['width']
            if u >= sm.LUT_SIZE:
                u = sm.LUT_SIZE - 1
            value += amp * model.lut[u]
    return value


def build_loopback(cycles, range_v, ladder=False, start='hold'):
    """The DAC codes for `cycles` pattern cycles, with the harness's quiet-period rule
    applied - the bench's own `--start`, and the choice that decides what the detector's
    reference is seeded from:

      hold  (default) the detector's contract: nothing happens during its quiet period,
            so the level it locks onto when detection arms is the *noise*.  The model
            draws pulses from sample 0, so every pulse group that overlaps
            DET_QUIET_SAMPLES is not injected at all in the first cycle - for the group's
            whole span, and only the *train* is dropped: the dither, the impulses and the
            slow wander the reference is meant to follow all stay.  The generator keeps
            ticking, so cycle 1's train - the one that gets plotted - is exactly the train
            it would have drawn anyway.  This is what test_bench_detect.exe does by
            default, and the mode the floor ladder's table is measured in.

      live  the pulses run from sample 0: the board's boot reality, in which the quiet
            period is contaminated and the reference it seeds is the signal's, not the
            noise's.  The floor moves up with it (see test/live_plot.py --sim-start,
            which forwards this same flag to the harness).

    Returns (model, codes), like signal_model.build_cycles(): `model.pulses` is the train
    in use for the *last* cycle.
    """
    model = sm.SignalModel(adc_per_dac=adc_per_dac(range_v), ladder=1 if ladder else 0)
    model.begin()
    train0 = list(model.pulses)                 # begin() drew cycle 0's train
    quiet = mc.Config().quiet_samples           # DET_QUIET_SAMPLES

    # Every pulse group that overlaps the quiet period is held off (see above) - for the
    # group's whole span, so a merged pair cannot be left half-drawn.  Same rule, and the
    # same grouping, as test_bench_detect.cpp.
    hold_until = 0
    if start == 'hold':
        hold_until = quiet
        for (s, e, _n) in sm.event_spans(train0):
            if s < quiet and e > hold_until:
                hold_until = e

    codes = []
    for i in range(cycles * model.cycle_samples):
        code = model.tick()
        if i < hold_until:
            # The code fed to the converter is the model's own sample with the *train*
            # subtracted: the pulses are held off; the dither, the impulses and the slow
            # wander stay, because those are what the quiet period is meant to measure.
            # (Holding the wander off too would put a step in the trace where the hold
            # ends, and a step of that size is outside the tracker's gate.)
            code = code - int(round(pulse_code(model, train0, i)))
        codes.append(code)
    return model, codes


def codes_to_volts(codes):
    """DAC output voltage == the AD7606 input voltage in the loopback."""
    return [c * DAC_VREF / DAC_MAX_CODE for c in codes]


def volts_to_counts(volts, range_v):
    """What the AD7606 actually outputs, given its RANGE strap full scale."""
    return [int(math.floor(v * (AD7606_FULL_SCALE_COUNTS / range_v) + 0.5))
            for v in volts]


def frontend_counts(counts, taps):
    """The analog path in front of each module (mirror of test_bench_detect.cpp's
    `frontend`): the AD7606 does not see the DAC's step edges, it sees a few samples of
    them - the DAC and the converter's anti-alias filter do not settle inside one 100 us
    sample period, so a moving average of `taps` samples is what the converter's own
    sample looks like.  1 = off, the model's own sample.  The bench's default is 3, and it
    matters here: the front end is what sets the noise sigma the detector sees, so it sets
    the floor (3 taps measured sigma 14.8 counts on the floor ladder against 23.7 at 1)."""
    if taps <= 1:
        return list(counts)
    window = [counts[0]] * taps          # the bench primes its ring with the baseline
    out = []
    for c in counts:
        window.append(c)
        window.pop(0)
        out.append(int(round(sum(window) / float(taps))))
    return out


def build_model_signal():
    """model_check.py's synthetic signal, in AD7606 counts (baseline 2000)."""
    return [x for (x, _t) in mc.build_signal(MODEL_SAMPLES, DT_US)]


def counts_to_volts(counts, range_v):
    return [c * range_v / AD7606_FULL_SCALE_COUNTS for c in counts]


def run_detector(counts, dt_us):
    """Run the C++ detector mirror; return closed events with their sample span."""
    det = mc.EventDetector(mc.Config())
    events = []
    for i, x in enumerate(counts):
        m = det.update(x, i * dt_us, i)
        if m is not None:
            # The event's window is its own start - the sample the trace left the noise
            # band, which is what PeakMetrics::start_us carries - to the sample it closed
            # on.  Not `end - width_samples`: that is only the same sample when the close
            # lands exactly on the first sample of the run back into the band.
            events.append({'start': m['start_us'] // dt_us, 'end': i, 'm': m})
    return events, det


def _esc(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def render_chart(volts, events, range_v, i0, i1, title, subtitle='',
                 width=1140, height=340, labels=False):
    """One SVG chart of volts[i0..i1] with the detector's own verdicts shaded."""
    ml, mr, mt, mb = 66, 16, 40, 40
    pw, ph = width - ml - mr, height - mt - mb

    vis = volts[i0:i1 + 1]
    lo, hi = min(vis), max(vis)
    span = max(hi - lo, 1e-3)
    lo -= span * 0.10
    hi += span * 0.10

    def X(i):
        return ml + (i - i0) / float(max(i1 - i0, 1)) * pw

    def Y(v):
        return mt + (hi - v) / (hi - lo) * ph

    def c2v(c):                                    # AD7606 counts -> volts
        return c * range_v / AD7606_FULL_SCALE_COUNTS

    p = ['<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
         'viewBox="0 0 %d %d" font-family="Segoe UI,Arial,sans-serif">'
         % (width, height, width, height)]
    p.append('<text x="%d" y="18" font-size="13" font-weight="600" fill="#111">%s</text>'
             % (ml, _esc(title)))
    if subtitle:
        p.append('<text x="%d" y="33" font-size="11" fill="#666">%s</text>'
                 % (ml, _esc(subtitle)))
    p.append('<rect x="%d" y="%d" width="%d" height="%d" fill="#fbfbfb" stroke="#ddd"/>'
             % (ml, mt, pw, ph))

    # horizontal gridlines + volt labels
    for k in range(5):
        v = lo + (hi - lo) * k / 4.0
        y = Y(v)
        p.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#ececec"/>'
                 % (ml, y, ml + pw, y))
        p.append('<text x="%d" y="%.1f" font-size="10" fill="#555" text-anchor="end" '
                 'dominant-baseline="middle">%.3f V</text>' % (ml - 6, y, v))

    # vertical gridlines + time labels (ms)
    step = max(1, (i1 - i0) // 8)
    for i in range(i0, i1 + 1, step):
        x = X(i)
        p.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#f2f2f2"/>'
                 % (x, mt, x, mt + ph))
        p.append('<text x="%.1f" y="%d" font-size="10" fill="#555" text-anchor="middle">'
                 '%.0f ms</text>' % (x, mt + ph + 16, i * 1000.0 / SAMPLE_RATE_HZ))

    # event windows, each with its baseline (dashed grey), k_on threshold (dotted) and the
    # noise band the window's ends sit on (dotted, lighter - the off-threshold)
    for ev in events:
        s, e = max(ev['start'], i0), min(ev['end'], i1)
        if e < s:
            continue
        m = ev['m']
        ok = m['valid'] == 1
        fill = 'rgba(46,125,50,0.15)' if ok else 'rgba(198,40,40,0.13)'
        col = '#2e7d32' if ok else '#c62828'
        p.append('<rect x="%.1f" y="%d" width="%.1f" height="%d" fill="%s" '
                 'stroke="%s" stroke-dasharray="3 3"/>'
                 % (X(s), mt, max(X(e) - X(s), 1.5), ph, fill, col))
        yb, yt = Y(c2v(m['baseline'])), Y(c2v(m['threshold']))
        p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#888" '
                 'stroke-dasharray="4 3"/>' % (X(s), yb, X(e), yb))
        p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
                 'stroke-dasharray="2 2"/>' % (X(s), yt, X(e), yt, col))
        # The level both ends of the window sit on: baseline + k_off * sigma, off the
        # event's own frozen reference (m['noise'] is that sigma, rounded - a count of
        # rounding is invisible at plot scale).
        yo = Y(c2v(m['baseline'] + mc.rint(mc.Config().k_off * m['noise'])))
        p.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#7b1fa2" '
                 'stroke-dasharray="1 3"/>' % (X(s), yo, X(e), yo))
        if labels:
            mid = (X(s) + X(e)) / 2.0
            p.append('<text x="%.1f" y="%d" font-size="11" font-weight="600" fill="%s" '
                     'text-anchor="middle">%s</text>'
                     % (mid, mt - 4, col, 'VALID' if ok else _esc(REASON_SHORT[m['reason']])))
            p.append('<text x="%.1f" y="%d" font-size="10" fill="#666" '
                     'text-anchor="middle">max=%d  win=%d smp  dur=%d smp</text>'
                     % (mid, mt + 12, m['max_value'], m['width_samples'],
                        m['duration_samples']))

    # the trace (subsampled on the overview so the SVG stays small)
    stride = max(1, (i1 - i0) // 4000)
    idx = list(range(i0, i1 + 1, stride))
    if idx[-1] != i1:
        idx.append(i1)
    pts = ' '.join('%.1f,%.1f' % (X(i), Y(volts[i])) for i in idx)
    p.append('<polyline points="%s" fill="none" stroke="#1565c0" stroke-width="1.1"/>' % pts)

    p.append('<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#bbb"/>'
             % (ml, mt, pw, ph))
    p.append('</svg>')
    return '\n'.join(p)


HTML_HEAD = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>AD7606 input signals - sorter firmware</title>
<style>
 body{font-family:'Segoe UI',Arial,sans-serif;margin:24px;color:#222;background:#fff}
 h1{font-size:20px;margin:0 0 4px}
 .meta{color:#555;font-size:13px;margin-bottom:12px;line-height:1.5}
 h2{font-size:14px;color:#333;margin:22px 0 4px;font-weight:600}
 .legend{margin:10px 0 4px;font-size:12px;color:#444}
 .legend span{display:inline-block;margin-right:18px}
 .sw{display:inline-block;width:12px;height:12px;vertical-align:-1px;
     margin-right:4px;border:1px solid #999}
 code{background:#f2f2f2;padding:1px 4px;border-radius:3px}
</style></head><body>
"""


def build_html(charts, meta):
    out = [HTML_HEAD]
    out.append('<h1>AD7606 input signal (before the hardware)</h1>')
    out.append('<div class="meta">%s</div>' % meta)
    out.append('<div class="legend">'
               '<span><i class="sw" style="background:rgba(46,125,50,0.15);'
               'border-color:#2e7d32"></i>detector VALID</span>'
               '<span><i class="sw" style="background:rgba(198,40,40,0.13);'
               'border-color:#c62828"></i>rejected (label = reason)</span>'
               '<span><b style="color:#1565c0">&#9472;</b> signal</span>'
               '<span><b style="color:#888">- -</b> tracked baseline</span>'
               '<span><b style="color:#c62828">&#183;&#183;</b> k_on threshold'
               ' (what decides the excursion is worth reporting)</span>'
               '<span><b style="color:#7b1fa2">&#183;&#183;</b> k_off band'
               ' (where the window opens and closes: win= below)</span>'
               '<span><b>dur=</b> the judged duration (the width the board sends)</span>'
               '</div>')
    for (title, svg) in charts:
        out.append('<h2>%s</h2>' % _esc(title))
        out.append(svg)
    out.append('</body></html>')
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser(
        description='Graph the voltage the AD7606 modules will digitize (no hardware needed).')
    ap.add_argument('--source', choices=['dac', 'model'], default='dac',
                    help='dac = signal_gen.cpp loopback (default); model = model_check.py signal')
    ap.add_argument('--range', type=float, choices=[10.0, 5.0], default=10.0,
                    help='AD7606 RANGE strap full scale in volts (default 10)')
    ap.add_argument('--ladder', action='store_true',
                    help='plot the floor ladder (SIGNALGEN_LADDER_*) instead of the '
                         'random train: isolated pulses of one width, apexes stepping '
                         'up from below the detector\'s floor - one zoomed chart per '
                         'rung, with the verdict on it')
    ap.add_argument('--start', choices=['hold', 'live'], default='hold',
                    help="the harness's quiet period, as test_bench_detect.exe --start: "
                         "hold (default) = the detector's contract, so the pulses that "
                         "overlap DET_QUIET_SAMPLES are not drawn in the first cycle and "
                         "the reference is seeded from noise; live = the pulses run from "
                         "sample 0, the board's boot reality (a higher floor)")
    ap.add_argument('--frontend', type=int, default=3,
                    help="taps of the analog path in front of each module, as "
                         "test_bench_detect.exe --frontend: 1 = off (the model's own "
                         "sample), 3 = the harness's default.  It is what the detector "
                         "sees, so it is what sets the noise sigma, and with it the floor")
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  'signals.html'),
                    help='output HTML path (default: test/signals.html)')
    ap.add_argument('--no-open', action='store_true',
                    help="don't launch the generated file in the default browser")
    args = ap.parse_args()

    train_lines = []
    if args.source == 'dac':
        # Two pattern cycles; analyse the second, so the detector's quiet period is behind
        # it.  `--start hold` (the default) drops the pulses that overlap that period -
        # the harness's own rule, so the reference is the noise and not the rungs (see
        # build_loopback).
        gen, codes = build_loopback(2, args.range, args.ladder, args.start)
        volts_all = codes_to_volts(codes)
        # The detector runs on what the modules digitize: the DAC waveform after the
        # analog front end (see --frontend).  The chart keeps the DAC's own voltage.
        counts_all = frontend_counts(volts_to_counts(volts_all, args.range), args.frontend)
        events_all, det = run_detector(counts_all, DT_US)
        off = CYCLE_SAMPLES
        volts = volts_all[off:off + CYCLE_SAMPLES]
        counts = counts_all[off:off + CYCLE_SAMPLES]
        events = [{'start': e['start'] - off, 'end': e['end'] - off, 'm': e['m']}
                  for e in events_all if e['start'] >= off]
        title = ('DAC loopback (floor ladder, SIGNALGEN_LADDER_*)' if args.ladder
                 else 'DAC loopback (randomized train, signal_model.h)')
        note = ('Voltage is the M4 DAC output, which the AD7606 input is wired to in the '
                'bench loopback (SORTER_SIGNAL_GEN = 1). The second pattern cycle is shown, '
                'so the detector is past its quiet period; the train drawn for that '
                'cycle (SIGNALGEN_SEED = 0x%X) is listed below.'
                % sm.DEFAULTS['seed'])
        note += ('  <b>--start %s</b>: the pulses that overlap the detector\'s quiet period '
                 '(DET_QUIET_SAMPLES) are %s.  The detector runs on the module\'s own '
                 'sample - the DAC voltage after the analog front end, <b>--frontend %d '
                 'tap(s)</b> - because that is what sets the noise sigma, and with it the '
                 'floor.'
                 % (args.start,
                    'not drawn, so it is seeded from noise - this is the harness\'s default '
                    'and the mode the floor is measured in'
                    if args.start == 'hold' else
                    'drawn from sample 0, the board\'s boot reality: the reference then '
                    'follows the signal and the floor moves up with it',
                    args.frontend))
        if args.ladder:
            # The ladder's rungs are the pulses in order, ascending: the chart below is
            # one zoomed window per rung, so the apex the detector answered on can be
            # read against the drawn one - and a rung with no event at all shows which
            # side of the floor it fell on.
            note += ('  <b>Floor ladder</b>: SIGNALGEN_LADDER = 1, so the pattern is '
                     '%d isolated pulses of %d samples (%d ms), apexes stepping by %d '
                     'counts from %d - the smallest apex the detector takes is the '
                     'floor at this width (see config.h).  Each chart shades the window '
                     'the detector measured - band breakout to the return into the band - '
                     'and prints its span (<code>win=</code>, the thinnest line) next to '
                     'the duration the bounds are judged with (<code>dur=</code>, the '
                     'width the board sends, see DET_MIN_WIDTH_SAMPLES in config.h).'
                     % (sm.DEFAULTS['ladder_count'], sm.DEFAULTS['ladder_width'],
                        sm.DEFAULTS['ladder_width'] // 10, sm.DEFAULTS['ladder_step'],
                        sm.DEFAULTS['ladder_min']))
        train_lines = sm.describe_train(gen, 'pulses drawn for the plotted cycle')
    else:
        counts = build_model_signal()
        volts = counts_to_volts(counts, args.range)
        events, det = run_detector(counts, DT_US)
        title = 'model_check.py synthetic signal'
        note = 'The detector cross-check signal (baseline + noise + broad peaks + glitches).'

    nvalid = sum(1 for e in events if e['m']['valid'])

    charts = [('Overview - full record (%.1f s)' % (len(counts) / SAMPLE_RATE_HZ),
               render_chart(volts, events, args.range, 0, len(counts) - 1,
                            'Overview: %s' % title,
                            'AD7606 RANGE = +/-%g V   |   %d samples @ %d Hz   |   '
                            '%d events, %d valid'
                            % (args.range, len(counts), SAMPLE_RATE_HZ, len(events), nvalid)))]
    for n, ev in enumerate(events):
        pad = max(60, 3 * (ev['end'] - ev['start']))
        i0 = max(0, ev['start'] - pad)
        i1 = min(len(counts) - 1, ev['end'] + pad)
        m = ev['m']
        charts.append(('Event #%d  @ sample %d  ->  %s' % (n, ev['start'], REASON_NAME[m['reason']]),
                       render_chart(volts, events, args.range, i0, i1,
                                    'Event #%d  @ %.1f ms' % (n, ev['start'] * 1000.0 / SAMPLE_RATE_HZ),
                                    '%s   (window %d smp = %d us; judged duration %d smp '
                                    '= %d us; max=%d)'
                                    % (REASON_NAME[m['reason']], m['width_samples'],
                                       m['width_us'], m['duration_samples'],
                                       m['duration_us'], m['max_value']),
                                    width=560, height=300, labels=True)))

    meta = ('Source: <b>%s</b>. %s<br>AD7606 RANGE strap = <b>+/-%g V</b> '
            '(%d counts full scale). Sample rate <b>%d Hz</b>; baseline tracked to '
            '<b>%.0f counts</b>, sigma <b>%.2f counts</b>.'
            % (title, note, args.range, AD7606_FULL_SCALE_COUNTS, SAMPLE_RATE_HZ,
               det.mean, det.sigma))

    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(build_html(charts, meta))

    for line in train_lines:
        print(line)
    if train_lines:
        print('')

    print('source : %s' % title)
    print('range  : +/-%g V  (%d counts full scale)' % (args.range, AD7606_FULL_SCALE_COUNTS))
    print('samples: %d @ %d Hz (%.1f s)' % (len(counts), SAMPLE_RATE_HZ,
                                            len(counts) / SAMPLE_RATE_HZ))
    print('baseline after run: %.1f counts   sigma: %.2f counts' % (det.mean, det.sigma))
    print('closed events: %d  (%d valid)' % (len(events), nvalid))
    for n, ev in enumerate(events):
        m = ev['m']
        print('  #%d  %-6s  max=%5d  win=%5d smp  dur=%5d smp  start=%6d  %s'
              % (n, 'VALID' if m['valid'] else 'REJECT', m['max_value'],
                 m['width_samples'], m['duration_samples'], ev['start'],
                 REASON_NAME[m['reason']]))
    abspath = os.path.abspath(args.out)
    print('')
    print('chart written : %s' % abspath)
    print('the graph is HTML/SVG (not drawn in the terminal) - open the file above.')
    if not args.no_open:
        try:
            webbrowser.open('file:///' + abspath.replace('\\', '/'))
            print('launched the default browser.')
        except Exception as exc:                       # pragma: no cover - env dependent
            print('could not launch a browser automatically (%s); open the file above.' % exc)


if __name__ == '__main__':
    main()

