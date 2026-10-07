#!/usr/bin/env python3
"""
Graph the voltage waveform that the AD7606 modules will digitize - before the
hardware is on the bench.

Two selectable sources:

  dac    (default) the exact waveform the M4 drives out of its two 12-bit DACs
         in the bench loopback (SORTER_SIGNAL_GEN = 1).  This is a mirror of
         signal_gen.cpp: the same kPattern pulses, the same 256-entry
         raised-cosine LUT and the same xorshift dither.  In the loopback those
         DAC pins are wired straight into the AD7606 inputs, so this is
         literally the voltage the modules will sample.  Two pattern cycles are
         run and the second is plotted, so the detector is past its 3000-sample
         warmup (on the real board P1 @ sample 2000 is only caught once armed).

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

# --- constants mirrored from config.h / signal_gen.cpp -----------------------
SAMPLE_RATE_HZ = 10000             # SORTER_SAMPLE_RATE_HZ
DT_US          = 1000000 // SAMPLE_RATE_HZ

# signal_gen.cpp (SIGNALGEN_* in config.h)
CYCLE_SAMPLES  = 20000             # SIGNALGEN_CYCLE_SAMPLES  (2.0 s @ 10 kSPS)
MODEL_SAMPLES  = 26000             # model_check.py main() length (covers the peak @ 20000)
BASELINE_CODE  = 2048              # SIGNALGEN_BASELINE_CODE  (~1.65 V)
NOISE_CODE     = 6                 # SIGNALGEN_NOISE_CODE     (~4.8 mV)
NOISE_ALPHA    = 1.0               # SIGNALGEN_NOISE_ALPHA    (1.0 = white)
SEED           = 0x2F6E2B1         # SIGNALGEN_SEED
MIN_CODE       = 250               # SIGNALGEN_MIN_CODE
MAX_CODE       = 3850              # SIGNALGEN_MAX_CODE
LUT_SIZE       = 256
ADC_PER_DAC    = 2.64              # SIGNALGEN_ADC_PER_DAC (code scaling at +/-10 V)

# The GIGA's 12-bit DAC: 0..4095 over the ~3.3 V VDDA rail.
DAC_MAX_CODE   = 4095
DAC_VREF       = 3.3

# AD7606 is 16-bit signed: +/-32768 counts across the selected full scale.
AD7606_FULL_SCALE_COUNTS = 32768

# (start, width, amplitude_in_AD7606_counts, 'cos'|'rect') == signal_gen.cpp kPattern
PATTERN = [
    ( 2000, 1100,  800, 'cos'),    # P1 broad peak      -> VALID
    ( 6000,  900, 1200, 'cos'),    # P2 broad peak      -> VALID
    (11000, 1400,  500, 'cos'),    # P3 broad peak      -> VALID
    (15000,    4,  900, 'rect'),   # G1 spike           -> SHORT_FOR_HEIGHT
    (17000,    2, 4000, 'rect'),   # G2 spike           -> SHORT_FOR_HEIGHT
                                   #   (signal_gen.cpp's comment says WIDTH_TOO_NARROW;
                                   #   the mirror measures width=3, which is not below
                                   #   the min, so the slope test fires first instead)
    (18500,   20,   60, 'rect'),   # G3 small bump      -> MAX_TOO_SMALL
]

REASON_NAME = {
    mc.REASON_OK:               'VALID',
    mc.REASON_MAX_TOO_SMALL:    'rejected: max too small',
    mc.REASON_WIDTH_TOO_NARROW: 'rejected: width too narrow',
    mc.REASON_WIDTH_TOO_WIDE:   'rejected: width too wide',
    mc.REASON_AREA_TOO_SMALL:   'rejected: area too small',
    mc.REASON_SHORT_FOR_HEIGHT: 'rejected: too short for its height',
}
REASON_SHORT = {
    mc.REASON_OK:               'VALID',
    mc.REASON_MAX_TOO_SMALL:    'max too small',
    mc.REASON_WIDTH_TOO_NARROW: 'too narrow',
    mc.REASON_WIDTH_TOO_WIDE:   'too wide',
    mc.REASON_AREA_TOO_SMALL:   'area too small',
    mc.REASON_SHORT_FOR_HEIGHT: 'short for height',
}

def _raise_cosine_lut():
    return [0.5 * (1.0 - math.cos(2.0 * math.pi * i / LUT_SIZE))
            for i in range(LUT_SIZE)]


def build_dac_codes(total=CYCLE_SAMPLES):
    """Mirror of signal_gen.cpp: DAC codes (0..4095) for `total` samples.

    The bench pattern repeats every CYCLE_SAMPLES (SignalGen::pos_ wraps) while
    the dither RNG keeps running, so a multi-cycle stream is continuous.
    """
    shape = _raise_cosine_lut()
    pulse_dac = [amp / ADC_PER_DAC for (_, _, amp, _) in PATTERN]
    codes = []
    rng = SEED
    dither = 0.0
    for i in range(total):
        pos = i % CYCLE_SAMPLES
        # --- SignalGen::tick(): xorshift32 dither, white at alpha = 1.0 ------
        rng = (rng ^ ((rng << 13) & 0xFFFFFFFF)) & 0xFFFFFFFF
        rng = (rng ^ (rng >> 17)) & 0xFFFFFFFF
        rng = (rng ^ ((rng << 5) & 0xFFFFFFFF)) & 0xFFFFFFFF
        signed = rng if rng < 0x80000000 else rng - 0x100000000
        white = signed / 2147483648.0
        dither += NOISE_ALPHA * (white - dither)

        # --- SignalGen::writeSample(pos) -------------------------------------
        v = BASELINE_CODE + dither * NOISE_CODE
        for idx, (start, width, _amp, kind) in enumerate(PATTERN):
            if pos < start or pos >= start + width:
                continue
            if kind == 'rect':
                v += pulse_dac[idx]
            else:
                u = ((pos - start) * LUT_SIZE) // width
                if u >= LUT_SIZE:
                    u = LUT_SIZE - 1
                v += pulse_dac[idx] * shape[u]
        v = min(MAX_CODE, max(MIN_CODE, v))
        codes.append(int(math.floor(v + 0.5)))       # C uses lroundf
    return codes


def codes_to_volts(codes):
    """DAC output voltage == the AD7606 input voltage in the loopback."""
    return [c * DAC_VREF / DAC_MAX_CODE for c in codes]


def volts_to_counts(volts, range_v):
    """What the AD7606 actually outputs, given its RANGE strap full scale."""
    return [int(math.floor(v * (AD7606_FULL_SCALE_COUNTS / range_v) + 0.5))
            for v in volts]


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
            events.append({'start': i - m['width_samples'], 'end': i, 'm': m})
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

    # event windows, each with its baseline (dashed grey) and k_on threshold (dotted)
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
        if labels:
            mid = (X(s) + X(e)) / 2.0
            p.append('<text x="%.1f" y="%d" font-size="11" font-weight="600" fill="%s" '
                     'text-anchor="middle">%s</text>'
                     % (mid, mt - 4, col, 'VALID' if ok else _esc(REASON_SHORT[m['reason']])))
            p.append('<text x="%.1f" y="%d" font-size="10" fill="#666" '
                     'text-anchor="middle">max=%d  w=%d smp</text>'
                     % (mid, mt + 12, m['max_value'], m['width_samples']))

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
               '<span><b style="color:#c62828">&#183;&#183;</b> k_on threshold</span>'
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
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  'signals.html'),
                    help='output HTML path (default: test/signals.html)')
    ap.add_argument('--no-open', action='store_true',
                    help="don't launch the generated file in the default browser")
    args = ap.parse_args()

    if args.source == 'dac':
        # Two pattern cycles; analyse the second so the detector is past warmup.
        codes = build_dac_codes(2 * CYCLE_SAMPLES)
        volts_all = codes_to_volts(codes)
        counts_all = volts_to_counts(volts_all, args.range)
        events_all, det = run_detector(counts_all, DT_US)
        off = CYCLE_SAMPLES
        volts = volts_all[off:off + CYCLE_SAMPLES]
        counts = counts_all[off:off + CYCLE_SAMPLES]
        events = [{'start': e['start'] - off, 'end': e['end'] - off, 'm': e['m']}
                  for e in events_all if e['start'] >= off]
        title = 'DAC loopback (signal_gen.cpp kPattern)'
        note = ('Voltage is the M4 DAC output, which the AD7606 input is wired to in the '
                'bench loopback (SORTER_SIGNAL_GEN = 1). The second pattern cycle is shown, '
                'so the detector is past its 3000-sample warmup.')
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
                                    '%s   (max=%d, width=%d samples, %d us)'
                                    % (REASON_NAME[m['reason']], m['max_value'],
                                       m['width_samples'], m['width_us']),
                                    width=560, height=300, labels=True)))

    meta = ('Source: <b>%s</b>. %s<br>AD7606 RANGE strap = <b>+/-%g V</b> '
            '(%d counts full scale). Sample rate <b>%d Hz</b>; baseline tracked to '
            '<b>%.0f counts</b>, sigma <b>%.2f counts</b>.'
            % (title, note, args.range, AD7606_FULL_SCALE_COUNTS, SAMPLE_RATE_HZ,
               det.mean, det.sigma))

    with open(args.out, 'w', encoding='utf-8') as f:
        f.write(build_html(charts, meta))

    print('source : %s' % title)
    print('range  : +/-%g V  (%d counts full scale)' % (args.range, AD7606_FULL_SCALE_COUNTS))
    print('samples: %d @ %d Hz (%.1f s)' % (len(counts), SAMPLE_RATE_HZ,
                                            len(counts) / SAMPLE_RATE_HZ))
    print('baseline after run: %.1f counts   sigma: %.2f counts' % (det.mean, det.sigma))
    print('closed events: %d  (%d valid)' % (len(events), nvalid))
    for n, ev in enumerate(events):
        m = ev['m']
        print('  #%d  %-6s  max=%5d  width=%5d smp  start=%6d  %s'
              % (n, 'VALID' if m['valid'] else 'REJECT', m['max_value'],
                 m['width_samples'], ev['start'], REASON_NAME[m['reason']]))
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

