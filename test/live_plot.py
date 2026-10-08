#!/usr/bin/env python3
"""
Live datastream viewer for the AD7606 sample stream - no scope, no third-party
packages.

The M4 fills sample frames continuously (stream_sink.h); the M7 prints them as one
line per point once streaming has been switched on, and stays quiet otherwise, so
the ordinary event log stays readable:

    STREAM on: decim=10 chans=0,1 wf=1 points=64
    S,<sample index>,<gen code>,<ch>:<count>,<ch>:<count>,...

This script is that host.  It sends "T1" - the serial monitor's stream-on command,
so the M7 always re-prints its STREAM header - parses the lines and pushes them to
a small web page over Server-Sent Events, which draws a rolling window of the
traces in the browser.  Pure Python stdlib plus a browser: no pyserial, no npm, no
build step.

    py -3 test/live_plot.py --sim              # no board: signal_model.py generates
    py -3 test/live_plot.py --port COM7        # the real board
    py -3 test/live_plot.py --sim --window 2000 --http-port 9000

--sim ticks the Python mirror of the bench waveform (test/signal_model.py) at the
same 10 kSPS the firmware uses, feeds the detector mirror from model_check.py and
emits the very same `S,` lines the board would, so the viewer can be exercised
with nothing plugged in.  Its "measured" columns are that waveform as the AD7606
would digitize it (DAC volts -> counts for the selected RANGE strap, plus a little
ADC noise), and its event log comes from the real detector mirror.  Sampling is
wall-clock paced at the firmware's 10 kSPS (`--sim-speed N` asks for N times real
time and `0` for no pacing at all, handy for filling a small window quickly).

The page shows

  * the generated ("expected") trace and the measured channels in AD7606 counts,
    over the last --window points (default 4000 = 4 s at the default decimation),
  * the point rate and the newest sample index (sample index / 10 kSPS = seconds),
  * a text pane with everything the board prints that is not a sample line: the
    STREAM metadata line, the boot banner and every closed event with its verdict,
    so a run can be read back afterwards.

Ctrl+C stops it.  Because the M7 would otherwise keep flooding the serial monitor
with 1000 lines/s, the script sends "T0" on the way out (--keep-streaming skips
that); pressing "T" in a monitor afterwards switches it back on.

See also test/plot_signals.py: the same signal, offline, with the detector's
verdicts shaded per event.
"""

import argparse
import collections
import json
import math
import os
import queue
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import model_check as mc           # noqa: E402  (detector mirror, for --sim)
import signal_model as sm          # noqa: E402  (waveform mirror of signal_model.h)

# --- constants mirrored from config.h ---------------------------------------
SAMPLE_RATE_HZ = 10000             # SORTER_SAMPLE_RATE_HZ
DT_US          = 1000000 // SAMPLE_RATE_HZ
DECIM          = 10                # SORTER_STREAM_DECIM
POINTS         = 64                # SORTER_STREAM_POINTS (points per frame)
CHANNELS       = 2                 # SORTER_STREAM_CHANNELS

# The GIGA's 12-bit DAC and the AD7606's 16-bit signed output.
DAC_MAX_CODE = 4095
DAC_VREF     = 3.3
AD7606_FULL_SCALE_COUNTS = 32768

# Where the simulated measured channels get their noise from: in the +/-10 V strap
# one AD7606 count is ~0.3 mV, so a couple of counts keeps the detector mirror
# honest without hiding the signal.
SIM_ADC_NOISE = 2.0
SIM_SEED      = 0x5EED1234

# --sim paces itself in wall-clock time, but a Windows timer tick is ~1 ms, so
# sleeping once per sample would crawl; sleep only when this far ahead instead.
PACE_MIN_SLEEP = 0.004

TRACE_COLORS = ['#c62828', '#2e7d32', '#f9a825', '#6a1b9a',
                '#00838f', '#4e342e', '#ad1457', '#37474f']

# PeakRejectReason names, spelled the way the M7 prints them (reasonName()).
REASON_NAME = {
    mc.REASON_OK:               'OK',
    mc.REASON_MAX_TOO_SMALL:    'MAX_TOO_SMALL',
    mc.REASON_WIDTH_TOO_NARROW: 'WIDTH_TOO_NARROW',
    mc.REASON_WIDTH_TOO_WIDE:   'WIDTH_TOO_WIDE',
    mc.REASON_AREA_TOO_SMALL:   'AREA_TOO_SMALL',
    mc.REASON_SHORT_FOR_HEIGHT: 'SHORT_FOR_HEIGHT',
    mc.REASON_RAILED:           'RAILED',
}


def volts_to_counts(volts, range_v):
    """What the AD7606 reports for an input voltage (RANGE strap full scale)."""
    return int(math.floor(volts * (AD7606_FULL_SCALE_COUNTS / range_v) + 0.5))


def gen_scale_counts(range_v):
    """AD7606 counts per DAC code.  The gen column is a DAC code while the channel
    columns are counts, so the two only overlay once the code is scaled."""
    return (DAC_VREF / DAC_MAX_CODE) * (AD7606_FULL_SCALE_COUNTS / range_v)


class StreamState:
    """The latest samples plus the connected browsers (SSE subscribers).

    One producer thread (the serial reader or the simulator) calls add_batch();
    every browser request calls subscribe() and then drains its own queue, which
    keeps a slow client from stalling the producer.  A queue that overflows is
    dropped: the page then simply misses that slice and keeps going.
    """

    def __init__(self, window):
        self.window = window
        self.lock   = threading.Lock()
        self.subs   = []
        self.meta   = None            # dict sent to the page as soon as it connects
        self.recent = collections.deque(maxlen=window)   # (index, gen, [adc, ...])
        self.points = 0               # points seen since the start
        self.frames = 0
        self.t0     = time.time()

    # -- producer side -------------------------------------------------------
    def set_meta(self, **kw):
        with self.lock:
            self.meta = kw
        self.publish(dict(kw, kind='meta'))

    def add_batch(self, first_index, gen, adc):
        """One frame: gen = DAC codes, adc = [[count, ...], ...] per channel."""
        n = len(gen)
        with self.lock:
            self.points += n
            self.frames += 1
            for k in range(n):
                self.recent.append((first_index + k * DECIM, gen[k],
                                    [ch[k] for ch in adc]))
        self.publish({'kind': 'data', 'i': first_index, 'n': n,
                      'gen': gen, 'adc': adc})

    def log(self, line):
        if line:
            self.publish({'kind': 'log', 'line': line})

    # -- browser side --------------------------------------------------------
    def subscribe(self):
        """A fresh queue, primed with the metadata and the samples we still have."""
        q = queue.Queue(maxsize=128)
        with self.lock:
            self.subs.append(q)
            meta = dict(self.meta) if self.meta else None
            snapshot = list(self.recent)
        if meta:
            q.put_nowait(dict(meta, kind='meta'))
        if snapshot:
            q.put_nowait({'kind': 'data', 'i': snapshot[0][0], 'n': len(snapshot),
                          'gen': [s[1] for s in snapshot],
                          'adc': [[s[2][c] for s in snapshot]
                                  for c in range(len(snapshot[0][2]))]})
        return q

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subs:
                self.subs.remove(q)

    def publish(self, obj):
        with self.lock:
            subs = list(self.subs)
        for q in subs:
            try:
                q.put_nowait(obj)
            except queue.Full:          # a stalled browser: drop it, not the run
                self.unsubscribe(q)

    def rate(self):
        """Points/s averaged over the whole run."""
        dt = max(time.time() - self.t0, 1e-6)
        return self.points / dt


# ===========================================================================
# Line protocol - the M7's serial text
# ===========================================================================
class StreamParser:
    """Turns the M7's serial bytes into batches for StreamState.

    Only two line shapes matter to the plot:

        STREAM on: decim=10 chans=0,1 wf=1 points=64      metadata (sent by `T`)
        S,<sample index>,<gen code>,<ch>:<count>,...      one sample point

    Points are collected until a whole frame is in - the M7 prints a frame's points
    back to back, so either a contiguous sample index or SORTER_STREAM_POINTS
    points means "frame done" - and everything else is passed through as log text:
    the boot banner, the STREAM line and every closed event with its verdict.
    """

    def __init__(self, state, source, range_v, echo=True):
        self.state    = state
        self.source   = source
        self.range_v  = range_v
        self.echo     = echo
        self.buf      = ''
        self.gen      = []
        self.cols     = []
        self.frame_i  = None
        self.last_i   = None
        self.decim    = DECIM
        self.channels = []
        self.wf       = 0

    # ---- bytes -> lines ----------------------------------------------------
    def feed_bytes(self, data):
        parts = (self.buf + data.decode('ascii', 'replace')).split('\n')
        self.buf = parts.pop()               # keep the (possibly torn) tail
        for line in parts:
            self.feed_line(line.strip())

    def feed_line(self, line):
        if not line:
            return
        if line.startswith('S,'):
            self._point(line)
        elif line.startswith('STREAM '):
            self._info(line)
        else:
            self.log(line)

    def log(self, line):
        """Non-sample text: printed here *and* pushed to the page's log pane."""
        if self.echo:
            print(line, flush=True)
        self.state.log(line)

    # ---- the two interesting lines -----------------------------------------
    def _info(self, line):
        stream_on = line.split(':', 1)[0].strip().endswith('on')
        fields = {}
        for tok in line.replace(':', ' ').split():
            key, _, value = tok.partition('=')
            if value:
                fields[key] = value

        self.decim    = int(fields.get('decim', self.decim))
        self.wf       = int(fields.get('wf', 0))
        self.channels = [int(c) for c in fields.get('chans', '').split(',')
                         if c.strip().isdigit()]
        self.state.set_meta(source=self.source, streaming=stream_on,
                            decim=self.decim, channels=self.channels, wf=self.wf,
                            points=int(fields.get('points', POINTS)),
                            sample_rate=SAMPLE_RATE_HZ, range=self.range_v,
                            gen_scale=gen_scale_counts(self.range_v),
                            window=self.state.window)
        self.log(line)

    def _point(self, line):
        f = line.split(',')
        try:
            index = int(f[1])
            gen   = int(f[2])
            adc   = [int(tok.partition(':')[2]) for tok in f[3:]]
        except (IndexError, ValueError):
            return                           # a torn or malformed line: drop it
        if not adc:
            return

        if self.frame_i is None:
            self.frame_i = index
        elif index != self.last_i + self.decim or len(self.gen) >= POINTS:
            self.flush()                     # the previous frame ends here
            self.frame_i = index
        self.last_i = index

        self.gen.append(gen)
        while len(self.cols) < len(adc):
            self.cols.append([])
        for c, value in enumerate(adc):
            self.cols[c].append(value)

    def flush(self):
        if self.gen:
            self.state.add_batch(self.frame_i, self.gen, self.cols)
        self.gen, self.cols, self.frame_i, self.last_i = [], [], None, None


# ===========================================================================
# Sources - what feeds StreamState
# ===========================================================================
class SerialSource:
    """The board: hold COMx open, ask for the stream, read the lines it prints."""

    def __init__(self, port, state, range_v, start_stream=True):
        self.port         = port
        self.state        = state
        self.parser       = StreamParser(state, port, range_v)
        self.start_stream = start_stream
        self.fh           = None
        self.thread       = None
        self.stopped      = threading.Event()

    def name(self):
        """Windows wants \\\\.\\COMx once the port number is above COM9."""
        return self.port if self.port.startswith('\\\\') else '\\\\.\\' + self.port

    def start(self):
        # Raw COMx open, no pyserial: unbuffered, so read() hands over whatever the
        # driver has rather than waiting for a full buffer, and a read that is still
        # waiting when stop() closes the handle raises - the reader thread takes that
        # as "port gone" and exits.
        self.fh = open(self.name(), 'r+b', buffering=0)
        self.thread = threading.Thread(target=self._run, name='serial', daemon=True)
        self.thread.start()
        if self.start_stream:
            # "T1": idempotent, and the M7 re-prints its STREAM header every time,
            # which is what the page needs for its metadata.  ("T" alone would
            # toggle the stream *off* if it happened to be on already.)
            self.command('T1')
            print('live_plot: sent "T1" - the M7 switched the sample stream on.',
                  flush=True)

    def command(self, text):
        fh = self.fh
        if fh is None:
            return
        try:
            fh.write((text + '\n').encode('ascii'))
        except OSError as exc:
            print('could not send %r: %s' % (text, exc), flush=True)

    def _run(self):
        fh = self.fh
        while not self.stopped.is_set() and fh is not None:
            try:
                data = fh.read(4096)
            except (OSError, ValueError):   # the port went away / was closed
                break
            if data:
                self.parser.feed_bytes(data)
            else:
                time.sleep(0.002)           # an idle read returns at once
        self.parser.flush()

    def stop(self):
        self.stopped.set()
        fh, self.fh = self.fh, None
        if fh is not None:
            try:
                fh.close()                  # unblocks the reader thread
            except OSError:
                pass
        if self.thread is not None:
            self.thread.join(timeout=1.5)


class SimSource:
    """No board: signal_model.py generates the waveform and prints the same lines.

    The samples go through the very same text protocol as the board's (they are
    built and re-parsed, so the parser is exercised too), but paced by this host
    rather than by the M4's own timer.  The measured columns are the generated
    waveform as the AD7606 would digitize it, and the detector mirror runs at the
    full sample rate on channel 0, so the log pane fills with the event lines the M7
    would print.
    """

    def __init__(self, state, range_v, speed=1.0, channels=CHANNELS, seed=None):
        overrides = {'seed': seed} if seed is not None else {}
        self.state   = state
        self.range_v = range_v
        self.speed   = speed
        self.nch     = channels
        self.parser  = StreamParser(state, 'sim', range_v)
        self.model   = sm.SignalModel(**overrides)
        self.det     = mc.EventDetector(mc.Config())
        self.dither  = sm.SignalModel(seed=SIM_SEED)   # only for its Gaussian noise
        self.thread  = None
        self.stopped = threading.Event()

    def start(self):
        self.parser.log('sim: waveform mirror signal_model.py, seed 0x%X, '
                        '%d-sample cycle (%.1f s)'
                        % (self.model.seed, self.model.cycle_samples,
                           self.model.cycle_samples / float(SAMPLE_RATE_HZ)))
        self.parser.log('sim: DAC -> AD7606 loopback at +/-%g V, %d measured channel(s)'
                        % (self.range_v, self.nch))
        self.parser.feed_line('STREAM on: decim=%d chans=%s wf=1 points=%d'
                              % (DECIM, ','.join(str(c) for c in range(self.nch)),
                                 POINTS))
        self.thread = threading.Thread(target=self._run, name='sim', daemon=True)
        self.thread.start()

    def _counts(self, code):
        """What each AD7606 reports for this DAC code, plus a little ADC noise.

        Box-Muller gives two independent normals per call, so one call covers the
        usual two channels.
        """
        base = volts_to_counts(code * DAC_VREF / DAC_MAX_CODE, self.range_v)
        noise = []
        while len(noise) < self.nch:
            noise.extend(self.dither._gauss_pair())
        return [base + int(round(noise[c] * SIM_ADC_NOISE))
                for c in range(self.nch)]

    def _run(self):
        paced  = self.speed > 0.0               # --sim-speed 0 = run flat out
        dt     = 1.0 / (SAMPLE_RATE_HZ * self.speed) if paced else 0.0
        next_t = time.time()
        frame  = []                      # (index, code, counts) of the open frame
        sample = 0

        while not self.stopped.is_set():
            code   = self.model.tick()
            counts = self._counts(code)

            # The firmware runs the detector on every sample of every active
            # channel; channel 0 is the primary one it reports on.
            m = self.det.update(counts[0], sample * DT_US, sample)
            if m is not None:
                self._event_text(m)

            if (sample % DECIM) == 0:
                frame.append((sample, code, counts))
                if len(frame) >= POINTS:
                    self._emit(frame)
                    frame = []

            sample += 1
            if paced:
                # Sleep only once every few milliseconds: a Windows timer tick is
                # ~1 ms, so one sleep per sample would pace this to a third of the
                # speed.  Behind by a lot (a slow host, or a step in the debugger)?
                # Re-base instead of accumulating a debt we can never pay off.
                next_t += dt
                delay = next_t - time.time()
                if delay > PACE_MIN_SLEEP:
                    time.sleep(delay)
                elif delay < -0.05:
                    next_t = time.time()

    def _emit(self, frame):
        for (index, code, counts) in frame:
            self.parser.feed_line('S,%d,%d%s'
                                  % (index, code,
                                     ''.join(',%d:%d' % (c, count) for c, count
                                             in enumerate(counts))))

    def _event_text(self, m):
        """The M7's own wording, so the log pane reads like a monitor session."""
        self.parser.log('Event #%u  %sreason=%u %s  window=%dus/%dsmp  primary=ch0'
                        % (m['index'], 'VALID     ' if m['valid'] else 'rejected  ',
                           m['reason'], REASON_NAME[m['reason']], m['width_us'],
                           m['width_samples']))
        self.parser.log('  ch0  max=%d  area=%d  width=%dus/%dsmp  base=%.0f  %s'
                        % (m['max_value'], int(m['area']), m['width_us'],
                           m['width_samples'], m['baseline'],
                           'VALID' if m['valid'] else 'rejected'))

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=1.5)
        self.parser.flush()


# ===========================================================================
# The web page - Server-Sent Events, one canvas, no dependencies
# ===========================================================================
PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>AD7606 sample stream - sorter firmware</title>
<style>
  body  { font-family: 'Segoe UI', Arial, sans-serif; margin: 16px; color: #222; }
  h1    { font-size: 17px; margin: 0 0 2px; }
  .meta { color: #555; font-size: 12px; margin-bottom: 6px; }
  .bar  { font-size: 12px; margin: 6px 0; }
  .bar label { margin-right: 14px; }
  .lg span { margin-right: 14px; font-size: 12px; white-space: nowrap; }
  .sw   { display: inline-block; width: 12px; height: 12px; margin-right: 4px;
          border: 1px solid #999; vertical-align: -1px; }
  canvas { display: block; border: 1px solid #ddd; }
  #log  { font-family: Consolas, 'Courier New', monospace; font-size: 11px;
          line-height: 1.35; height: 190px; overflow: auto; margin-top: 8px;
          padding: 6px; border: 1px solid #ddd; background: #fafafa; }
  #log div { white-space: pre-wrap; }
</style>
</head>
<body>
<h1>AD7606 sample stream</h1>
<div class="meta" id="meta">waiting for the board...</div>
<div class="lg" id="legend"></div>
<div class="bar" id="stats">&nbsp;</div>
<canvas id="plot" width="1140" height="430"></canvas>
<div class="bar">
  <label><input type="checkbox" id="autoy" checked> auto Y</label>
  <label><input type="checkbox" id="showgen" checked> expected waveform</label>
  <label><input type="checkbox" id="pause"> pause</label>
  <label><input type="checkbox" id="clear"> clear</label>
</div>
<div id="log"></div>
<script>
'use strict';
const WINDOW  = __WINDOW__;      // points kept per trace
const COLORS  = __COLORS__;      // one colour per measured channel
const RANGE_V = __RANGE__;       // AD7606 input range (full scale = +/-RANGE_V V)

const el = (id) => document.getElementById(id);
const cv = el('plot'), g = cv.getContext('2d');

let meta = { channels: [], decim: 10, sample_rate: 10000, window: WINDOW,
             range: RANGE_V, wf: 0, streaming: false, source: '?' };
let gen = null, chans = [];
let showGen = true, autoY = true, paused = false, frozen = null;
let received = 0, t0 = 0, rate = 0, newest = null, dirty = true, lastDraw = 0;

function newSeries(label, dashed, color){
  return { label: label, first: null, vals: [], dashed: dashed, color: color };
}

function buildSeries(){
  chans = (meta.channels || []).map((c, k) =>
            newSeries('ch' + c, false, COLORS[k % COLORS.length]));
  gen   = newSeries('expected', true, '#1565c0');
  received = 0; t0 = performance.now(); newest = null; frozen = null;
  renderLegend();
}

function renderLegend(){
  const lg = el('legend'); lg.innerHTML = '';
  chans.forEach((s, k) => {
    const sp = document.createElement('span'), sw = document.createElement('span');
    sw.className = 'sw'; sw.style.background = s.color;
    sp.appendChild(sw);
    sp.appendChild(document.createTextNode(' ' + s.label + ' measured'));
    lg.appendChild(sp);
  });
  const sp = document.createElement('span'), sw = document.createElement('span');
  sw.className = 'sw'; sw.style.background = gen.color;
  sp.appendChild(sw);
  sp.appendChild(document.createTextNode(' expected (DAC code x scale)'));
  lg.appendChild(sp);
}

function metaText(){
  const sr = meta.sample_rate || 0, d = meta.decim || 1;
  const span = sr > 0 ? meta.window * d / sr : 0;
  return 'source ' + meta.source
       + '   ' + (meta.streaming ? 'stream on' : 'stream off')
       + '   ' + (sr / 1000).toFixed(1) + ' kSPS, decim ' + d
       + '   window ' + meta.window + ' pts (' + span.toFixed(2) + ' s)'
       + '   range +/-' + meta.range + ' V  (full scale +/-32768 counts)'
       + '   waveform gen ' + (meta.wf ? 'on' : 'off');
}

function applyMeta(m){
  const changed = (m.channels || []).length !== chans.length;
  meta = m;
  if (changed) buildSeries(); else renderLegend();
  el('meta').textContent = metaText();
  dirty = true;
}

/* SSE payloads: the whole batch of a frame, oldest point first. */
function applyData(m){
  if (paused) return;
  const d = meta.decim || 1, i0 = m.i;
  if (m.gen) {
    for (let k = 0; k < m.gen.length; k++) addPoint(gen, i0 + k * d, m.gen[k]);
  }
  (m.adc || []).forEach((vals, c) => {
    const s = chans[c];
    if (!s || !vals) return;
    for (let k = 0; k < vals.length; k++) addPoint(s, i0 + k * d, vals[k]);
  });
  const n = (m.adc && m.adc[0] ? m.adc[0].length : 0);
  received += n;
  if (n) newest = i0 + (n - 1) * d;
  trimAll();
  dirty = true;
}

function addPoint(s, i, v){
  if (s.first === null) s.first = i;
  s.vals.push(v);
}

function trimAll(){
  const now = performance.now();
  if (now - lastDraw < 200) return;          // cheap enough: trim once a frame
  trim(gen);
  chans.forEach(trim);
}

function trim(s){
  const extra = s.vals.length - WINDOW;
  if (extra > 0){
    s.vals.splice(0, extra);
    s.first += extra * (meta.decim || 1);    // indices stay absolute
  }
}

function addLog(line){
  const lg = el('log'), d = document.createElement('div');
  d.textContent = line;
  lg.appendChild(d);
  while (lg.childElementCount > 400) lg.removeChild(lg.firstChild);
  lg.scrollTop = lg.scrollHeight;
}

/* ---- drawing ---------------------------------------------------------- */
function visible(){
  const d = meta.decim || 1;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  const all = (showGen ? [gen] : []).concat(chans);
  all.forEach((s) => {
    if (!s || s.first === null || !s.vals.length) return;
    const last = s.first + (s.vals.length - 1) * d;
    if (s.first < x0) x0 = s.first;
    if (last > x1) x1 = last;
    for (let k = 0; k < s.vals.length; k++){
      const v = s.vals[k];
      if (v < y0) y0 = v;
      if (v > y1) y1 = v;
    }
  });
  if (x0 === Infinity || y0 === Infinity) return null;
  if (autoY || frozen === null){
    const pad = Math.max((y1 - y0) * 0.08, 8);
    y0 -= pad; y1 += pad;
    frozen = { y0: y0, y1: y1 };
  } else {
    y0 = frozen.y0; y1 = frozen.y1;
  }
  if (x1 <= x0) x1 = x0 + 1;
  if (y1 <= y0) y1 = y0 + 1;
  return { x0: x0, x1: x1, y0: y0, y1: y1 };
}

function draw(){
  const w = cv.width, h = cv.height;
  const ml = 78, mr = 16, mt = 26, mb = 36;
  const pw = w - ml - mr, ph = h - mt - mb;
  g.clearRect(0, 0, w, h);
  g.fillStyle = '#fff'; g.fillRect(0, 0, w, h);

  const view = visible();
  g.font = '11px Segoe UI';
  g.fillStyle = '#555'; g.textAlign = 'left'; g.textBaseline = 'top';
  g.fillText('counts (full scale +/-32768 = +/-' + meta.range + ' V)', ml, 6);
  if (!view) return;

  const X = (i) => ml + (i - view.x0) * pw / (view.x1 - view.x0);
  const Y = (v) => mt + ph - (v - view.y0) * ph / (view.y1 - view.y0);

  g.strokeStyle = '#ececec'; g.lineWidth = 1;
  g.fillStyle = '#666'; g.textAlign = 'right'; g.textBaseline = 'middle';
  for (let k = 0; k <= 5; k++){
    const v = view.y0 + (view.y1 - view.y0) * k / 5;
    const y = Math.round(Y(v)) + 0.5;
    g.beginPath(); g.moveTo(ml, y); g.lineTo(ml + pw, y); g.stroke();
    g.fillText(v.toFixed(0), ml - 6, y);
  }

  /* One vertical line per second of captured time. */
  const sr = meta.sample_rate || 1, d = meta.decim || 1;
  const pps = Math.max(1, sr / d);
  const spanS = (view.x1 - view.x0) / pps;
  const step = pps * (spanS < 1.6 ? 0.25 : spanS < 3.2 ? 0.5 : 1);
  g.textAlign = 'center'; g.textBaseline = 'top';
  for (let i = Math.ceil(view.x0 / step) * step; i <= view.x1; i += step){
    const x = Math.round(X(i)) + 0.5;
    g.beginPath(); g.moveTo(x, mt); g.lineTo(x, mt + ph); g.stroke();
    const t = i / sr;
    g.fillText(t < 10 ? t.toFixed(2) + ' s' : t.toFixed(1) + ' s', x, mt + ph + 6);
  }

  const order = (showGen ? [gen] : []).concat(chans);
  order.forEach((s) => {
    if (!s || s.first === null || s.vals.length < 2) return;
    g.strokeStyle = s.color;
    g.lineWidth = s.dashed ? 1.5 : 1.2;
    g.setLineDash(s.dashed ? [6, 4] : []);
    g.beginPath();
    for (let k = 0; k < s.vals.length; k++){
      const x = X(s.first + k * d), y = Y(s.vals[k]);
      if (k) g.lineTo(x, y); else g.moveTo(x, y);
    }
    g.stroke();
    g.setLineDash([]);
  });

  g.strokeStyle = '#bbb';
  g.strokeRect(ml + 0.5, mt + 0.5, pw, ph);
}

function statsText(){
  const sr = meta.sample_rate || 1, d = meta.decim || 1;
  const secs = (performance.now() - t0) / 1000;
  if (secs > 1.5) rate = received / secs;
  const t = newest === null ? 0 : newest / sr;
  return 'newest sample ' + (newest === null ? '-' : newest)
       + ' (t = ' + t.toFixed(2) + ' s)'
       + '   received ' + received + ' points'
       + '   ' + rate.toFixed(0) + ' points/s = '
       + (rate * d / 1000).toFixed(2) + ' kSPS'
       + '   trace holds ' + (gen ? gen.vals.length : 0) + ' points'
       + (paused ? '   [PAUSED]' : '');
}

/* ---- live wiring ------------------------------------------------------ */
const es = new EventSource('/stream');
es.onmessage = (ev) => {
  let m;
  try { m = JSON.parse(ev.data); } catch (e) { return; }
  if (m.kind === 'meta')      applyMeta(m);
  else if (m.kind === 'data') applyData(m);
  else if (m.kind === 'log')  addLog(m.line);
};
es.onopen  = () => { el('meta').textContent = metaText(); };
es.onerror = () => { el('meta').textContent = 'stream lost - reconnecting...'; };

el('autoy').onchange   = (e) => { autoY  = e.target.checked; dirty = true; };
el('showgen').onchange = (e) => { showGen = e.target.checked; dirty = true; };
el('pause').onchange   = (e) => { paused = e.target.checked; dirty = true; };
el('clear').onchange   = (e) => {
  if (e.target.checked){ buildSeries(); el('log').innerHTML = ''; }
  e.target.checked = false;
};

function frame(){
  const now = performance.now();
  if (now - lastDraw > 100 && (dirty || now - lastDraw > 500)){
    dirty = false; lastDraw = now;
    draw();
    el('stats').textContent = statsText();
  }
  requestAnimationFrame(frame);
}
buildSeries();
requestAnimationFrame(frame);
</script>
</body>
</html>
"""


def page_html(window, range_v):
    """The page, with its three constants filled in."""
    return (PAGE
            .replace('__WINDOW__', str(window))
            .replace('__COLORS__', json.dumps(TRACE_COLORS))
            .replace('__RANGE__', '%g' % range_v))


class Handler(BaseHTTPRequestHandler):
    """`GET /` serves the page; `GET /stream` is the SSE feed it subscribes to."""

    server_version  = 'live_plot'
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        pass                          # SSE reconnects would spam the console

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.send_page()
        elif self.path.split('?')[0] == '/stream':
            self.send_events()
        else:
            self.send_error(404, 'not found')

    def send_page(self):
        body = page_html(self.server.state.window, self.server.range_v).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def send_events(self):
        """One long event-stream response; the body ends when the tab closes."""
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Accel-Buffering', 'no')
        self.end_headers()
        self.wfile.write(b'retry: 1000\n\n')
        self.wfile.flush()

        state = self.server.state
        q = state.subscribe()         # primed with the meta line + the samples we have
        try:
            while not self.server.stopping.is_set():
                try:
                    obj = q.get(timeout=5.0)
                except queue.Empty:
                    self.wfile.write(b': ping\n\n')      # keeps the socket warm
                    self.wfile.flush()
                    continue
                payload = json.dumps(obj, separators=(',', ':')).encode('utf-8')
                self.wfile.write(b'data: ' + payload + b'\n\n')
                self.wfile.flush()
        except (OSError, ValueError):  # the browser tab went away
            pass
        finally:
            state.unsubscribe(q)


# ===========================================================================
# Command line
# ===========================================================================
def main(argv=None):
    p = argparse.ArgumentParser(
        description='Live view of the AD7606 sample stream (no third-party tools).',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='examples:\n'
               '  py -3 test/live_plot.py --sim\n'
               '  py -3 test/live_plot.py --sim --window 2000 --http-port 9000\n'
               '  py -3 test/live_plot.py --port COM7')

    src = p.add_argument_group('source')
    src.add_argument('--port', help='serial port of the M7 UART, e.g. COM7')
    src.add_argument('--sim', action='store_true',
                     help='no board: signal_model.py generates the waveform')
    src.add_argument('--sim-speed', type=float, default=1.0,
                     help='simulated time scale: 2 = twice as fast, '
                          '0 = as fast as Python can (default 1)')
    src.add_argument('--sim-seed', type=lambda s: int(s, 0), default=None,
                     help='override the simulated generator seed')
    src.add_argument('--no-stream-command', action='store_true',
                     help='never send "T1"/"T0" (the board is already streaming)')

    view = p.add_argument_group('view')
    view.add_argument('--channels', type=int, default=CHANNELS,
                      help='simulated and plotted channel count (default %(default)s)')
    view.add_argument('--range', dest='range_v', type=float, default=10.0,
                      help='AD7606 full-scale input in volts (default %(default)s)')
    view.add_argument('--window', type=int, default=4000,
                      help='points kept in the rolling window (default %(default)s)')
    view.add_argument('--host', default='127.0.0.1', help='bind address')
    view.add_argument('--http-port', type=int, default=8000,
                      help='page/SSE port (default %(default)s)')
    view.add_argument('--no-open', action='store_true',
                      help='do not open a browser')
    view.add_argument('--keep-streaming', action='store_true',
                      help='leave the board streaming on exit (no "T0")')
    args = p.parse_args(argv)

    if not args.sim and not args.port:
        p.error('give --port COMx for the board, or --sim to run without one')

    state = StreamState(window=max(args.window, 2 * POINTS))
    if args.sim:
        source = SimSource(state, args.range_v, speed=args.sim_speed,
                           channels=max(args.channels, 1), seed=args.sim_seed)
    else:
        source = SerialSource(args.port, state, args.range_v,
                              start_stream=not args.no_stream_command)

    httpd = ThreadingHTTPServer((args.host, args.http_port), Handler)
    httpd.state         = state
    httpd.range_v       = args.range_v
    httpd.stopping      = threading.Event()
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, name='http', daemon=True).start()

    url = 'http://%s:%d/' % ('localhost' if args.host in ('0.0.0.0', '::') else args.host,
                             args.http_port)
    print('live_plot: source = %s' % ('simulator (signal_model.py)' if args.sim
                                      else args.port))
    print('live_plot: open %s   (Ctrl+C to stop)' % url)
    if not args.no_open:
        webbrowser.open(url)

    code = 0
    try:
        source.start()
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('\nlive_plot: stopping...')
    except OSError as exc:
        print('live_plot: could not start %s: %s'
              % ('the simulator' if args.sim else args.port, exc), file=sys.stderr)
        print('live_plot: is another monitor holding the port?  (--sim needs no '
              'board at all)', file=sys.stderr)
        code = 1
    finally:
        if isinstance(source, SerialSource) and source.fh is not None \
                and not args.keep_streaming and not args.no_stream_command:
            source.command('T0')        # stop the 1000 lines/s flood
            print('live_plot: sent "T0" - the sample stream is off again.',
                  flush=True)
        source.stop()
        httpd.stopping.set()
        httpd.shutdown()
        httpd.server_close()

    print('live_plot: %d points in %d frames, %.1f s, %.0f points/s'
          % (state.points, state.frames, time.time() - state.t0, state.rate()))
    return code


if __name__ == '__main__':
    sys.exit(main())

