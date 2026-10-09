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
    py -3 test/live_plot.py --sim --window 2 --http-port 9000

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
    over a rolling *time* window - `--window` seconds, one by default, adjustable
    with the page's slider (from 0.1 s up to the `--history` seconds kept per trace,
    8 by default and capped at 60 s).  Time, not a point count, is what keeps the pane
    readable: the same second of stream whatever the board's rate, so events cannot
    pile up in it, and widening the window redraws from samples already in hand,
  * the point rate and the newest sample index (sample index / 10 kSPS = seconds),
  * every closed event drawn over the traces as the detector's decision: the window
    it claimed (its width in samples), the baseline it froze for that window and the
    peak it reported on top of that baseline, coloured by the verdict - so a pulse
    buried in noise, or two pulses merged into one window, can be judged against the
    same numbers the log prints (`detector overlay` unticks it),
  * the two things those lines cannot show, read back off the plotted points and
    tagged in colours of their own: `2 peaks  dip +58` when one window swallowed two
    peaks, and `!max` / `?max` when the window's own samples do not reach the max it
    reported (an impulse the decimation dropped, most often).  Both describe the
    trace, never replace the measured numbers - see *Live sample stream* below.
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
import re
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
EVENT_KEEP     = 400               # detector decisions kept for the page's overlay

# The page's rolling window, in seconds.  A *time* window rather than a point count
# keeps the pane readable: the same second of stream, whatever the board's rate, so
# events cannot pile up in it.  --window is where the slider opens, --history is how
# much is kept per trace - that is what lets the window be widened without waiting
# for new samples (and what a newly opened tab is primed with).
VIEW_SEC       = 1.0
HISTORY_SEC    = 8.0
MIN_VIEW_SEC   = 0.1               # the slider's floor, in seconds
MAX_KEEP_SEC   = 60.0              # .. and the ceiling on --history: beyond it the
                                   # sample buffers here and in the browser cost more
                                   # than a window that wide could ever show
POINTS_PER_SEC = SAMPLE_RATE_HZ // DECIM       # 1000 at the defaults


def view_points(seconds):
    """Points per trace `seconds` of stream is worth at the default decimation."""
    return max(2 * POINTS, int(round(seconds * POINTS_PER_SEC)))

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

# printBundle() in sorter-firmware.ino: the two lines a closed event prints.  The
# head carries the window and the verdict, the channel line the numbers behind it.
#
#   Event #12  VALID     reason=0 OK  window=1234..5678us/24smp  primary=ch0  mode=linked
#     ch0  max=1042  area=12033  width=2400us/24smp  base=5412  VALID
EVENT_HEAD = re.compile(r'^Event #(\d+)\s+(VALID|rejected)\s+reason=(\d+)\s+(\S+)'
                        r'\s+window=(\d+)\.\.(\d+)us/(\d+)smp\s+primary=ch(\d+)'
                        r'\s+mode=(\S+)')
EVENT_CHANNEL = re.compile(r'^ch(\d+)\s+max=(-?\d+)\s+area=(\d+)\s+width=(\d+)us/'
                           r'(\d+)smp\s+base=(-?\d+)\s+(\S+)(?:\s+(\S+))?')


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

    `window` is how many points per trace are kept - enough for `keep_sec` seconds
    of stream, which is the widest window the page's slider offers; `window_sec` is
    what that slider reads when the page opens.  Both travel in the meta line, so a
    page served from the command line and the samples it is primed with agree.
    """

    def __init__(self, window, window_sec=VIEW_SEC, keep_sec=HISTORY_SEC):
        self.window = window
        self.window_sec = window_sec
        self.keep_sec   = keep_sec
        self.lock   = threading.Lock()
        self.subs   = []
        self.meta   = None            # dict sent to the page as soon as it connects
        self.recent = collections.deque(maxlen=window)   # (index, gen, [adc, ...])
        self.events = collections.deque(maxlen=EVENT_KEEP)  # detector decisions
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

    def add_event(self, ev):
        """One detector decision: the window, its width, the baseline and the peak."""
        with self.lock:
            self.events.append(ev)
        self.publish(dict(ev, kind='event'))

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
            events = list(self.events)
        if meta:
            q.put_nowait(dict(meta, kind='meta'))
        if snapshot:
            q.put_nowait({'kind': 'data', 'i': snapshot[0][0], 'n': len(snapshot),
                          'gen': [s[1] for s in snapshot],
                          'adc': [[s[2][c] for s in snapshot]
                                  for c in range(len(snapshot[0][2]))]})
        if events:
            q.put_nowait({'kind': 'events', 'list': events})
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

    Three line shapes matter to the page:

        STREAM on: decim=10 chans=0,1 wf=1 points=64      metadata (sent by `T`)
        S,<sample index>,<gen code>,<ch>:<count>,...      one sample point
        Event #12  VALID  reason=0 OK  window=...         a closed event, plus
          ch0  max=...  area=...  width=...  base=...       its per-channel line

    Points are collected until a whole frame is in - the M7 prints a frame's points
    back to back, so either a contiguous sample index or SORTER_STREAM_POINTS
    points means "frame done".  Event lines are parsed into the decision the page
    draws over the traces (window, width, baseline, peak, verdict); everything else
    is passed through as log text: the boot banner, the STREAM line and every event
    line, so the log pane still reads like a monitor session.
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
        self.pending  = None                  # Event # head waiting for its ch line
        self.sample_rate = SAMPLE_RATE_HZ      # events arrive in us; traces count samples

    # ---- bytes -> lines ----------------------------------------------------
    def feed_bytes(self, data):
        parts = (self.buf + data.decode('ascii', 'replace')).split('\n')
        self.buf = parts.pop()               # keep the (possibly torn) tail
        for line in parts:
            self.feed_line(line.strip())

    def feed_line(self, line):
        line = line.strip()                  # callers may hand us the raw text
        if not line:
            return
        if line.startswith('S,'):
            self.flush_event()               # a sample line ends an open event group
            self._point(line)
        elif line.startswith('STREAM '):
            self.flush_event()
            self._info(line)
        else:
            self._event(line)                # also keeps the line in the log pane
            self.log(line)

    def log(self, line):
        """Non-sample text: printed here *and* pushed to the page's log pane."""
        if self.echo:
            print(line, flush=True)
        self.state.log(line)

    def us_to_sample(self, us):
        """The sample index a microsecond timestamp falls on.

        The board stamps events with micros() while the stream counts samples, so
        this is the paced grid rather than an exact index: the loop re-syncs its
        time grid when it runs late, which drifts a few samples apart over a long
        run - far less than a pixel at the window the page shows.
        """
        return int(round(us * self.sample_rate / 1000000.0))

    # ---- the decision behind the log lines ---------------------------------
    def _event(self, line):
        """Parse one closed event into what the page overlays on the traces.

        The head line arrives first, then one line per active channel - printBundle()
        prints the whole group in one go.  The channel lines are held until the
        window's *primary* channel reports, because that is the channel whose
        crossing opened the window and whose numbers the verdict was computed from.
        A RAILED or absent channel prints no numbers; the group is then closed with
        whichever channel did report, and the tag names it.
        """
        m = EVENT_HEAD.match(line)
        if m:
            self.flush_event()               # the previous group ended unfinished
            self.pending = {
                'index':     int(m.group(1)),
                'valid':     1 if m.group(2) == 'VALID' else 0,
                'reason':    int(m.group(3)),
                'reason_name': m.group(4),
                'mode':      m.group(9),
                'primary':   int(m.group(8)),
                'start':     self.us_to_sample(int(m.group(5))),
                'end':       self.us_to_sample(int(m.group(6))),
                'width_smp': int(m.group(7)),
                'chs':       [],
            }
            return True

        c = EVENT_CHANNEL.match(line)
        if c is not None and self.pending is not None:
            ch = {
                'channel':     int(c.group(1)),
                'max':         int(c.group(2)),
                'area':        int(c.group(3)),
                # This channel's own width.  In the default LINKED mode it equals the
                # window above; in per-channel mode a channel can be narrower, and
                # the window span is the only extent the lines pin down.
                'width_smp':   int(c.group(5)),
                'base':        int(c.group(6)),
                'verdict':     c.group(7),
                'reason_name': c.group(8) or c.group(7),
            }
            self.pending['chs'].append(ch)
            if ch['channel'] == self.pending['primary']:
                self.flush_event()
            return True

        if self.pending is not None and line.startswith('ch'):
            self.flush_event()               # RAILED / absent: no numbers to draw
        return False

    def flush_event(self):
        """Hand the decision collected so far to StreamState.

        The primary channel's numbers win; if a group never reported them (it
        railed, or the stream was cut short), the first channel that did report is
        used so the window still gets an overlay.
        """
        pend, self.pending = self.pending, None
        if pend is None or not pend['chs']:
            return
        chans = pend.pop('chs')
        primary = pend.pop('primary')
        pend.update(next((c for c in chans if c['channel'] == primary), chans[0]))
        self.state.add_event(pend)

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
                            window_s=self.state.window_sec,
                            keep_s=self.state.keep_sec)
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
        # --range selects the AD7606 RANGE strap, and the firmware's matching
        # SIGNALGEN_ADC_PER_DAC has to be set the same way (2.64 at +/-10 V, 5.28 at
        # +/-5 V - see config.h).  Mirror that here: the model works in counts per
        # DAC code, so with the wrong factor a +/-5 V run would report every peak at
        # twice its programmed height.
        self.model   = sm.SignalModel(adc_per_dac=gen_scale_counts(range_v),
                                      **overrides)
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
        """The M7's own wording, so the log pane reads like a monitor session.

        Word for word printBundle() in sorter-firmware.ino, window included: the
        parser turns these two lines into the decision overlay the page draws, so
        feeding them through feed_line() is what makes the simulator exercise the
        very path the board's serial text takes.
        """
        self.parser.feed_line('Event #%d  %sreason=%d %s  window=%d..%dus/%dsmp  '
                              'primary=ch0  mode=linked'
                              % (m['index'],
                                 'VALID     ' if m['valid'] else 'rejected  ',
                                 m['reason'], REASON_NAME[m['reason']],
                                 m['start_us'], m['start_us'] + m['width_us'],
                                 m['width_samples']))
        self.parser.feed_line('  ch0  max=%d  area=%d  width=%dus/%dsmp  base=%.0f  %s'
                              % (m['max_value'], int(m['area']), m['width_us'],
                                 m['width_samples'], m['baseline'],
                                 'VALID' if m['valid']
                                 else 'rejected ' + REASON_NAME[m['reason']]))

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
  #view  { width: 150px; vertical-align: -3px; }
  #viewout { display: inline-block; min-width: 58px; color: #555;
             font-variant-numeric: tabular-nums; }
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
  <label>window <input type="range" id="view" min="0.1" max="__HISTORY__" step="0.1"
                       value="__VIEW__"><span id="viewout">__VIEW__ s</span></label>
  <label><input type="checkbox" id="autoy" checked> auto Y</label>
  <label><input type="checkbox" id="fullscale"> full scale</label>
  <label><input type="checkbox" id="showgen" checked> expected waveform</label>
  <label><input type="checkbox" id="showevents" checked> detector overlay</label>
  <label><input type="checkbox" id="pause"> pause</label>
  <label><input type="checkbox" id="clear"> clear</label>
</div>
<div id="log"></div>
<script>
'use strict';
const VIEW_S  = __VIEW__;        // rolling window the page opens with, seconds
let   KEEP_S  = __HISTORY__;     // seconds kept per trace: the slider's ceiling
const MIN_VIEW_S = 0.1;          // .. and its floor (the slider's min attribute)
const COLORS  = __COLORS__;      // one colour per measured channel
const RANGE_V = __RANGE__;       // AD7606 input range (full scale = +/-RANGE_V V)
const ADC_FULL_SCALE = 32768;    // counts the AD7606 reports at that full scale
const ML = 78, MR = 16, MT = 26, MB = 36;             // canvas margins
// Detector overlay.  These are deliberately *not* in TRACE_COLORS: an annotation
// must never be mistaken for a measured channel.  Accepted windows are deep green,
// rejected ones orange, and the baseline the detector froze is deep violet.
const EV_OK = '#1b5e20', EV_BAD = '#e65100', EV_BASE = '#4a148c';
const EV_FILL_OK = 'rgba(27,94,32,0.10)', EV_FILL_BAD = 'rgba(230,81,0,0.10)';
/* The two ways a window can be technically right and still hide the truth, each in a
   colour of its own so neither reads as a measured channel:
     EV_DIP     - the trace falls between two peaks, i.e. one window covers a pair;
     EV_SUPPORT - the reported max is more than the trace can carry: no plotted sample
                  comes near it, or the one that does stands alone.  An impulse is the
                  usual reason (the decimation dropped it), a pulse sharper than the
                  trace can draw is the other - the marker says "look here", not
                  "impulse did it". */
const EV_DIP = '#00695c', EV_SUPPORT = '#0277bd';
const EV_LOBES    = 2;      // excursions above half height that make "more than one"
const EV_DIP_PTS  = 2;      // plotted points an excursion has to hold, either way
const EV_DIP_DROP = 0.4;    // of a peak's own height: how far down it is over
const EV_MAX_PTS  = 4;      // plotted points a window needs for max= to be judged
const EV_MAX_MISS = 0.25;   // .. and how far short of the reported max is too far
const EV_NEAR_SPAN = 0.15;  // of that window, each side of the apex: "the shape"
const EV_SPIKE_PTS = 6;     // plotted points a lone apex has to sit inside first
const EV_LANES = 3;              // tag rows packed along the top of the pane
const EV_KEEP = 400;             // decisions kept in the page (mirrors EVENT_KEEP)
const FRAME_PTS = 64;            // points in one stream frame (mirrors POINTS)

const el = (id) => document.getElementById(id);
const cv = el('plot'), g = cv.getContext('2d');

let meta = { channels: [], decim: 10, sample_rate: 10000, range: RANGE_V,
             window_s: VIEW_S, keep_s: KEEP_S,
             wf: 0, streaming: false, source: '?' };
let gen = null, chans = [];
let showGen = true, autoY = true, fullScale = false, paused = false, frozen = null;
let showEvents = true, events = [];      // detector decisions, oldest first
let flagTally = { judged: 0, dip: 0, max: 0 };  // windows the trace disagreed with
let received = 0, t0 = 0, rate = 0, newest = null, dirty = true, lastDraw = 0;
let viewSec = VIEW_S, viewTouched = false;   // the rolling window, in seconds

/* Points per second the page receives: the window and the kept buffer are both
   measured in *time*, so the point counts behind them follow the board's decimation. */
const pps = () => {
  const sr = meta.sample_rate || 10000, d = meta.decim || 1;
  return Math.max(1e-9, sr / d);
};

/* How much is kept per trace - the widest window the slider offers. */
const keepPoints = () => Math.max(2 * FRAME_PTS, Math.round((meta.keep_s || KEEP_S) * pps()));

/* The rolling window itself, in plotted points. */
const viewPoints = () => Math.max(2, Math.round(viewSec * pps()));

/* Seconds the page holds: keepPoints() has a floor of two frames, so at the very
   shortest settings this is a shade more than KEEP_S - the slider follows it. */
const keepSeconds = () => keepPoints() / pps();

/* Keep the slider and its readout honest: how far it may go, and where it is. */
function syncView(){
  const sl = el('view');
  sl.max   = keepSeconds().toFixed(1);
  sl.value = Math.min(viewSec, keepSeconds());
  el('viewout').textContent = viewSec.toFixed(2) + ' s';
}

function newSeries(label, dashed, color){
  return { label: label, first: null, vals: [], dashed: dashed, color: color };
}

function buildSeries(){
  chans = (meta.channels || []).map((c, k) =>
            newSeries('ch' + c, false, COLORS[k % COLORS.length]));
  gen   = newSeries('expected', true, '#1565c0');
  received = 0; t0 = performance.now(); newest = null; frozen = null;
  events = [];
  flagTally = { judged: 0, dip: 0, max: 0 };
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
  const ep = document.createElement('span'), esw = document.createElement('span');
  esw.className = 'sw'; esw.style.background = 'rgba(27,94,32,0.35)';
  ep.appendChild(esw);
  ep.appendChild(document.createTextNode(' detector window'));
  const bsw = document.createElement('span');
  bsw.className = 'sw'; bsw.style.background = EV_BASE;
  ep.appendChild(bsw);
  ep.appendChild(document.createTextNode(
    ' frozen baseline, reported max (dashed; green accepted, orange rejected)'));
  /* .. plus the two things only the trace can show: a dip between two peaks, and a
     max no sample of the window can reach. */
  const dsw = document.createElement('span');
  dsw.className = 'sw'; dsw.style.background = EV_DIP;
  ep.appendChild(dsw);
  const msw = document.createElement('span');
  msw.className = 'sw'; msw.style.background = EV_SUPPORT;
  ep.appendChild(msw);
  ep.appendChild(document.createTextNode(
    ' dip between peaks / max above the trace'));
  /* The overlay key is only meaningful while the overlay is on. */
  if (showEvents) lg.appendChild(ep);
}

function metaText(){
  const sr = meta.sample_rate || 0, d = meta.decim || 1;
  return 'source ' + meta.source
       + '   ' + (meta.streaming ? 'stream on' : 'stream off')
       + '   ' + (sr / 1000).toFixed(1) + ' kSPS, decim ' + d
       + '   window ' + viewSec.toFixed(2) + ' s (' + viewPoints() + ' pts, '
       + keepPoints() + ' kept = ' + keepSeconds().toFixed(1) + ' s)'
       + '   range +/-' + meta.range + ' V  (full scale +/-32768 counts)'
       + '   waveform gen ' + (meta.wf ? 'on' : 'off');
}

function applyMeta(m){
  const changed = (m.channels || []).length !== chans.length;
  meta = m;
  /* The server owns the two window numbers - it is what keeps (and re-primes) the
     samples - so take them from the meta line unless the slider has been touched
     already, and never let the window exceed the seconds being kept. */
  if (m.keep_s > 0) KEEP_S = m.keep_s;
  if (m.window_s > 0 && !viewTouched) viewSec = m.window_s;
  viewSec = Math.max(MIN_VIEW_S, Math.min(viewSec, keepSeconds()));
  syncView();
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

/* A closed event, parsed out of the M7's log lines: the window it claimed (in
   sample indices), its width, the baseline it froze, the peak it reported on top
   of that baseline and the verdict.  Drawing these over the traces is what makes
   a decision auditable - a pulse buried in noise or two pulses merged into one
   window cannot be judged from the trace alone. */
function applyEvent(ev){
  events.push(ev);
  if (events.length > EV_KEEP) events.splice(0, events.length - EV_KEEP);
  dirty = true;
}

/* Every decision still held, replayed to a browser that just connected. */
function applyEvents(list){
  events = (list || []).slice(-EV_KEEP);
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
  const extra = s.vals.length - keepPoints();
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
/* The rolling window: the last `viewSec` seconds of the stream, right-aligned on the
   newest sample.  The scale is fixed in *time*, so the traces scroll left at the rate
   the samples arrive and the pane always holds the same number of points - that is
   what keeps events from piling up in it.  Two consequences worth knowing:

     * before a full window has arrived the pane shows what there is and the scale
       grows with it (clamped at t = 0, so the times never go negative);
     * only the points inside the window are scanned for Y - auto Y fits what is on
       screen instead of a peak that has already scrolled out of it. */
function visible(){
  const d = meta.decim || 1, sr = meta.sample_rate || 10000;
  const all = (showGen ? [gen] : []).concat(chans);
  let x1 = -Infinity;
  all.forEach((s) => {
    if (!s || s.first === null || !s.vals.length) return;
    const last = s.first + (s.vals.length - 1) * d;
    if (last > x1) x1 = last;
  });
  if (x1 === -Infinity) return null;
  let x0 = x1 - viewSec * sr;
  if (x0 < 0) x0 = 0;

  let y0 = Infinity, y1 = -Infinity;
  all.forEach((s) => {
    const pts = traceIn(s, x0, x1);
    for (let k = 0; k < pts.length; k++){
      if (pts[k].v < y0) y0 = pts[k].v;
      if (pts[k].v > y1) y1 = pts[k].v;
    }
  });
  if (y0 === Infinity) return null;          // nothing of the window is plotted
  if (fullScale){
    // Pin the axis to the ADC's own full scale.  The traces are what the AD7606
    // reports, so only a real input that spans the strap fills the pane - the
    // DAC loopback always stops at its clamp (3.1 V), i.e. 10166 counts at
    // +/-10 V and 20333 at +/-5 V.  Auto Y would zoom onto that and hide how
    // much of the scale is left.
    y0 = -ADC_FULL_SCALE; y1 = ADC_FULL_SCALE;
  } else if (autoY || frozen === null){
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

/* The time gridlines: the first step in the 1-2-5 series that puts at most ten
   lines across the window, in sample indices.  A 1 s window gets 0.1 s, an 8 s
   window 1 s - so the labels stay readable at any window length (they are drawn
   every stepS seconds, i.e. the pane never carries more than eleven of them). */
const TICK_STEPS = [0.01, 0.02, 0.05, 0.1, 0.2, 0.25, 0.5, 1, 2, 5, 10];
function xTicks(view){
  const sr = meta.sample_rate || 10000;
  const spanS = (view.x1 - view.x0) / sr;
  let stepS = TICK_STEPS[TICK_STEPS.length - 1];
  for (let k = 0; k < TICK_STEPS.length; k++){
    if (spanS / TICK_STEPS[k] <= 10){ stepS = TICK_STEPS[k]; break; }
  }
  return { step: stepS * sr, stepS: stepS, dec: stepS < 0.1 ? 3 : 2 };
}

function draw(){
  const w = cv.width, h = cv.height;
  const ml = ML, mr = MR, mt = MT, mb = MB;
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
  const ndiv = (view.y1 - view.y0 === 2 * ADC_FULL_SCALE) ? 8 : 5;
  for (let k = 0; k <= ndiv; k++){
    const v = view.y0 + (view.y1 - view.y0) * k / ndiv;
    const y = Math.round(Y(v)) + 0.5;
    g.beginPath(); g.moveTo(ml, y); g.lineTo(ml + pw, y); g.stroke();
    g.fillText(v.toFixed(0), ml - 6, y);
  }

  /* A vertical line every step of captured time (see xTicks). */
  const sr = meta.sample_rate || 10000, d = meta.decim || 1;
  const tk = xTicks(view);
  g.textAlign = 'center'; g.textBaseline = 'top';
  for (let n = Math.ceil(view.x0 / tk.step); n * tk.step <= view.x1; n++){
    const i = n * tk.step;
    const x = Math.round(X(i)) + 0.5;
    g.beginPath(); g.moveTo(x, mt); g.lineTo(x, mt + ph); g.stroke();
    g.fillText((i / sr).toFixed(tk.dec) + ' s', x, mt + ph + 6);
  }

  /* The detector's decision goes under the traces, its tags above them. */
  const tags = [];
  if (showEvents) drawEvents(view, X, Y, tags);

  const order = (showGen ? [gen] : []).concat(chans);
  order.forEach((s) => {
    if (!s || s.first === null || s.vals.length < 2) return;
    /* Only the points inside the window: the buffer holds keepPoints() of them, so
       drawing all of it would draw (and join) points that are not shown. */
    const k0 = Math.max(0, Math.ceil((view.x0 - s.first) / d));
    const k1 = Math.min(s.vals.length - 1, Math.floor((view.x1 - s.first) / d));
    if (k1 - k0 < 1) return;
    g.strokeStyle = s.color;
    g.lineWidth = s.dashed ? 1.5 : 1.2;
    g.setLineDash(s.dashed ? [6, 4] : []);
    g.beginPath();
    for (let k = k0; k <= k1; k++){
      const x = X(s.first + k * d), y = Y(s.vals[k]);
      if (k > k0) g.lineTo(x, y); else g.moveTo(x, y);
    }
    g.stroke();
    g.setLineDash([]);
  });

  if (tags.length) drawEventTags(tags);

  g.strokeStyle = '#bbb';
  g.strokeRect(ml + 0.5, mt + 0.5, pw, ph);
}

/* ---- judging a window against the trace -------------------------------- */
/* The log's numbers are measured on EVERY raw sample; the trace carries only every
   DECIM-th one.  A window can therefore claim a peak the trace never reaches, or
   cover two pulses without saying so.  Neither is visible in the log lines, and
   neither is necessarily wrong:
     * a dip is *meant* to fuse two pulses - the window closes only after
       DET_END_DEBOUNCE = 2 consecutive samples below base + k_off*sigma, so a
       brief dip between two peaks cannot end it.  The overlay reports the dip
       instead of calling the merge a mistake;
     * an impulse is a sample the detector is right to take, but it masks the
       height of the pulse underneath it, which is all a reader of max= needs.
   Both are reported *next to* the detector's own numbers, as what the trace
   supports, never in place of them.  Only these two checks are shipped: an "area
   over width too high, so an impulse narrowed it" test was measured over the
   simulator and cannot separate the two - a clamp-limited pulse and an
   impulse-widened one overlap completely on that ratio. */

function seriesFor(ch){
  for (let k = 0; k < chans.length; k++) if (chans[k].label === 'ch' + ch) return chans[k];
  return null;
}

/* The plotted samples of `s` inside [i0, i1] as { i, v } - one point per DECIM-th raw
   sample, i.e. the points the trace is drawn from. */
function traceIn(s, i0, i1){
  const out = [], d = meta.decim || 1;
  if (s === null || s.first === null || !s.vals.length) return out;
  const k0 = Math.max(0, Math.ceil((i0 - s.first) / d));
  const k1 = Math.min(s.vals.length - 1, Math.floor((i1 - s.first) / d));
  for (let k = k0; k <= k1; k++) out.push({ i: s.first + k * d, v: s.vals[k] });
  return out;
}

/* What the trace says about one window.  Cached on the event: the answer changes
   only while the window's own samples are still arriving. */
function judge(ev){
  if (ev.q && (ev.qdone || ev.qcov === newest)) return ev.q;
  const d = meta.decim || 1;
  const q = { ch: null, lobes: 0, dip: null, dipIdx: null, tops: [],
              support: null, why: null, apexIdx: null, apexV: null };
  const s = seriesFor(ev.channel);
  const pts = (s === null) ? [] : traceIn(s, ev.start, ev.end);

  if (pts.length){
    q.ch = s.label;
    let top = -Infinity, kTop = 0;
    for (let k = 0; k < pts.length; k++) if (pts[k].v > top){ top = pts[k].v; kTop = k; }

    if (ev.max > 0 && top > ev.base){       // a window that claims no peak has none
      /* A peak is an excursion the trace *holds* above half of the window's own height
         - taken from the trace, so a max an impulse inflated cannot hide the peak
         underneath it - and it is over once the trace has come back down to
         EV_DIP_DROP of that peak's own height and stayed there for EV_DIP_PTS plotted
         points (which are DECIM raw samples apart, so no single sample fakes it).
         Both sides of the pair need the run: one point of dither above the line cannot
         open a peak, and one below it cannot close one.  Measuring the come-down
         against the peak's own height, and leaving a gap between the two levels, is
         what keeps a slow, noisy edge in one piece - walking down through half height
         the trace crosses that level again and again, but it never climbs back to
         where a new peak has to start. */
      const hi = ev.base + 0.5 * (top - ev.base);
      let inLobe = false, up = 0, low = 0, lowMin = Infinity, lowIdx = null;
      for (let k = 0; k < pts.length; k++){
        const v = pts[k].v;
        const t = q.tops.length ? q.tops[q.tops.length - 1] : null;
        if (!inLobe && v >= hi){                  // an excursion, if the trace holds it
          if (++up >= EV_DIP_PTS){
            inLobe = true; q.lobes++; q.tops.push({ i: pts[k].i, v: v });
            up = 0; low = 0; lowMin = Infinity;
          }
        } else if (!inLobe){
          up = 0;
        } else if (v >= ev.base + EV_DIP_DROP * (t.v - ev.base)){  // still that peak
          if (v > t.v){ t.v = v; t.i = pts[k].i; }
          low = 0; lowMin = Infinity;
        } else {                                  // under this peak's own drop level
          low++;
          if (v < lowMin){ lowMin = v; lowIdx = pts[k].i; }
          if (low >= EV_DIP_PTS){                 // down and stayed: peak over
            inLobe = false; low = 0;
            if (q.dip === null || lowMin - ev.base < q.dip){
              q.dip = lowMin - ev.base; q.dipIdx = lowIdx;
            }
          }
        }
      }
    }

    /* Was the reported max ever *drawn* as a peak?  Two ways it cannot be, and they
       are worth different amounts of trust:
         'trace' - no plotted sample comes near it: the impulse fell between two DECIM
                   samples, so nothing on the canvas reaches the line the verdict
                   colour drew.  The trace disproves the max;
         'lone'  - the one sample that does reach it stands alone, with the shape of the
                   pulse far below it on both sides.  A real apex is broad: within
                   EV_NEAR_SPAN of the window of itself the trace is still most of its
                   own height, and scaling that band with the window is what keeps this
                   right for wide pulses.  A pulse sharper than the trace can draw looks
                   the same, so this one says "look here", not "impulse". */
    const span = (ev.end - ev.start) / d;
    if (span >= EV_MAX_PTS && ev.max > 0){        // a max of 0 claims nothing
      if ((ev.base + ev.max) - top > EV_MAX_MISS * ev.max){
        q.support = top - ev.base; q.why = 'trace';      // simply not drawn
      } else if (q.lobes < EV_LOBES && span >= EV_SPIKE_PTS){
        /* Only for a window the trace calls one peak: with two lobes the apex belongs
           to a peak the detector did catch, and the dip note already says so. */
        const near = Math.max(1, Math.round(EV_NEAR_SPAN * span));
        let nv = -Infinity;
        const j0 = Math.max(0, kTop - near), j1 = Math.min(pts.length - 1, kTop + near);
        for (let j = j0; j <= j1; j++) if (j !== kTop && pts[j].v > nv) nv = pts[j].v;
        if (nv > -Infinity && top - nv > 0.5 * (top - ev.base)){
          q.support = nv - ev.base; q.why = 'lone';
          q.apexIdx = pts[kTop].i; q.apexV = top;
        }
      }
    }
  }

  ev.q = q; ev.qcov = newest;
  if (newest !== null && newest > ev.end){         // the whole window is in hand
    ev.qdone = true;
    if (q.ch !== null){                            // tally it once, for the stats line
      flagTally.judged++;
      if (q.lobes >= EV_LOBES) flagTally.dip++;
      if (q.support !== null) flagTally.max++;
    }
  }
  return q;
}

/* The M7's decision, drawn where it was taken: the window it claimed (with the
   width in samples it reports), the baseline it froze for that window, and the
   peak it reported on top of that baseline.  That is enough to tell a real
   detection from a lucky one when the pulse is buried in noise, or to see two
   pulses swallowed by a single window - neither is visible in the trace alone.

   In the default LINKED mode (SORTER_METRIC_MODE = 1) the window *is* the detected
   width; in per-channel mode a channel can be narrower than the group window, and
   the tag then spells out both.

   Two things the lines cannot show are read off the trace and drawn in their own
   colours: a dip between two peaks inside one window (EV_DIP) and a max the window's
   own samples do not support (EV_SUPPORT).  See judge(). */
function drawEvents(view, X, Y, tags){
  const top = MT, bot = cv.height - MB;
  events.forEach((ev) => {
    if (ev.end < view.x0 || ev.start > view.x1) return;     // scrolled away
    const col = ev.valid ? EV_OK : EV_BAD;
    const x0 = X(ev.start), x1 = Math.max(X(ev.end), x0 + 1);
    /* Reference lines sit on a half pixel (like the gridlines) so they stay 1 px
       and crisp instead of smearing over two rows at fractional positions. */
    const e0 = Math.round(x0) + 0.5, e1 = Math.round(x1) + 0.5;

    g.fillStyle = ev.valid ? EV_FILL_OK : EV_FILL_BAD;
    g.fillRect(x0, top, x1 - x0, bot - top);
    g.strokeStyle = col; g.lineWidth = 1; g.setLineDash([3, 3]);
    [e0, e1].forEach((x) => {                  // the window's own edges
      g.beginPath(); g.moveTo(x, top); g.lineTo(x, bot); g.stroke();
    });

    const yb = Math.round(Y(ev.base)) + 0.5;
    const ym = Math.round(Y(ev.base + ev.max)) + 0.5;
    g.strokeStyle = EV_BASE; g.setLineDash([4, 3]);         // frozen baseline
    g.beginPath(); g.moveTo(e0, yb); g.lineTo(e1, yb); g.stroke();
    g.strokeStyle = col; g.setLineDash([6, 3]);             // the peak it reported
    g.beginPath(); g.moveTo(e0, ym); g.lineTo(e1, ym); g.stroke();
    g.setLineDash([]);

    /* What the trace says the window does not: a dip between two peaks, and a max the
       window's own samples never reach (or reach only at one lone point). */
    const q = judge(ev);
    if (q.lobes >= EV_LOBES){
      const xd = Math.round(X(q.dipIdx)) + 0.5;
      const yd = Math.round(Y(ev.base + q.dip)) + 0.5;
      g.fillStyle = EV_DIP;                                // every peak it swallowed
      q.tops.forEach((t) => {
        g.fillRect(Math.round(X(t.i)) - 1, Math.round(Y(t.v)) - 1, 3, 3);
      });
      g.strokeStyle = EV_DIP; g.lineWidth = 2;             // .. and the dip between
      g.beginPath(); g.moveTo(xd, yb); g.lineTo(xd, yd); g.stroke();
      g.beginPath(); g.moveTo(xd - 3, yd); g.lineTo(xd + 3, yd); g.stroke();
      g.lineWidth = 1;
    }
    if (q.support !== null){
      /* Where the claim would sit if it matched what the window's own samples
         support - always below the line the verdict colour drew. */
      const ys = Math.round(Y(ev.base + q.support)) + 0.5;
      g.strokeStyle = EV_SUPPORT; g.lineWidth = 1; g.setLineDash([1, 3]);
      g.beginPath(); g.moveTo(e0, ys); g.lineTo(e1, ys); g.stroke();
      g.setLineDash([]);
      if (q.apexIdx !== null){                    // .. and the lone sample it stands on
        g.fillStyle = EV_SUPPORT;
        g.fillRect(Math.round(X(q.apexIdx)) - 1, Math.round(Y(q.apexV)) - 1, 3, 3);
      }
    }

    const notes = [];
    if (q.lobes >= EV_LOBES){
      notes.push({ col: EV_DIP, text: '  ' + q.lobes + ' peaks  dip '
                    + (q.dip > 0 ? '+' : '') + Math.round(q.dip) });
    }
    if (q.support !== null){
      /* "!" is the hard one - nothing the trace drew reaches the max; "?" the softer
         one - only a lone sample does.  Both are read off the trace, neither is proof. */
      notes.push({ col: EV_SUPPORT, text: (q.why === 'trace' ? '  !max (' : '  ?max (')
                    + q.why + ' ' + Math.round(q.support) + ')' });
    }

    tags.push({ x: x0 + 3, col: col, notes: notes,
                text: '#' + ev.index + ' ch' + ev.channel + ' '
                      + (ev.valid ? 'VALID' : 'rej ' + ev.reason_name)
                      + '  w=' + ev.width_smp + 'smp  max=' + ev.max
                      + '  base=' + ev.base
                      + (ev.width_smp === (ev.end - ev.start) ? ''
                         : '  (win ' + (ev.end - ev.start) + 'smp)') });
  });
}

/* One tag per event, packed into a few rows across the top of the pane so that
   neighbouring windows (the busy case: merged pulses) stay readable.  A tag is the
   measured numbers in the verdict colour, plus the overlay's own findings after
   them, each in the colour it was drawn in on the canvas. */
function drawEventTags(tags){
  g.font = 'bold 11px Segoe UI';
  g.textAlign = 'left'; g.textBaseline = 'top';
  const laneRight = [];
  tags.sort((a, b) => a.x - b.x);
  tags.forEach((t) => {
    const notes = t.notes || [];
    let textW = g.measureText(t.text).width;
    notes.forEach((n) => { textW += g.measureText(n.text).width; });
    const tw = textW + 10;
    let lane = 0;
    while (lane < EV_LANES - 1 && t.x < laneRight[lane] + 4) lane++;
    laneRight[lane] = Math.max(laneRight[lane] || t.x, t.x + tw);
    const y = MT + 4 + lane * 15;
    g.fillStyle = 'rgba(255,255,255,0.9)';
    g.fillRect(t.x, y, tw, 14);
    g.strokeStyle = t.col; g.lineWidth = 1;
    g.strokeRect(t.x + 0.5, y + 0.5, tw - 1, 13);
    let x = t.x + 5;
    g.fillStyle = t.col;
    g.fillText(t.text, x, y + 2);
    x += g.measureText(t.text).width;
    notes.forEach((n) => {
      g.fillStyle = n.col;
      g.fillText(n.text, x, y + 2);
      x += g.measureText(n.text).width;
    });
  });
  g.font = '11px Segoe UI';
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
       + '   trace holds ' + (gen ? gen.vals.length : 0) + ' points, '
       + viewPoints() + ' in the ' + viewSec.toFixed(2) + ' s window'
       + eventsText()
       + (paused ? '   [PAUSED]' : '');
}

/* How many decisions the page is holding, and how many of them were accepted. */
function eventsText(){
  if (!events.length) return '';
  let ok = 0;
  for (let k = 0; k < events.length; k++) if (events[k].valid) ok++;
  return '   windows ' + events.length + ' (' + ok + ' accepted)' + flagsText();
}

/* What the trace said about the windows it was asked to judge - a running count, so
   the numbers also cover windows that have since scrolled out of the trace.  "max?"
   is a reported peak no plotted sample came near, or one only a single point reached:
   the trace cannot prove an impulse did it, but it is where to look. */
function flagsText(){
  if (!flagTally.judged) return '';
  return '   read back ' + flagTally.judged + ' windows: '
       + flagTally.dip + ' two-peak, ' + flagTally.max + ' max?';
}

/* ---- live wiring ------------------------------------------------------ */
const es = new EventSource('/stream');
es.onmessage = (ev) => {
  let m;
  try { m = JSON.parse(ev.data); } catch (e) { return; }
  if (m.kind === 'meta')        applyMeta(m);
  else if (m.kind === 'data')   applyData(m);
  else if (m.kind === 'log')    addLog(m.line);
  else if (m.kind === 'event')  applyEvent(m);
  else if (m.kind === 'events') applyEvents(m.list);
};
es.onopen  = () => { el('meta').textContent = metaText(); };
es.onerror = () => { el('meta').textContent = 'stream lost - reconnecting...'; };

el('autoy').onchange   = (e) => { autoY  = e.target.checked; dirty = true; };
el('fullscale').onchange = (e) => { fullScale = e.target.checked; dirty = true; };
el('showgen').onchange = (e) => { showGen = e.target.checked; dirty = true; };
el('showevents').onchange = (e) => {
  showEvents = e.target.checked; renderLegend(); dirty = true;
};
el('pause').onchange   = (e) => { paused = e.target.checked; dirty = true; };
el('clear').onchange   = (e) => {
  if (e.target.checked){ buildSeries(); el('log').innerHTML = ''; }
  e.target.checked = false;
};
/* The rolling window is the page's own knob: widening it redraws from the samples
   already kept (keepPoints), narrowing it tightens what is on screen - no waiting
   for the stream either way, and the events in view scale with the window. */
el('view').oninput = (e) => {
  viewTouched = true;
  const v = +e.target.value;
  viewSec = Math.max(MIN_VIEW_S, Math.min(keepSeconds(), v > 0 ? v : VIEW_S));
  syncView();
  el('meta').textContent = metaText();
  dirty = true;
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
syncView();
requestAnimationFrame(frame);
</script>
</body>
</html>
"""


def page_html(window_s, keep_s, range_v):
    """The page, with its own constants filled in.

    `window_s` is the rolling window the page opens with and `keep_s` the seconds it
    keeps per trace (the slider's ceiling), both in seconds: the page holds time, not
    a point count, so the pane looks the same whatever the board's stream rate is.
    """
    return (PAGE
            .replace('__VIEW__', '%g' % window_s)
            .replace('__HISTORY__', '%g' % keep_s)
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
        body = page_html(self.server.state.window_sec, self.server.state.keep_sec,
                         self.server.range_v).encode('utf-8')
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
               '  py -3 test/live_plot.py --sim --window 2 --http-port 9000\n'
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
    view.add_argument('--window', type=float, default=VIEW_SEC,
                      help='rolling window the page opens with, in seconds - the page '
                           'has a slider for it (default %(default)s)')
    view.add_argument('--history', type=float, default=HISTORY_SEC,
                      help='seconds kept per trace, i.e. how far the page may widen '
                           'its window without waiting for new samples (default '
                           '%%(default)s, capped at %g s)' % MAX_KEEP_SEC)
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

    # The window is measured in seconds, so an old point count ("--window 2000") would
    # ask for a buffer no page could draw: keep the two sensible and in order.
    view_sec = min(max(args.window, MIN_VIEW_SEC), MAX_KEEP_SEC)
    keep_sec = min(max(args.history, view_sec), MAX_KEEP_SEC)
    if max(args.history, args.window) > MAX_KEEP_SEC:
        print('live_plot: window/history above %g s are capped there - the sample '
              'buffers are sized for that much' % MAX_KEEP_SEC)
    state = StreamState(window=view_points(keep_sec), window_sec=view_sec,
                        keep_sec=keep_sec)
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
    print('live_plot: rolling window %.2f s, %d points kept (%.1f s) - the page\'s '
          'slider goes down to %.1f s'
          % (state.window_sec, state.window, state.keep_sec, MIN_VIEW_SEC))
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

