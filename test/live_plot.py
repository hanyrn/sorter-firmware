#!/usr/bin/env python3
"""
Live datastream viewer for the AD7606 sample stream - no scope, no third-party
packages.

The M4 fills sample frames continuously (stream_sink.h); the M7 prints them as one
line per point once streaming has been switched on, and stays quiet otherwise, so
the ordinary event log stays readable:

    STREAM on: decim=10 chans=0,1 wf=1 points=64 minw=35
    S,<sample index>,<gen code>,<ch>:<count>,<ch>:<count>,...

This script is that host.  It sends "T1" - the serial monitor's stream-on command,
so the M7 always re-prints its STREAM header - parses the lines and pushes them to
a small web page over Server-Sent Events, which draws a rolling window of the
traces in the browser.  Pure Python stdlib plus a browser: no pyserial, no npm, no
build step.

    py -3 test/live_plot.py --sim              # no board: the bench harness runs
    py -3 test/live_plot.py --port COM7        # the real board
    py -3 test/live_plot.py --stdin < bench.txt    # a capture, or a pipe

--sim runs the firmware without a board.  It starts test/test_bench_detect.exe
(built by test/run_tests.ps1) with --stream, which makes the *shipped*
EventAggregator and EventDetector run over signal_model.h's bench waveform and
print the board's own serial text for that run - the `STREAM on:` header, the `S,`
points and printBundle()'s event lines - then replays that text at 10 kSPS through
the very parser the COM port feeds.  The page therefore draws the firmware's
decisions over the samples they were made on, not a model of them: there is no
Python mirror anywhere in this path.  Everything the harness prints, its own
verdict included, lands in the log pane.

That capture belongs to one *build* of the harness: the log pane names the build it
came from (and says so when the exe is older than the sources it was compiled from),
and rebuilding test_bench_detect.exe while the viewer runs replays the new capture
with the sample indices continuing, instead of leaving the previous waveform on
screen.  That is the usual answer to "the page does not show the updated test
signal" - the other being a board whose firmware was not re-flashed.

    py -3 test/live_plot.py --sim --sim-cycles 6 --sim-speed 2 --window 2
    py -3 test/live_plot.py --sim --sim-start live     # the board's boot reality
    py -3 test/live_plot.py --sim --sim-ladder         # the floor ladder: small peaks

--sim-ladder runs the harness's *floor ladder* instead of the random train
(SIGNALGEN_LADDER_* in config.h): a staircase of isolated pulses whose apexes step up
from inside the noise band, one per slot of the cycle, with no random pulses, glitches
or impulses drawn at all.  It is the one bench pattern whose small apexes the pane's own
vertical scale can show - the random train's span the whole DAC range (~10000 counts),
so a few-hundred-count apex lands in the dither there, and its gaps are 3..90 ms against
120 ms pulses, which leaves such an apex a shoulder on a neighbour rather than a peak.
With the ladder the pane settles near 1000 counts (0.37 px per count): the noise band is
+/-15 px, a 100-count rung 37 px, the 460-count pinnacle 169 px - so "how close to the
baseline can a peak be and still be detected" is something to *look at*.  The same run
prints as a table with `test/test_bench_detect.exe --ladder` (rung by rung, with the
reported max= and the floor between the last two rungs), and its --ladder-width /
--ladder-min / --ladder-step / --ladder-count flags move the staircase - pipe its
--stream into `--stdin` for a width the viewer has no flag for.

--sim-speed N replays the capture at N times real time and `0` replays it as fast
as Python can (handy for filling a small window); the capture itself is always
produced as fast as the host manages, so the pacing is this script's choice.

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
  * the width floor that decision was measured against, drawn on the window itself: a
    band `minw=` samples wide centred on the window (3.5 ms of *duration* at the
    defaults) is the shortest event the detector accepts, so a `w=27smp` tag can be
    *seen* to be under it instead of only being reported as too narrow.  The duration is
    what the floor is about, so the page draws the width each window was judged with as a
    rule of its own, coloured by the *width* question alone: green while it cleared the
    band, orange only when the floor is what refused the window.  A window refused on the
    shape bounds instead (a needle, `rej SHORT_FOR_HEIGHT`) carries a duration well over
    the band, and painting that rule in the rejection colour is what made a rise rejection
    read as a width one.  The
    number travels in the STREAM header, printed by the board, so the drawing cannot
    drift from the firmware it is drawing; a header without it (an older build piped in
    with --stdin) draws no floor,
  * the three things those lines cannot show, read back off the plotted points and
    tagged in colours of their own: `2 peaks  dip +58` when one window swallowed two
    peaks, `!max` / `?max` when the window's own samples do not reach the max it
    reported (an impulse the decimation dropped, most often), and `!rise <=1ms` when
    the trace climbs to its peak faster than a bead's own edge - the shape of a spike,
    not a dome.  All three describe the trace, never replace the measured numbers - see
    *Live sample stream* below.
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
import shutil
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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

# Where the bench harness puts its build (test/run_tests.ps1) and the two ways
# running it can fail loudly enough to be worth translating for the reader.
BENCH_EXE           = 'test_bench_detect.exe'
BENCH_CYCLE_SAMPLES = 20000        # SIGNALGEN_CYCLE_SAMPLES in config.h
BENCH_EXIT_DLL      = -1073741515  # 0xC0000135: its C++ runtime is not on PATH

# --sim and --stdin pace themselves in wall-clock time, but a Windows timer tick is
# ~1 ms, so sleeping once per point would crawl; sleep only when this far ahead
# instead.  (The pacer re-bases after a long stall rather than trying to pay back a
# debt it can never pay off.)
PACE_MIN_SLEEP = 0.004
PACE_REBASE    = -0.05

TRACE_COLORS = ['#c62828', '#2e7d32', '#f9a825', '#6a1b9a',
                '#00838f', '#4e342e', '#ad1457', '#37474f']

# The verdict names are *not* mirrored here any more: the event lines carry the name
# the firmware's own reasonName() printed, and the page shows that word (see _event()
# below).  A table of enum values in this file could only ever disagree with it.

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

    def add_batch(self, first_index, gen, adc, step=DECIM):
        """One frame: gen = DAC codes, adc = [[count, ...], ...] per channel.

        `step` is how many raw samples one point covers - the decimation the header
        announced, not this file's default, so a board (or a harness run) that
        streams at another decimation still lands on the right sample indices.
        """
        n = len(gen)
        with self.lock:
            self.points += n
            self.frames += 1
            for k in range(n):
                self.recent.append((first_index + k * step, gen[k],
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

        STREAM on: decim=10 chans=0,1 wf=1 points=64 minw=35  metadata (sent by `T`)
        S,<sample index>,<gen code>,<ch>:<count>,...      one sample point
        Event #12  VALID  reason=0 OK  window=...         a closed event, plus
          ch0  max=...  area=...  width=...  base=...       its per-channel line

    `minw=` is the detector's floor on the event's *duration*, in samples (config.h's
    DET_MIN_WIDTH_SAMPLES): the page draws it over every window it overlays as a band
    centred on the window and states it beside the point rate, so what a window was
    judged against is on screen - and it is the board that says so.  The per-channel
    `width=` on the event line is that same quantity for that event (the duration the
    detector estimated), while the head line's `window=` is the excursion the measurement
    covered.

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
        self.min_width_samples = 0             # the header's minw=: the judged floor
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
        # The judged width floor, straight from the board (config.h's
        # DET_MIN_WIDTH_SAMPLES): a floor on the event's *duration*, in samples.  0 when
        # the line came from a build that does not print it: the page then draws no floor
        # rather than a guessed one.
        try:
            self.min_width_samples = int(fields.get('minw', 0))
        except ValueError:
            self.min_width_samples = 0
        self.channels = [int(c) for c in fields.get('chans', '').split(',')
                         if c.strip().isdigit()]
        self.state.set_meta(source=self.source, streaming=stream_on,
                            decim=self.decim, channels=self.channels, wf=self.wf,
                            points=int(fields.get('points', POINTS)),
                            min_width=self.min_width_samples,
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
            self.state.add_batch(self.frame_i, self.gen, self.cols, self.decim)
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


def default_bench_exe():
    """The bench harness test/run_tests.ps1 builds, next to this script."""
    here = os.path.dirname(os.path.abspath(__file__))
    exe  = os.path.join(here, BENCH_EXE)
    if not os.path.exists(exe):
        bare = exe[:-4] if exe.endswith('.exe') else exe    # a Unix build has no .exe
        if os.path.exists(bare):
            return bare
    return exe


def bench_runtime_dirs():
    """The folders that hold the harness's C++ runtime, if it is not on PATH.

    run_tests.ps1 builds test_bench_detect.exe with MinGW g++ (or clang++), and that
    links libstdc++-6.dll / libgcc_s_*.dll *dynamically*: the exe only starts when the
    compiler's own bin folder is on PATH, which is why run_tests.ps1 prepends it for
    its runs.  The WinLibs package winget installs is not added to PATH by its
    installer, so --sim would otherwise fail with a bare 0xC0000135 on the very
    machines the bench is built on.  The same fallbacks as the test runner, in the
    same order, so both agree on which compiler is "the" one.
    """
    if os.name != 'nt':
        return []                       # every Unix build finds its libs itself
    found = []
    for name in ('g++.exe', 'clang++.exe'):
        exe = shutil.which(name)
        if exe:
            found.append(os.path.dirname(exe))
    winget = os.path.join(os.environ.get('LOCALAPPDATA', ''),
                          'Microsoft', 'WinGet', 'Packages')
    if os.path.isdir(winget):
        for root, _dirs, files in os.walk(winget):
            if 'g++.exe' in files:
                found.append(root)
                break
    for exe in (r'C:\msys64\mingw64\bin\g++.exe', r'C:\mingw64\bin\g++.exe',
                r'C:\ProgramData\mingw64\bin\g++.exe'):
        if os.path.exists(exe):
            found.append(os.path.dirname(exe))
    return found


# What the harness is compiled from: test/run_tests.ps1 builds test_bench_detect.cpp
# plus the repo headers it pulls in.  Deliberately *not* in the list: config.h - the
# harness mirrors the SIGNALGEN_* numbers from it by hand (its "bench build, mirrored
# from config.h" block), so the waveform a --sim page draws is the waveform baked into
# the exe, and editing config.h alone does not move it.
BENCH_SOURCES = ['test_bench_detect.cpp',
                 os.path.join('..', 'signal_model.h'),
                 os.path.join('..', 'event_aggregator.h'),
                 os.path.join('..', 'event_detector.h'),
                 os.path.join('..', 'peak_metrics.h')]


def bench_newer_sources(exe):
    """Harness sources newer than the harness itself, newest first.

    --sim draws a *built* harness, and a stale exe replays the previous waveform with
    nothing on screen to say so - the one way "the page does not show the updated
    signal" happens without anything actually being broken.  Naming the files turns
    that into a rebuild (`test/run_tests.ps1`) instead of a hunt.
    """
    try:
        built = os.path.getmtime(exe)
    except OSError:
        return []
    here = os.path.dirname(exe) or '.'
    newer = []
    for rel in BENCH_SOURCES:
        try:
            mt = os.path.getmtime(os.path.join(here, rel))
        except OSError:
            continue
        if mt > built + 1.0:        # an edit and its rebuild land in the same second
            newer.append((mt, os.path.basename(rel)))
    return [name for _, name in sorted(newer, reverse=True)]


class FirmwareSource:
    """No board: the firmware's own bench harness prints the very same lines.

    test/test_bench_detect.exe runs the shipped EventAggregator and EventDetector -
    the headers the M4 compiles - over signal_model.h's waveform, and ``--stream``
    makes it print the M7's serial text for that run: the `STREAM on:` header, one
    `S,` line per stream point and printBundle()'s two lines per closed event.  This
    class runs it once, then replays that text at 10 kSPS through the same
    StreamParser the COM port feeds, so what the overlay draws is the firmware's own
    verdict on the samples it was given.  Nothing is interpreted here: text in, text
    out - which is the point, because the mirror this replaced had drifted away from
    the firmware, and a drifted mirror is worse than no view at all.

    The harness's verdict (its counters, and any check it failed) comes back on
    stderr and is copied to the log pane, so `14 event(s) (14 VALID), 0 missed, ...`
    can be read in the browser next to the peaks it is talking about.

    A capture is one build's worth of text: the harness runs when this source starts
    and is replayed from then on.  Rebuilding test_bench_detect.exe while the viewer
    runs is therefore picked up here rather than ignored - the new capture is replayed
    with its sample indices continuing where the last one stopped, so the page never
    has to be restarted to show a changed waveform - and the build actually on screen
    is named in the log pane, a stale exe older than its sources included.
    """

    def __init__(self, state, range_v, speed=1.0, seed=None, exe=None, cycles=4,
                 frontend=None, start=None, hw=False, repeat=False, decim=DECIM,
                 echo=True, ladder=False):
        self.state      = state
        self.range_v    = range_v
        self.speed      = speed
        self.seed       = seed
        self.exe        = exe or default_bench_exe()
        self.cycles     = max(int(cycles), 2)   # the harness clamps to >= 2 cycles
        self.frontend   = frontend              # None = the harness's default (3 taps)
        self.start_mode = start                 # None = hold, the detector's contract
        self.hw         = hw
        self.repeat     = repeat
        self.ladder     = ladder                # --sim-ladder: SIGNALGEN_LADDER_*
        self.decim      = max(int(decim), 1)
        self.parser     = StreamParser(state, 'bench', range_v, echo=echo)
        self.thread     = None
        self.stopped    = threading.Event()
        self.lines      = []
        self.stamp      = self._exe_stamp()   # the build the capture in hand came from

    def argv(self):
        """The harness command line this run is fed by (logged, so a review is
        reproducible)."""
        argv = [self.exe, '--stream', '--cycles', str(self.cycles),
                '--stream-decim', str(self.decim)]
        if self.ladder:
            # SIGNALGEN_LADDER_* (config.h): the staircase of isolated, near-baseline
            # peaks, instead of the random train - the one pattern whose small apexes
            # the pane's own scale can show.  A different width or a finer staircase is
            # the harness's own flags:  test_bench_detect.exe --ladder --ladder-width
            # 200 --stream | py -3 test/live_plot.py --stdin
            argv.append('--ladder')
        if self.frontend is not None:
            argv += ['--frontend', str(int(self.frontend))]
        if self.start_mode is not None:
            argv += ['--start', self.start_mode]
        if self.hw:
            argv.append('--hw')
        if self.seed is not None:
            argv += ['--seed', '0x%X' % self.seed]
        return argv

    def start(self):
        self._report_build()
        self.lines = self._capture()
        self.thread = threading.Thread(target=self._run, name='bench', daemon=True)
        self.thread.start()

    def _report_build(self):
        """Name the harness build this page is drawing - and say if it is stale.

        The page cannot tell one build from another, so a viewer started against an exe
        built before the last edit replays the previous waveform and simply looks
        wrong.  One line in the log pane settles it.
        """
        try:
            built = os.path.getmtime(self.exe)
        except OSError:
            return                      # a missing harness is _capture()'s to report
        self.parser.log('bench: harness built %s (%s)'
                        % (time.strftime('%Y-%m-%d %H:%M:%S',
                                         time.localtime(built)),
                           os.path.basename(self.exe)))
        stale = bench_newer_sources(self.exe)
        if stale:
            self.parser.log('bench: !! %s is older than %s - this run draws the '
                            'waveform baked into the exe, not the tree it was built '
                            'from; rebuild with powershell -ExecutionPolicy Bypass '
                            '-File test\\run_tests.ps1'
                            % (os.path.basename(self.exe), ', '.join(stale)))

    def _capture(self):
        """Run the harness once, now: the board's own text, as --stream prints it."""
        self.stamp = self._exe_stamp()      # a rebuild *during* the run counts too
        argv = self.argv()
        self.parser.log('bench: %s %s' % (os.path.basename(argv[0]),
                                          ' '.join(argv[1:])))
        if abs(self.range_v - 10.0) > 1e-9:
            # The counts the harness prints come out of config.h's SIGNALGEN_ADC_PER_DAC
            # (2.64 counts per DAC code in the +/-10 V strap); --range only scales this
            # page's own overlay, so a mismatched value slides the expected waveform
            # off the measured one.
            self.parser.log('bench: note - --range %g V scales the page overlay only; '
                            'the counts themselves come from config.h '
                            '(SIGNALGEN_ADC_PER_DAC), which is set for +/-10 V'
                            % self.range_v)
        if not os.path.exists(self.exe):
            raise FileNotFoundError(
                '%s is missing - build the harness first: powershell '
                '-ExecutionPolicy Bypass -File test\\run_tests.ps1' % self.exe)
        env = dict(os.environ)
        extra = [d for d in bench_runtime_dirs() if d not in env.get('PATH', '')]
        if extra:
            env['PATH'] = os.pathsep.join(extra + [env.get('PATH', '')])
        proc = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              universal_newlines=True,
                              cwd=os.path.dirname(self.exe) or None, env=env)
        if proc.returncode == BENCH_EXIT_DLL:
            raise OSError('the harness could not load its C++ runtime (0xC0000135): '
                          "put the compiler's bin folder on PATH - run_tests.ps1 does "
                          'that for its own runs')
        if proc.returncode != 0:
            raise OSError('the harness exited %d' % proc.returncode)
        for line in proc.stderr.splitlines():       # the harness's own verdict
            if line.strip():
                self.parser.log('bench: ' + line.strip())
        lines = [l for l in proc.stdout.splitlines() if l.strip()]
        if not lines:
            raise OSError('the harness printed nothing - is %s built from a source '
                          'tree that has --stream?' % self.exe)
        return lines

    def _span(self):
        """Samples in one pass of the capture: the last index plus one decimation.

        --sim-repeat uses it to put the next pass further along the board's timeline
        instead of rewinding it: a sample index that jumps *backwards* would flush
        the open frame and drag the page's window with it.
        """
        last = 0
        for line in self.lines:
            if line.startswith('S,'):
                try:
                    last = max(last, int(line.split(',', 2)[1]))
                except ValueError:
                    continue
        return last + self.decim

    @staticmethod
    def _shift(line, shift, shift_us):
        """Move a verbatim board line onto a later pass's timeline (--sim-repeat)."""
        if shift == 0:
            return line
        if line.startswith('S,'):
            _, _, rest = line.partition(',')
            index, _, tail = rest.partition(',')
            return 'S,%d,%s' % (int(index) + shift, tail)
        if line.startswith('Event #'):
            return re.sub(r'window=(\d+)\.\.(\d+)us',
                          lambda m: 'window=%d..%dus' % (int(m.group(1)) + shift_us,
                                                         int(m.group(2)) + shift_us),
                          line)
        return line

    def _run(self):
        """Replay the harness's text at the board's own 10 kSPS.

        The sample index in the `S,` lines is the clock, exactly as it is on the
        board: `--sim-speed 1` means one second of waveform per second of wall clock,
        and a capture can be replayed faster than the harness could ever produce it.
        """
        paced = self.speed > 0.0                    # --sim-speed 0 = flat out
        span  = self._span()
        shift = 0                                   # samples, 0 on the first pass
        passes = 0

        while not self.stopped.is_set():
            t0 = time.time()
            for line in self.lines:
                if self.stopped.is_set():
                    break
                text = self._shift(line, shift, shift * DT_US)
                if paced and text.startswith('S,'):
                    # Pace on the *pass-local* index: the shift only keeps the board's
                    # timeline moving forward for --sim-repeat, so pacing on the
                    # absolute index would make every pass wait shift/rate seconds
                    # before printing its first point - a hole in the page that grows
                    # by one pass per loop.
                    index = int(text.split(',', 2)[1]) - shift
                    delay = (t0 + index / (SAMPLE_RATE_HZ * self.speed)
                             - time.time())
                    if delay > PACE_MIN_SLEEP:
                        time.sleep(delay)
                    elif delay < PACE_REBASE:       # a stalled host: re-base, do not
                        t0 = time.time()            # try to pay the debt back
                self.parser.feed_line(text)
            passes += 1
            if self.stopped.is_set():
                break
            if not self.repeat:
                self.parser.log('bench: end of the capture - the page keeps the last '
                                'window; Ctrl+C stops the viewer (--sim-repeat loops, '
                                '--sim-cycles lengthens the run, and a rebuild of the '
                                'harness is replayed here)')
                if not self._watch_exe():
                    break
            elif not self._exe_changed(self.stamp):
                shift += span
                self.parser.log('bench: replaying the capture (pass %d, indices '
                                'continue from %d)' % (passes, shift))
                continue
            # A rebuild: fetch the new capture.  The old one is deliberately *not*
            # replayed while we wait - a sample index that jumps backwards would drag
            # the page's window with it (the same reason --sim-repeat shifts).
            lines = None
            while lines is None and not self.stopped.is_set():
                try:
                    lines = self._capture()         # start()'s work, mid-session
                except OSError as exc:
                    self.parser.log('bench: the rebuilt harness did not run: %s' % exc)
                    if not self._watch_exe():       # a half-written exe: wait it out
                        break
            if lines is None:
                break
            shift += span                       # past the samples already on screen
            self.lines = lines
            span = self._span()
            self.parser.log('bench: the harness was rebuilt - replaying the new '
                            'capture (indices continue from %d)' % shift)
        self.parser.flush()

    def _exe_stamp(self):
        """The harness's timestamp, or None when it cannot be read."""
        try:
            return os.path.getmtime(self.exe)
        except OSError:
            return None

    def _exe_changed(self, seen):
        """True if the harness has been rebuilt since `seen` (None = cannot tell)."""
        now = self._exe_stamp()
        return now is not None and seen is not None and now != seen

    def _watch_exe(self):
        """Wait for a rebuild of the harness.  True if it happened, False if we stop.

        Once the capture has been replayed the page is a still picture of a build that
        no longer exists - which is what "the page does not show the updated test
        signal" looks like when nothing is broken.  The exe's own timestamp closes that
        gap without restarting the viewer, and an exe halfway through being written
        just keeps us waiting.
        """
        while not self.stopped.wait(0.2):
            if self._exe_changed(self.stamp):
                return True
        return False

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=1.5)
        self.parser.flush()


class PipeSource:
    """The board's text arriving on stdin: a capture, or another process's pipe.

        py -3 test/live_plot.py --stdin < bench.txt
        test\\test_bench_detect.exe --stream | py -3 test/live_plot.py --stdin

    Nothing but the text protocol crosses the pipe, so this handles a file an
    engineer saved from a monitor session exactly like a live run.  There is no pacing
    here: whoever writes the lines decides the rate - `--stream` makes the harness write
    them line-buffered, one at a time, so a piped run arrives as it is produced and the
    viewer follows it.  A harness run is fast (a 4-cycle capture is 8 s of stream in
    well under a second), so for watching it in real time use --sim, which replays the
    same text at the board's own 10 kSPS; --stdin is the path for a capture or for a
    board that is already printing.
    """

    def __init__(self, state, range_v, stream=None):
        self.state   = state
        self.parser  = StreamParser(state, 'stdin', range_v)
        self.stream  = stream if stream is not None else sys.stdin
        self.thread  = None
        self.stopped = threading.Event()

    def start(self):
        self.thread = threading.Thread(target=self._run, name='stdin', daemon=True)
        self.thread.start()
        return self

    def _run(self):
        # Read the raw bytes in small chunks: a buffered text iterator would sit on
        # the first few kilobytes, and a live review has to see the lines as printed.
        fh = getattr(self.stream, 'buffer', self.stream)
        while not self.stopped.is_set():
            try:
                data = fh.read(256)
            except (OSError, ValueError):
                break
            if not data:
                break
            if isinstance(data, bytes):
                self.parser.feed_bytes(data)
            else:
                self.parser.feed_line(data)
        self.parser.flush()
        if not self.stopped.is_set():
            self.parser.log('stdin: end of input - the page keeps the last window '
                            '(Ctrl+C stops the viewer)')

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
/* The three ways a window can be technically right and still hide the truth, each in a
   colour of its own so none reads as a measured channel:
     EV_DIP     - the trace falls between two peaks, i.e. one window covers a pair;
     EV_SUPPORT - the reported max is more than the trace can carry: no plotted sample
                  comes near it, or the one that does stands alone.  An impulse is the
                  usual reason (the decimation dropped it), a pulse sharper than the
                  trace can draw is the other - the marker says "look here", not
                  "impulse did it";
     EV_RISE    - the trace climbs to its own top faster than the stream can show a
                  real bead's edge.  The AD7606 sees a bead as a dome tens of
                  milliseconds wide; a climb inside EV_RISE_MS is a spike, an edge the
                  analog front end is still settling, or a bad sample - and the same
                  suspects the max notes point at. */
const EV_DIP = '#00695c', EV_SUPPORT = '#0277bd', EV_RISE = '#8e24aa';
/* The width floor the board's header announces (`minw=`, in samples of the event's
   DURATION - config.h's DET_MIN_WIDTH_SAMPLES).  Drawn on every window the overlay draws
   as a *band* the floor wide, centred on the window: the duration the detector estimates
   is centred on the excursion it measured, so a window whose width comes back under the
   band is one it refused as too short - and a `w=27smp` tag can be *seen* to be under the
   floor instead of only read as narrow.  The width that window *was* judged with is drawn
   as the rule in the verdict's colour (see drawEvents).  A colour of its own, for the same
   reason the three above have theirs. */
const EV_FLOOR = '#5d4037';
const EV_LOBES    = 2;      // excursions above half height that make "more than one"
const EV_DIP_PTS  = 2;      // plotted points an excursion has to hold, either way
const EV_DIP_DROP = 0.4;    // of a peak's own height: how far down it is over
const EV_MAX_PTS  = 4;      // plotted points a window needs for max= to be judged
const EV_MAX_MISS = 0.25;   // .. and how far short of the reported max is too far
const EV_NEAR_SPAN = 0.15;  // of that window, each side of the apex: "the shape"
const EV_SPIKE_PTS = 6;     // plotted points a lone apex has to sit inside first
const EV_RISE_MS   = 1.0;   // ms: a climb the trace shows no longer than this is fast
const EV_RISE_FRAC = 0.1;   // of the peak's height: where the climb is taken to start
const EV_LANES = 5;              // tag rows packed along the top of the pane.  Three was
                                 // one too few for a region of long tags: the packing gives
                                 // up at the last lane and draws over whatever is already
                                 // there, which is how a disputed window's numbers (`rej
                                 // SHORT_FOR_HEIGHT  w=54smp ... >= floor 35smp`) ended up
                                 // hidden behind a healthy neighbour's tag.  The extra rows
                                 // are only used where the tags actually collide.
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
let flagTally = { judged: 0, dip: 0, max: 0, rise: 0 };  // windows the trace disagreed with
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

/* One series = the points of one trace, each kept with the sample index it was
   taken at.  The index is what the canvas positions on, never a count of points:
   a frame that never arrived then leaves the gap the stream had instead of sliding
   the whole trace under the detector windows, which are drawn at their absolute
   sample indices (see addPoint / resetSeries / traceIn). */
function newSeries(label, dashed, color){
  return { label: label, idx: [], vals: [], dashed: dashed, color: color };
}

/* Where a sample index falls in a series: idx is strictly increasing, so these bound
   the range to draw or to judge.  Binary search, because a window is judged against
   the points the stream really printed inside it - and after a gap those are not at
   a fixed offset any more.  `atOrAfter` may return idx.length and `atOrBefore` may
   return -1: k0 > k1 then means "nothing in range", which every caller already
   treats as an empty series. */
function atOrAfter(idx, i){
  let lo = 0, hi = idx.length;                    // first k with idx[k] >= i
  while (lo < hi){
    const mid = (lo + hi) >> 1;
    if (idx[mid] < i) lo = mid + 1; else hi = mid;
  }
  return lo;
}

function atOrBefore(idx, i){
  let lo = 0, hi = idx.length;                    // first k with idx[k] > i
  while (lo < hi){
    const mid = (lo + hi) >> 1;
    if (idx[mid] <= i) lo = mid + 1; else hi = mid;
  }
  return lo - 1;
}

function buildSeries(){
  chans = (meta.channels || []).map((c, k) =>
            newSeries('ch' + c, false, COLORS[k % COLORS.length]));
  gen   = newSeries('expected', true, '#1565c0');
  gen.scale = meta.gen_scale || 1;         // DAC code -> AD7606 counts (see applyMeta)
  received = 0; t0 = performance.now(); newest = null; frozen = null;
  events = [];
  flagTally = { judged: 0, dip: 0, max: 0, rise: 0 };
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
  /* .. plus the three things only the trace can show: a dip between two peaks, a max
     no sample of the window can reach, and an edge no dome has. */
  const dsw = document.createElement('span');
  dsw.className = 'sw'; dsw.style.background = EV_DIP;
  ep.appendChild(dsw);
  const msw = document.createElement('span');
  msw.className = 'sw'; msw.style.background = EV_SUPPORT;
  ep.appendChild(msw);
  const rsw = document.createElement('span');
  rsw.className = 'sw'; rsw.style.background = EV_RISE;
  ep.appendChild(rsw);
  /* .. and the floor they were all judged against, whenever the board announces one. */
  const mw = meta.min_width || 0;
  if (mw > 0){
    const wsw = document.createElement('span');
    wsw.className = 'sw'; wsw.style.background = EV_FLOOR;
    ep.appendChild(wsw);
  }
  ep.appendChild(document.createTextNode(
    ' dip between peaks / max above the trace / fast rise'
    /* The floor is a floor on the event's *duration*: the rule under the window is the
       duration it measured, green while that cleared the band - orange only when the floor
       is what refused the window (`rej WIDTH_TOO_NARROW`).  A window refused on the shape
       bounds instead (`rej SHORT_FOR_HEIGHT`) keeps its green rule, which is the whole
       point of colouring the rule by the width question alone - see drawEvents(). */
    + (mw > 0 ? ' / width floor ' + mw + 'smp (' + (mw * 1000
        / (meta.sample_rate || 10000)).toFixed(1)
        + 'ms) of duration: the rule under each window is green while its width cleared '
        + 'the band, orange only when the floor is what refused it' : '')));
  /* The overlay key is only meaningful while the overlay is on. */
  if (showEvents) lg.appendChild(ep);
}

function metaText(){
  const sr = meta.sample_rate || 0, d = meta.decim || 1;
  const mw = meta.min_width || 0;
  return 'source ' + meta.source
       + '   ' + (meta.streaming ? 'stream on' : 'stream off')
       + '   ' + (sr / 1000).toFixed(1) + ' kSPS, decim ' + d
       + '   window ' + viewSec.toFixed(2) + ' s (' + viewPoints() + ' pts, '
       + keepPoints() + ' kept = ' + keepSeconds().toFixed(1) + ' s)'
       + '   range +/-' + meta.range + ' V  (full scale +/-32768 counts)'
       + '   waveform gen ' + (meta.wf ? 'on' : 'off')
       /* The floor the board judged with, as the board's own samples and as time - the
          same number every window's floor band is drawn from (see drawEvents), so the
          page never has to be told twice.  It is a floor on the event's *duration*. */
       + (mw > 0 ? '   width floor ' + mw + 'smp/'
                   + (mw * 1000 / (sr || 10000)).toFixed(1) + 'ms of duration (the '
                   + 'coloured rule is orange only on a width rejection)' : '');
}

function applyMeta(m){
  const changed = (m.channels || []).length !== chans.length;
  meta = m;
  /* The gen column is a DAC code while the channel columns are AD7606 counts, so the
     expected trace only overlays the measured ones once the code is scaled - that is
     what gen_scale is (SIGNALGEN_ADC_PER_DAC, i.e. the strap --range names: 2.64
     counts per code at +/-10 V, 5.28 at +/-5 V).  Re-scale whatever is already held
     when the value changes, so one trace never mixes two scales. */
  if (gen && gen.vals.length){
    const was = gen.scale || 1, now = meta.gen_scale || 1;
    if (was !== now){
      for (let k = 0; k < gen.vals.length; k++) gen.vals[k] = gen.vals[k] * now / was;
    }
  }
  if (gen) gen.scale = meta.gen_scale || 1;
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
  const n = (m.adc && m.adc[0] ? m.adc[0].length : (m.gen ? m.gen.length : 0));
  const held = newestHeld();
  if (n && held !== null && i0 <= held) resetSeries();
  if (m.gen) {
    /* Scaled where it is added: the page draws counts, the stream carries DAC codes
       (see applyMeta). */
    const gs = meta.gen_scale || 1;
    for (let k = 0; k < m.gen.length; k++) addPoint(gen, i0 + k * d, m.gen[k] * gs);
  }
  (m.adc || []).forEach((vals, c) => {
    const s = chans[c];
    if (!s || !vals) return;
    for (let k = 0; k < vals.length; k++) addPoint(s, i0 + k * d, vals[k]);
  });
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

/* One point of one trace: the value AND the sample index it was taken at.  Nothing
   here looks at the clock: the index travels with the point, so a frame that never
   arrived cannot shift the ones after it. */
function addPoint(s, i, v){
  s.idx.push(i);
  s.vals.push(v);
}

/* The newest sample index any trace is holding - null while nothing has arrived. */
function newestHeld(){
  let last = null;
  [gen].concat(chans).forEach((s) => {
    if (s.idx.length && (last === null || s.idx[s.idx.length - 1] > last)){
      last = s.idx[s.idx.length - 1];
    }
  });
  return last;
}

/* Drop every held point because the stream went backwards.  That is what a browser
   that just reconnected is handed: StreamState.subscribe primes it with the whole
   buffer again, from the oldest sample it still keeps.  Appending that on top of the
   points already held would fold the trace back over itself and leave it - and now
   the detector windows with it - drawn at the wrong place, so the traces restart at
   the frame they are given.  The decisions are kept: they are absolute indices and
   still name the samples they were taken on. */
function resetSeries(){
  [gen].concat(chans).forEach((s) => { s.idx = []; s.vals = []; });
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
    s.idx.splice(0, extra);
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
       screen instead of a peak that has already scrolled out of it, and it does so in
       deliberate steps rather than on every frame (see AUTO_STEPS below). */
/* ---- the vertical scale ------------------------------------------------- */
/* Auto Y follows what is on screen, but deliberately.  A pane that re-fits to the
   window's min/max on every frame moves the whole trace whenever a tall peak or impulse
   enters or leaves the window (the bench capture spans x2-x3 between one window and the
   next), and one that zooms onto the noise of a quiet window crawls across the pane as
   the generator's DC wander moves - both look like the *signal* moving, which is the one
   thing this pane must not do.  So the scale is quantised, it grows the moment a sample
   does not fit, it steps down only once a smaller range has held for AUTO_SHRINK_MS, and
   it never goes below AUTO_MIN_SPAN: a window with no peak in it still shows the noise as
   noise instead of as a wall. */
const AUTO_STEPS     = [100, 200, 250, 500, 800, 1000, 2000, 2500, 5000, 10000, 20000,
                        25000, 50000];
const AUTO_MIN_SPAN  = 800;      // counts: ~2.5 % of full scale (+/-32768)
const AUTO_PAD       = 0.08;     // head/foot room, as a fraction of the range
const AUTO_SHRINK_MS = 1200;     // a smaller range has to hold this long to be adopted
function autoStep(need){
  for (let k = 0; k < AUTO_STEPS.length; k++){
    if (AUTO_STEPS[k] >= need) return AUTO_STEPS[k];
  }
  return AUTO_STEPS[AUTO_STEPS.length - 1];
}

function visible(){
  const sr = meta.sample_rate || 10000;
  const all = (showGen ? [gen] : []).concat(chans);
  let x1 = -Infinity;
  all.forEach((s) => {
    if (!s || !s.idx.length) return;
    const last = s.idx[s.idx.length - 1];
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
    frozen = { step: 2 * ADC_FULL_SCALE, mid: 0 };
  } else if (autoY || frozen === null){
    const need = Math.max(y1 - y0, AUTO_MIN_SPAN) * (1 + 2 * AUTO_PAD);
    const step = autoStep(need);
    const mid  = 0.5 * (y0 + y1);
    const now  = performance.now();
    if (frozen === null || frozen.step === undefined){
      frozen = { step: step, mid: mid, at: now };
    } else if (step > frozen.step){
      // A sample does not fit: grow to the step that holds it, and follow its centre.
      frozen.step = step; frozen.mid = mid; frozen.at = now;
    } else if (now - frozen.at >= AUTO_SHRINK_MS){
      /* It has fitted for a while: close in, at most halving each time so the pane
         settles instead of snapping, and follow the window's centre if the signal has
         drifted away from the pane's. */
      const want = autoStep(Math.max(need, 0.5 * frozen.step));
      frozen.step = (want < step) ? step : want;
      if (Math.abs(mid - frozen.mid) > 0.25 * frozen.step){ frozen.mid = mid; }
      frozen.at = now;
    }
    y0 = frozen.mid - 0.5 * frozen.step;
    y1 = frozen.mid + 0.5 * frozen.step;
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
  const sr = meta.sample_rate || 10000;
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
    if (!s || s.idx.length < 2) return;
    /* Only the points inside the window: the buffer holds keepPoints() of them, so
       drawing all of it would draw (and join) points that are not shown.  Bounded by
       index, not by a stride: a frame the page never got shows up as the gap it is
       rather than pulling the rest of the trace left. */
    const k0 = atOrAfter(s.idx, view.x0), k1 = atOrBefore(s.idx, view.x1);
    if (k1 - k0 < 1) return;
    g.strokeStyle = s.color;
    g.lineWidth = s.dashed ? 1.5 : 1.2;
    g.setLineDash(s.dashed ? [6, 4] : []);
    g.beginPath();
    for (let k = k0; k <= k1; k++){
      const x = X(s.idx[k]), y = Y(s.vals[k]);
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
  const out = [];
  if (s === null || !s.idx.length) return out;
  const k0 = atOrAfter(s.idx, i0), k1 = atOrBefore(s.idx, i1);
  for (let k = k0; k <= k1; k++) out.push({ i: s.idx[k], v: s.vals[k] });
  return out;
}

/* What the trace says about one window.  Cached on the event: the answer changes
   only while the window's own samples are still arriving. */
function judge(ev){
  if (ev.q && (ev.qdone || ev.qcov === newest)) return ev.q;
  const d = meta.decim || 1;
  const q = { ch: null, lobes: 0, dip: null, dipIdx: null, tops: [],
              support: null, why: null, apexIdx: null, apexV: null,
              rise: null, riseIdx: null, riseTop: null };
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
    /* Fast or slow is the other half of "is this a bead".  Measure the climb to the
       trace's own top: walk back from the apex to the last plotted point still within
       EV_RISE_FRAC of the baseline and see how much of the board's clock that is.  A
       decimated stream cannot resolve anything finer than one plotted interval, so the
       measurement is a ceiling: when it lands at or under EV_RISE_MS the true edge is
       at most that, which is why the note says "<=" and not "=".  Only for a window
       whose max the trace does carry - a max the trace misses says more, and a lone
       apex is a fast edge by construction. */
    if (span >= EV_MAX_PTS && ev.max > 0 && q.support === null && top > ev.base){
      const low = ev.base + EV_RISE_FRAC * (top - ev.base);
      let j = kTop;
      while (j > 0 && pts[j - 1].v > low) j--;
      const riseSmp = (kTop - j) * d;
      const limit = Math.round(EV_RISE_MS * (meta.sample_rate || 10000) / 1000);
      if (j > 0 && riseSmp > 0 && riseSmp <= limit){    // j==0: the window opens mid-climb
        q.rise = riseSmp; q.riseIdx = pts[j].i; q.riseTop = top;
      }
    }
  }
  if (newest !== null && newest > ev.end){         // the whole window is in hand
    ev.qdone = true;
    if (q.ch !== null){                            // tally it once, for the stats line
      flagTally.judged++;
      if (q.lobes >= EV_LOBES) flagTally.dip++;
      if (q.support !== null) flagTally.max++;
      if (q.rise !== null) flagTally.rise++;
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

   Three things the lines cannot show are read off the trace and drawn in their own
   colours: a dip between two peaks inside one window (EV_DIP), a max the window's own
   samples do not support (EV_SUPPORT), and an edge the trace climbs faster than a bead
   can (EV_RISE).  See judge().

   The width floor the board announced rides on every window as a band of its own
   (EV_FLOOR, `minw` samples of *duration* centred on the window), because that is the
   number the firmware refused or accepted the window against: a window whose width is
   under the band is one it calls too narrow.  What that window's width *was* - the
   duration the detector estimated for it, which is the number the tag shows and the band
   is compared with - is drawn as a rule coloured by the width question alone (see wCol):
   green while it cleared the band, orange only when the floor is what refused the window,
   so a window refused on the shape bounds cannot read as a width rejection. */
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

    /* The width floor the board announced, as a BAND centred on the window.  The number
       is a floor on the event's *duration* (`minw` samples of it), and the duration the
       detector estimates is centred on the excursion it measured - so a window whose own
       width comes back under the band is one it refused as too narrow, which is what its
       w= tag says.  The estimate itself is drawn just below (the duration rule), which is
       what makes the pair readable: the band is what was needed, the rule what was
       measured. */
    const mw  = meta.min_width || 0;
    const mid = 0.5 * (ev.start + ev.end);
    if (mw > 0){
      g.strokeStyle = EV_FLOOR; g.lineWidth = 1; g.setLineDash([1, 4]);
      [mid - 0.5 * mw, mid + 0.5 * mw].forEach((i) => {
        const x = Math.round(X(i)) + 0.5;
        g.beginPath(); g.moveTo(x, top); g.lineTo(x, bot); g.stroke();
      });
      g.setLineDash([]);
    }

    /* The width question has its own answer, and it is NOT the event's verdict: the floor
       only ever refused a window the firmware reported as WIDTH_TOO_NARROW (or, at the
       other end, WIDTH_TOO_WIDE).  An event refused on the *shape* bounds - the ones a
       needle trips, `rej SHORT_FOR_HEIGHT` - can carry a duration well over the band, and
       painting its rule in the rejection colour is what makes a rise/slope rejection read
       as a width one (and the tag beside it as a contradiction).  So the rule is coloured
       by the width test alone: green while the duration is at or over the floor, orange
       when the floor is what refused the window.  The tag spells the comparison out. */
    const wRej = (ev.reason === 2 || ev.reason === 3);          // the width reasons
    const wCol = (wRej || (mw > 0 && ev.width_smp > 0 && ev.width_smp < mw))
               ? EV_BAD : EV_OK;

    const yb = Math.round(Y(ev.base)) + 0.5;
    const ym = Math.round(Y(ev.base + ev.max)) + 0.5;
    g.strokeStyle = EV_BASE; g.setLineDash([4, 3]);         // frozen baseline
    g.beginPath(); g.moveTo(e0, yb); g.lineTo(e1, yb); g.stroke();
    g.strokeStyle = col; g.setLineDash([6, 3]);             // the peak it reported
    g.beginPath(); g.moveTo(e0, ym); g.lineTo(e1, ym); g.stroke();
    g.setLineDash([]);

    /* The width the bounds were judged with - the event's duration, and the number the
       floor band above is drawn from - as a rule along the baseline, in the *width*'s own
       colour (wCol: green when this duration cleared the band, orange when the floor is
       what refused the window - the event's verdict may have come from another bound
       entirely).  The shaded window is the span the *measurement* covered; where the two
       differ, the duration is the longer (a faint pulse is cut off high up its flanks).
       See EventDetector::envelopeDuration and the `w=` / `(exc ..)` tag.

       It is drawn on the same time scale as everything else (X()), centred on the window,
       so its *length* is the estimate and its position is simply where the event is - the
       same reference the floor band above is measured against. */
    if (ev.width_smp > 0){
      const xa = Math.round(X(mid - 0.5 * ev.width_smp)) + 0.5;
      const xb = Math.round(X(mid + 0.5 * ev.width_smp)) + 0.5;
      g.strokeStyle = wCol; g.lineWidth = 2; g.setLineDash([2, 2]);
      g.beginPath(); g.moveTo(xa, yb); g.lineTo(xb, yb); g.stroke();
      g.lineWidth = 1; g.setLineDash([]);
    }

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

    if (q.rise !== null){
      /* The edge itself, where the trace made it: from the point the climb started to
         the top it reached.  A slope this steep is the shape of a spike, not a dome. */
      const xr = Math.round(X(q.riseIdx)) + 0.5;
      const ya = Math.round(Y(ev.base)) + 0.5;
      const xt = Math.round(X(q.riseIdx + q.rise)) + 0.5;
      const yt = Math.round(Y(q.riseTop)) + 0.5;
      g.strokeStyle = EV_RISE; g.lineWidth = 2;
      g.beginPath(); g.moveTo(xr, ya); g.lineTo(xt, yt); g.stroke();
      g.lineWidth = 1;
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
    if (q.rise !== null){
      /* A ceiling, not a measurement: the stream's own resolution is the floor of what
         can be read off the trace, so the note says "at most". */
      notes.push({ col: EV_RISE, text: '  !rise <=' + Math.round(q.rise * 1e6
                    / (meta.sample_rate || 10000)) + 'us' });
    }

    tags.push({ x: x0 + 3, col: col, notes: notes,
                text: '#' + ev.index + ' ch' + ev.channel + ' '
                      + (ev.valid ? 'VALID' : 'rej ' + ev.reason_name)
                      + '  w=' + ev.width_smp + 'smp  max=' + ev.max
                      + '  base=' + ev.base
                      /* `w=` is the event's *duration* - the width the bounds were judged
                         with and the one the floor band is drawn from - while the window
                         itself is the excursion the measurement covered.  The two differ
                         whenever the pulse is faint enough for the envelope to widen it,
                         so the measured span is printed beside it. */
                      + (ev.width_smp === (ev.end - ev.start) ? ''
                         : '  (exc ' + (ev.end - ev.start) + 'smp)')
                      /* On a rejected window the floor comparison is spelled out, because
                         `rej <reason>` next to a `w=` over the floor is exactly the reading
                         that goes wrong: the reason word names the bound that refused it,
                         this says whether the width floor was one of them. */
                      + (!ev.valid && mw > 0 && ev.width_smp > 0
                         ? (ev.width_smp >= mw ? '  >= floor ' : '  < floor ')
                           + mw + 'smp' : '') });
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
   the trace cannot prove an impulse did it, but it is where to look.  "fast rise" is a
   peak the trace climbs to inside EV_RISE_MS, which a bead's dome never does. */
function flagsText(){
  if (!flagTally.judged) return '';
  return '   read back ' + flagTally.judged + ' windows: '
       + flagTally.dip + ' two-peak, ' + flagTally.max + ' max?, '
       + flagTally.rise + ' fast rise';
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
               '  py -3 test/live_plot.py --sim --sim-ladder     # the floor, drawn\n'
               '  py -3 test/live_plot.py --sim --sim-cycles 6 --window 2 '
               '--http-port 9000\n'
               '  py -3 test/live_plot.py --sim --sim-speed 0 --no-open\n'
               '  py -3 test/live_plot.py --stdin < bench.txt\n'
               '  test\\test_bench_detect.exe --stream | py -3 test/live_plot.py '
               '--stdin\n'
               '  py -3 test/live_plot.py --port COM7')

    src = p.add_argument_group('source')
    src.add_argument('--port', help='serial port of the M7 UART, e.g. COM7')
    src.add_argument('--stdin', action='store_true',
                     help='read the board text from stdin: a capture file '
                          '("--stdin < bench.txt") or a pipe '
                          '(test_bench_detect.exe --stream | ... --stdin)')
    src.add_argument('--sim', action='store_true',
                     help='no board: run test_bench_detect.exe --stream and draw the '
                          'firmware\'s own decisions')
    src.add_argument('--firmware', metavar='EXE', default=None,
                     help='bench harness to run for --sim (default: '
                          'test/test_bench_detect.exe next to this script)')
    src.add_argument('--sim-cycles', type=int, default=4,
                     help='pattern cycles the harness generates (default %(default)s, '
                          '1 cycle = 2 s of stream)')
    src.add_argument('--sim-start', choices=('hold', 'live'), default=None,
                     help='the harness\'s quiet period: hold (default) = noise only, '
                          'live = the pulses run from sample 0, the board\'s boot '
                          'reality')
    src.add_argument('--sim-ladder', action='store_true',
                     help='run the harness\'s floor ladder (SIGNALGEN_LADDER_* in '
                          'config.h) instead of the random train: a staircase of '
                          'isolated pulses whose apexes step up from below the '
                          'detector\'s floor, so the pane\'s own scale settles on them '
                          'and "how close to the baseline can a peak be and still be '
                          'detected" can be *seen*.  The harness\'s own --ladder-* '
                          'flags move the staircase: pipe its --stream here for a '
                          'different width, min, step or count')
    src.add_argument('--sim-frontend', type=int, default=None, metavar='TAPS',
                     help='analog front end the harness models: 1 = none, 3 = a real '
                          'board (default)')
    src.add_argument('--hw', action='store_true',
                     help='harness runs two slightly mismatched modules instead of '
                          'an ideal pair')
    src.add_argument('--sim-repeat', action='store_true',
                     help='replay the harness capture over and over, continuing the '
                          'sample indices (the board\'s timeline) between passes')
    src.add_argument('--sim-speed', type=float, default=1.0,
                     help='simulated time scale: 2 = twice as fast, '
                          '0 = as fast as Python can (default 1)')
    src.add_argument('--sim-seed', type=lambda s: int(s, 0), default=None,
                     help='override the harness waveform seed')
    src.add_argument('--no-stream-command', action='store_true',
                     help='never send "T1"/"T0" (the board is already streaming)')

    view = p.add_argument_group('view')
    view.add_argument('--channels', type=int, default=CHANNELS,
                      help='simulated and plotted channel count (default %(default)s; '
                           'the firmware path streams whatever SORTER_STREAM_CHANNELS '
                           'selects)')
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

    if not args.sim and not args.port and not args.stdin:
        p.error('give --port COMx for the board, --sim for the bench harness, or '
                '--stdin for a capture/pipe')

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
        source = FirmwareSource(state, args.range_v, speed=args.sim_speed,
                                seed=args.sim_seed, exe=args.firmware,
                                cycles=args.sim_cycles, frontend=args.sim_frontend,
                                start=args.sim_start, hw=args.hw,
                                repeat=args.sim_repeat, ladder=args.sim_ladder)
    elif args.stdin:
        source = PipeSource(state, args.range_v)
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
    what = ('the bench harness, its own verdicts' if args.sim
            else 'stdin (board text)' if args.stdin else args.port)
    print('live_plot: source = %s' % what)
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
        print('live_plot: could not start %s: %s' % (what, exc), file=sys.stderr)
        if args.sim:
            print('live_plot: test/run_tests.ps1 builds the harness the same way it '
                  'needs to be run (its MinGW C++ runtime on PATH).', file=sys.stderr)
        elif args.stdin:
            print('live_plot: the pipeline is closed - feed it "test_bench_detect.exe '
                  '--stream" or a capture file.', file=sys.stderr)
        else:
            print('live_plot: is another monitor holding the port?', file=sys.stderr)
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

