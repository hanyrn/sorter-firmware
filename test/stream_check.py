#!/usr/bin/env python3
"""
Contract check for the harness -> viewer text protocol.

test/test_bench_detect.cpp --stream is the one place the *shipped*
EventAggregator and EventDetector run without a board, and test/live_plot.py draws
whatever its parser understood of the text that comes back.  The two only meet on
the shape of those lines, and no C++ test notices when that shape changes: the
viewer just stops drawing the overlay the line carried.  That is how the old Python
mirror of the detector drifted away from the firmware it was supposed to show -
silently, one decision at a time - which is why the mirror is gone and the harness
prints the board's own text instead.

This script closes the loop.  It runs the harness, feeds its stdout through the
viewer's own StreamParser and StreamState, and asserts the pair still agrees on

  * the header - the decimation, the channel list, the points per frame;
  * the samples - one point per printed `S,` line, the indices on the decimation
    grid, the frame count the viewer stitches out of them and the last index the
    run should reach;
  * the events - one overlay per printed `Event #` group, carrying the window, the
    width, the baseline, the peak and the verdict words the firmware printed rather
    than anything this side worked out;
  * the harness's verdict on stderr - the same event and VALID counts the viewer
    ended up holding, so the line in the log pane describes what is on the page.

Reword a line on either side and this fails; that is the contract, and the message
says which line and which number stopped adding up.

Run:  py -3 test/stream_check.py [--cycles N] [--decim N] [--exe PATH] [--range V]
Needs the harness:  powershell -ExecutionPolicy Bypass -File test/run_tests.ps1
(test/run_tests.ps1 runs this too, as soon as the harness it built passes.)
"""

import argparse
import os
import queue
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live_plot                                  # noqa: E402  - the pair under test

EXE  = live_plot.default_bench_exe()      # the same harness --sim would run

# The harness's verdict, word for word (test_bench_detect.cpp's --stream block).
# Pinned like the line shapes the viewer pins, because this is the text the log pane
# puts next to the peaks it is talking about.
VERDICT = re.compile(r'^stream: (\d+) cycle\(s\), (\d+) sample\(s\), (\d+) event\(s\) '
                     r'\((\d+) VALID\), (\d+) missed, (\d+) ghost, '
                     r'(\d+) peak rejected, (\d+) spike-set max=, (\d+) split, '
                     r'(\d+) fused\s+\[test/run_tests\.ps1 owns the checks\]')

# The harness's stream-on header, word for word - the line the harness prints for the
# board's `STREAM on:` metadata, and the one the page takes its channel list, its frame
# size, its decimation and its width floor from.
HEADER = re.compile(r'^STREAM on: decim=(\d+) chans=([0-9,]+) wf=(\d+) '
                    r'points=(\d+) minw=(\d+)\s*$')


class Recorder(live_plot.StreamState):
    """The viewer's own state object, plus the lines that reached the log pane.

    Nothing subscribes while the text is fed, so publishing is dropped - the check
    only needs the state the parser left behind, and a queue would otherwise sit on
    one message per point.
    """

    def __init__(self, *args, **kwargs):
        live_plot.StreamState.__init__(self, *args, **kwargs)
        self.lines = []

    def log(self, line):
        if line:
            self.lines.append(line)
        live_plot.StreamState.log(self, line)

    def publish(self, obj):
        pass


class Report:
    """Counts what held, names what did not."""

    def __init__(self):
        self.passed   = 0
        self.failures = []

    def ok(self, cond, what, detail=''):
        if cond:
            self.passed += 1
        else:
            self.failures.append(what + (('   [%s]' % detail) if detail else ''))
        return bool(cond)


def run_harness(exe, cycles, decim, ladder=False):
    """Run the harness and return (argv, stdout lines, stderr lines).

    `ladder` adds --ladder: the floor ladder instead of the random train, whose table
    the harness prints on *stderr* precisely so that stdout stays the board's own serial
    text - which is what this file exists to pin down (see check_ladder).
    """
    argv = [exe, '--stream', '--cycles', str(cycles), '--stream-decim', str(decim)]
    if ladder:
        argv.append('--ladder')
    env  = dict(os.environ)
    extra = [d for d in live_plot.bench_runtime_dirs() if d not in env.get('PATH', '')]
    if extra:
        # The same PATH fixup --sim does: the MinGW runtime DLLs may not be on PATH.
        env['PATH'] = os.pathsep.join(extra + [env.get('PATH', '')])
    proc = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          universal_newlines=True, env=env)
    if proc.returncode == live_plot.BENCH_EXIT_DLL:
        raise SystemExit('the harness could not load its C++ runtime (0xC0000135): put '
                         "the compiler's bin folder on PATH")
    if proc.returncode != 0:
        raise SystemExit('the harness exited %d' % proc.returncode)
    return argv, proc.stdout.splitlines(), proc.stderr.splitlines()


def feed(lines, range_v):
    """The viewer, fed the harness's text exactly the way --sim and --stdin feed it.

    A short history is enough here (the check reads the state the parser left
    behind, not a page): one second of points, primed the way a new tab is.
    """
    state  = Recorder(window=live_plot.view_points(1.0), window_sec=live_plot.VIEW_SEC,
                      keep_sec=1.0)
    parser = live_plot.StreamParser(state, 'bench', range_v, echo=False)
    for line in lines:
        parser.feed_line(line)
    parser.flush()
    return state, parser


def check_header(rep, meta, point_cols, raw_head):
    rep.ok(bool(meta), 'the STREAM header never reached the viewer (no meta line)')
    rep.ok(meta.get('streaming') is True,
           'the viewer did not read the header as a stream-on line', str(meta.get('source')))
    rep.ok(meta.get('points') == live_plot.POINTS,
           "the header's points-per-frame did not reach the page",
           'header %r, viewer %r' % (meta.get('points'), live_plot.POINTS))
    rep.ok(meta.get('channels') == point_cols,
           'the channels in the header are not the channels on the points',
           'header %r, points %r' % (meta.get('channels'), point_cols))
    rep.ok(meta.get('sample_rate') == live_plot.SAMPLE_RATE_HZ,
           "the page's sample rate is not the board's", str(meta.get('sample_rate')))
    # The stream's gen column is a DAC code while the channels are AD7606 counts, so
    # the page needs the counts-per-code scale to draw the expected trace on top of the
    # traces it is meant to overlay: without it, the code goes up as a count and the
    # expected waveform sits ~3.4 kcounts low, under every window's base and max lines.
    rep.ok(meta.get('gen_scale') == live_plot.gen_scale_counts(meta.get('range', 0.0)),
           'the counts-per-DAC-code scale did not reach the page',
           'page %r for range %r' % (meta.get('gen_scale'), meta.get('range')))

    # The same line again, read by this file's own regex: a reworded header has to be
    # noticed on both sides, and every field it carries has to land in the page's meta.
    m = HEADER.match(raw_head)
    if not rep.ok(m is not None,
                  "the harness's STREAM line is no longer the one the page expects",
                  raw_head):
        return
    rep.ok(int(m.group(1)) == meta.get('decim'),
           'the decimation in the header did not reach the page',
           '%s vs %r' % (m.group(1), meta.get('decim')))
    rep.ok([int(c) for c in m.group(2).split(',')] == meta.get('channels'),
           'the channel list in the header did not reach the page',
           '%s vs %r' % (m.group(2), meta.get('channels')))
    rep.ok(int(m.group(3)) == meta.get('wf'),
           'the waveform selector in the header did not reach the page',
           '%s vs %r' % (m.group(3), meta.get('wf')))
    rep.ok(int(m.group(4)) == meta.get('points'),
           'the points-per-frame in the header did not reach the page',
           '%s vs %r' % (m.group(4), meta.get('points')))
    # The floor the page draws on every window has to be the floor the detector judged
    # with, or a tick right of a window's edge and a WIDTH_TOO_NARROW verdict could
    # contradict each other on screen.
    rep.ok(int(m.group(5)) == meta.get('min_width'),
           'the width floor in the header did not reach the page',
           '%s vs %r' % (m.group(5), meta.get('min_width')))


def check_samples(rep, state, decim, cycles, idxs, meta):
    rep.ok(idxs[0] == 0, 'the stream does not start at sample 0', str(idxs[0]))
    bad = [(a, b) for a, b in zip(idxs, idxs[1:]) if b - a != decim]
    rep.ok(not bad, 'the printed indices are not on the %d-sample grid' % decim,
           'first gap %r' % (bad[0] if bad else None))
    step = (idxs[1] - idxs[0]) if len(idxs) > 1 else decim
    rep.ok(meta.get('decim') == step,
           'the header announces a decimation the printed indices are not on',
           'header %r, grid %r' % (meta.get('decim'), step))
    want_last = ((cycles * live_plot.BENCH_CYCLE_SAMPLES) // decim - 1) * decim
    rep.ok(idxs[-1] == want_last,
           'the run stops at index %d; %d cycle(s) of %d samples at decim %d reach %d'
           % (idxs[-1], cycles, live_plot.BENCH_CYCLE_SAMPLES, decim, want_last))
    rep.ok(state.points == len(idxs),
           'the viewer holds %d of the %d printed points' % (state.points, len(idxs)))
    per_frame = int(meta.get('points', live_plot.POINTS))
    want_frames = (len(idxs) + per_frame - 1) // per_frame
    rep.ok(state.frames == want_frames,
           'the viewer stitched %d frames out of the points; they make %d'
           % (state.frames, want_frames))
    rep.ok(all(state.recent[k][0] == state.recent[0][0] + k * decim
               for k in range(len(state.recent))),
           'the sample indices the page is primed with are not one decimation apart')


def check_events(rep, state, parser, groups, idxs, meta):
    """One overlay per printed event group, carrying the numbers that were printed."""
    evs = {e['index']: e for e in state.events}
    rep.ok(len(groups) == len(state.events),
           'the harness printed %d event group(s), the viewer kept %d overlay(s)'
           % (len(groups), len(state.events)))
    rep.ok(sum(1 for h, _ in groups if 'VALID' in h) == sum(e['valid'] for e in state.events),
           'the VALID events the viewer holds are not the ones the harness printed')

    channels = meta.get('channels') or []
    for head, chans in groups:
        m = live_plot.EVENT_HEAD.match(head)
        if not rep.ok(m is not None,
                      "the viewer's event regex no longer matches the harness's head line",
                      head):
            continue
        index = int(m.group(1))
        ev = evs.get(index)
        if not rep.ok(ev is not None, 'nothing was drawn for the printed event #%d' % index):
            continue

        rep.ok(ev['valid'] == (1 if m.group(2) == 'VALID' else 0),
               'event #%d: the verdict did not reach the overlay' % index)
        rep.ok(ev['reason'] == int(m.group(3)),
               'event #%d: reason number %s did not reach the overlay' % (index, m.group(3)))
        rep.ok(ev['mode'] == m.group(9),
               'event #%d: the drawing mode did not reach the overlay' % index,
               '%s vs %s' % (ev['mode'], m.group(9)))
        rep.ok(ev.get('channel') == int(m.group(8)),
               'event #%d: the tag is pinned to ch%s, the firmware named ch%s as primary'
               % (index, ev.get('channel'), m.group(8)))
        rep.ok(ev.get('channel') in channels,
               'event #%d is pinned to ch%s, which is not one of the streamed channels'
               % (index, ev.get('channel')))
        rep.ok(ev['start'] == parser.us_to_sample(int(m.group(5)))
               and ev['end'] == parser.us_to_sample(int(m.group(6))),
               'event #%d: the window did not convert onto the sample grid' % index)
        rep.ok(ev['end'] - ev['start'] == int(m.group(7)),
               'event #%d: the window is %d samples wide, the head line says %s'
               % (index, ev['end'] - ev['start'], m.group(7)))
        rep.ok(0 <= ev['start'] and ev['end'] <= idxs[-1],
               'event #%d falls outside the samples the stream carried' % index)

        # The primary channel's line is the one whose numbers the tag shows.
        prim = None
        for line in chans:
            c = live_plot.EVENT_CHANNEL.match(line)
            if c is not None and int(c.group(1)) == ev['channel']:
                prim = c
        if not rep.ok(prim is not None,
                      'event #%d has no ch line for its primary channel' % index):
            continue
        rep.ok(ev.get('max') == int(prim.group(2)) and ev.get('area') == int(prim.group(3))
               and ev.get('base') == int(prim.group(6))
               and ev.get('width_smp') == int(prim.group(5)),
               'event #%d: the overlay carries different numbers than the ch line' % index,
               prim.group(0))
        rep.ok(ev.get('verdict') == prim.group(7),
               'event #%d: the channel verdict was misread' % index)
        # The head line names the event, the channel line names that channel's own
        # outcome.  The viewer keeps the channel's - it is what a rejection is tagged
        # with - so both are legitimately "the word the firmware printed" here.
        rep.ok(ev.get('reason_name') in (m.group(4), prim.group(8) or prim.group(7)),
               'event #%d: the reason name in the overlay (%r) is not one the firmware '
               'printed' % (index, ev.get('reason_name')))
        if ev['valid']:
            rep.ok(ev.get('max', 0) > 0 and ev.get('area', 0) > 0
                   and ev.get('width_smp', 0) > 0,
                   'event #%d is VALID but has no peak for the tag to show' % index,
                   'max=%s area=%s w=%s' % (ev.get('max'), ev.get('area'),
                                            ev.get('width_smp')))


def check_log(rep, state, groups):
    rep.ok(any(l.startswith('STREAM ') for l in state.lines),
           'the STREAM header never reached the log pane')
    missing = [h for h, _ in groups if h not in state.lines]
    rep.ok(not missing, '%d event line(s) never reached the log pane' % len(missing),
           missing[0] if missing else '')
    rep.ok(not any(l.startswith('S,') for l in state.lines),
           'the sample lines are being echoed into the log pane')


def check_verdict(rep, stderr, cycles, state):
    line = next((l.strip() for l in stderr if l.startswith('stream: ')), None)
    if not rep.ok(line is not None,
                  'the harness printed no verdict on stderr - the log pane takes its '
                  'counters from there',
                  ' | '.join(l.strip() for l in stderr if l.strip())[:200]):
        return
    m = VERDICT.match(line)
    if not rep.ok(m is not None,
                  "the harness's verdict line is no longer the one this check (and the "
                  'log pane) expects', line):
        return
    rep.ok(int(m.group(1)) == cycles
           and int(m.group(2)) == cycles * live_plot.BENCH_CYCLE_SAMPLES,
           'the verdict describes a different run than the one that was drawn', line)
    rep.ok(int(m.group(3)) == len(state.events),
           'the harness counted %s event(s), the viewer drew %d'
           % (m.group(3), len(state.events)))
    rep.ok(int(m.group(4)) == sum(e['valid'] for e in state.events),
           'the harness counted %s VALID, the viewer drew %d'
           % (m.group(4), sum(e['valid'] for e in state.events)))


def check_ladder(rep, exe, range_v, cycles, decim):
    """The same contract for `--ladder --stream` - the run a viewer draws the floor with.

    The ladder's *table* is a report, and a report on stdout would be text the viewer
    cannot classify: it lands in the log pane as noise, which is why the harness prints
    it on stderr (printLadder in test_bench_detect.cpp).  This is the check that keeps
    that true: every line the parser logs has to be a line the format defines - the
    STREAM header or an event head - and every point has to arrive.
    """
    _argv, stdout, stderr = run_harness(exe, cycles, decim, ladder=True)
    s_lines = [l for l in stdout if l.startswith('S,')]
    if not rep.ok(bool(s_lines), 'the ladder run printed no sample lines at all'):
        return
    state, parser = feed(stdout, range_v)
    heads = [l.strip() for l in stdout if l.strip().startswith('Event #')]

    rep.ok(state.points == len(s_lines),
           'the viewer holds %d of the ladder\'s %d printed points'
           % (state.points, len(s_lines)))
    rep.ok(len(state.events) == len(heads),
           'the ladder printed %d event(s); the viewer drew %d'
           % (len(heads), len(state.events)))
    rep.ok(state.events and all(e['valid'] for e in state.events),
           'the ladder run drew an event that is not VALID - it draws nothing but '
           'isolated raised cosines over the baseline')
    # The one that matters: nothing on stdout is text the parser had to guess at.  The
    # log pane legitimately holds the header and the two lines of every event (that is
    # what a reader wants there) - anything *else* is a report that leaked into the
    # stream, and it is exactly what a table printed with std::printf would look like.
    stray = [l for l in state.lines if not (l.startswith('STREAM ')
                                            or l.startswith('Event #')
                                            or l.startswith('ch'))]
    rep.ok(not stray,
           '%d line(s) of the ladder run reached the log pane as unrecognised text '
           '(the table belongs on stderr)' % len(stray),
           stray[0] if stray else '')
    rep.ok(any('floor ladder:' in l for l in stderr),
           'the ladder table did not reach stderr: the stream then carries a report')
    print('ladder:  %d point(s), %d event(s) - the table on stderr, nothing else on '
          'stdout' % (len(s_lines), len(heads)))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Check that the bench harness --stream text and the live viewer '
                    'still agree.')
    ap.add_argument('--cycles', type=int, default=4,
                    help='pattern cycles to run (default %(default)s; 1 cycle = 2 s of '
                         'stream)')
    ap.add_argument('--decim', type=int, default=live_plot.DECIM,
                    help='stream decimation the harness prints (default %(default)s, '
                         'SORTER_STREAM_DECIM)')
    ap.add_argument('--exe', default=EXE, help='bench harness (default %(default)s)')
    ap.add_argument('--range', dest='range_v', type=float, default=10.0,
                    help="AD7606 full scale in volts, for the page's labels "
                         '(default %(default)s)')
    args = ap.parse_args(argv)
    cycles = max(int(args.cycles), 2)          # the harness clamps to >= 2 cycles
    decim  = max(int(args.decim), 1)

    if not os.path.exists(args.exe):
        print('%s is missing - build the harness first:\n'
              '  powershell -ExecutionPolicy Bypass -File test\\run_tests.ps1'
              % args.exe)
        return 1

    argv_h, stdout, stderr = run_harness(args.exe, cycles, decim)
    s_lines = [l for l in stdout if l.startswith('S,')]
    if not s_lines:
        print('the harness printed no single sample line - is %s built from a source '
              'tree that has --stream?' % args.exe)
        return 1

    state, parser = feed(stdout, args.range_v)
    meta = state.meta or {}
    idxs = [int(l.split(',', 2)[1]) for l in s_lines]
    point_cols = sorted(int(tok.partition(':')[0]) for tok in s_lines[0].split(',')[3:])
    groups = []                                # [(head line, [ch line, ...]), ...]
    for raw in stdout:
        line = raw.strip()                     # the parser strips too: match what it saw
        if line.startswith('Event #'):
            groups.append((line, []))
        elif line.startswith('ch') and groups:
            groups[-1][1].append(line)
    ch_lines = [l.strip() for l in stdout if l.strip().startswith('ch')]
    raw_head = next((l.strip() for l in stdout if l.startswith('STREAM ')), '')

    rep = Report()
    check_header(rep, meta, point_cols, raw_head)
    check_samples(rep, state, decim, cycles, idxs, meta)
    check_events(rep, state, parser, groups, idxs, meta)
    check_log(rep, state, groups)
    check_verdict(rep, stderr, cycles, state)
    # ... and the one other run this viewer has a flag for (--sim-ladder): same lines,
    # same parser, and a report that has to stay off the stream.
    check_ladder(rep, args.exe, args.range_v, cycles, decim)

    # What a browser gets when it connects after the run: the metadata, the newest
    # second of trace and every decision.  That is also how a page reload recovers.
    q = state.subscribe()
    primed = []
    while True:
        try:
            primed.append(q.get_nowait())
        except queue.Empty:
            break
    kinds = set(o.get('kind') for o in primed)
    rep.ok({'meta', 'data', 'events'} <= kinds,
           'a tab connecting after the run is not primed with meta + trace + events',
           sorted(kinds))
    data = next((o for o in primed if o.get('kind') == 'data'), None)
    if data is not None:
        want_n = min(len(idxs), state.window)
        rep.ok(data['n'] == want_n and data['i'] == idxs[-want_n],
               'the trace a new tab is primed with does not end on the newest point',
               str(data['n']))
        rep.ok(len(data['adc']) == len(meta.get('channels') or []),
               'the priming trace has %d column(s); the header announced %d'
               % (len(data['adc']), len(meta.get('channels') or [])))

    print('harness: %s' % ' '.join(argv_h[1:]))
    print('stream:  %d point(s) in %d frame(s), decim %d, channel(s) %s'
          % (len(idxs), state.frames, decim, ','.join(str(c) for c in point_cols)))
    print('events:  %d group(s) (%d VALID), %d channel line(s), %d overlay(s) kept'
          % (len(groups), sum(1 for h, _ in groups if 'VALID' in h), len(ch_lines),
             len(state.events)))
    if rep.failures:
        print('\nFAIL: the harness and the viewer no longer agree on the protocol -')
        for f in rep.failures:
            print('  -', f)
        return 1
    print('\nPASS: %d check(s) - every printed line is a line the viewer understood, and'
          % rep.passed)
    print('      the numbers in the overlays are the numbers the firmware printed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
