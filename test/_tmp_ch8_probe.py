"""Throwaway probe: read a few seconds of the board's stream and summarise it.

What it answers (nothing is assumed about which channel is wired where):

  * the `STREAM on:` header, i.e. the channel columns the board says it prints,
  * the `S,` line verbatim (column layout) and how each column moves,
  * every non-`S,` line (event groups), so a missing / railed / absent channel
    can be told from a channel whose *samples* are constant.

The port is opened exactly the way live_plot.py does (raw COMx + SetCommState
+ DTR) and closed the same way too: "T0", then wait for the board to stop
transmitting, then hang up - a GIGA's USB CDC parks the M7 loop if the host
stops reading mid-transfer.

    py -3 test/_tmp_ch8_probe.py COM6 6
"""

import collections
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live_plot                                    # noqa: E402  (path set above)


def read_lines(port, seconds, out_path=None):
    name = port if port.startswith('\\\\') else '\\\\.\\' + port
    fh = open(name, 'r+b', buffering=0)
    live_plot.configure_serial_port(fh)
    fh.write(b'T1\n')                               # idempotent: stream on

    lines, buf, t0 = [], b'', time.time()
    while time.time() - t0 < seconds:
        data = fh.read(4096)
        if not data:
            time.sleep(0.002)
            continue
        buf += data
        while b'\n' in buf:
            line, _, buf = buf.partition(b'\n')
            lines.append(line.decode('utf-8', 'replace').rstrip('\r'))

    fh.write(b'T0\n')                               # stop, then drain to quiet
    last = time.time()
    while time.time() - last < 2.0:
        data = fh.read(4096)
        if data:
            lines.append('T0-ack: ' + data.decode('utf-8', 'replace').rstrip())
            last = time.time()
        elif time.time() - last >= 0.3:
            break
        else:
            time.sleep(0.02)
    fh.close()

    if out_path:
        with open(out_path, 'w', encoding='utf-8') as f:
            for line in lines:
                f.write(line + '\n')
    return lines


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM6'
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    dump = os.path.join(os.environ.get('TEMP', '.'), 'ch8_capture.txt')

    t0 = time.time()
    lines = read_lines(port, secs, dump)
    span = time.time() - t0

    print('port %s: %d line(s) in %.1f s (dump: %s)' % (port, len(lines), span, dump))

    s_lines = [l for l in lines if l.startswith('S,')]
    other = [l for l in lines if not l.startswith('S,')]

    print('\n--- non-sample lines (%d) ---' % len(other))
    for line in other[:40]:
        print('   ' + line)
    if len(other) > 40:
        print('   ... %d more' % (len(other) - 40))

    print('\n--- sample lines: %d ---' % len(s_lines))
    if not s_lines:
        return 0

    cols = collections.OrderedDict()
    for tok in s_lines[0].split(',')[3:]:
        ch, _, _v = tok.partition(':')
        cols[ch] = []

    for line in s_lines:
        for tok in line.split(',')[3:]:
            ch, _, val = tok.partition(':')
            if ch in cols and val.lstrip('-').isdigit():
                cols[ch].append(int(val))

    for i in (0, len(s_lines) // 2, len(s_lines) - 1):
        print('   %s' % s_lines[i])

    print('\n   column   n     min       max      span    distinct')
    for ch, vals in cols.items():
        if not vals:
            print('   %-6s   -     (no values)' % ch)
            continue
        print('   %-6s %5d  %6d  %6d  %6d  %8d'
              % (ch, len(vals), min(vals), max(vals), max(vals) - min(vals),
                 len(set(vals))))

    gen = [int(l.split(',')[2]) for l in s_lines]
    print('   gen    %5d  %6d  %6d  %6d  %8d'
          % (len(gen), min(gen), max(gen), max(gen) - min(gen), len(set(gen))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
