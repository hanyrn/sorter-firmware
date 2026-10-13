#!/usr/bin/env python3
"""
Algorithm model-check for EventAggregator in event_aggregator.h.

It mirrors the C++ aggregator's window rules - the primary's band breakout opening the
shared window, the primary's own close ending it, and a breakout that never became an
event dropping it again - plus the per-channel records, including the rule that every
channel inherits the primary's judged *duration* rather than the window it was measured
inside.  What it does not mirror is the four-phase detector underneath (see the REWORK
NOTE in model_check.py), so its counts on a noisy stream can still differ:
test/test_event_aggregator.cpp is authoritative where the two disagree.

This is a 1:1 mirror of the C++ aggregator (LINKED and PER_CHANNEL), built on top
of the EventDetector mirror in model_check.py so both files always agree.  It
validates the window/group logic and the per-channel records when no C++ compiler
is available; test/test_event_aggregator.cpp exercises the real C++ instead.

Scenarios
---------
1. LINKED       : ch8 is the primary; ch0 sees a smaller version of the same
                  peaks, ch1 sees nothing, ch2 is not in the channel mask.  One
                  event must carry one record per MASKED channel, every width must
                  be ch8's, ch0 must be smaller than ch8 and ch1 must be absent.
2. LINKED clip  : ch0's peak arrives late, so the window closes while ch0 is
                  still rising: its height must be clipped (much smaller than its
                  real amplitude) and the record must be flagged TRUNCATED.
3. PER_CHANNEL  : ch0, ch1, ch8 and ch9 all fire, but with different widths; the
                  widths must differ and every channel must report its own
                  metrics while the window covers the whole group.
6. PC fault     : ch8 is unplugged - its input steps up and never returns, so the
                  detector's max_width_samples guard closes its "event".  ch0's
                  pulses must still be reported with ch0's verdict, and as they
                  close rather than when ch8's fault finally does.

Run:  python test/model_check_aggregator.py
"""

import math
import random

import model_check as mc

MODE_PER_CHANNEL = 0
MODE_LINKED = 1

CH_FLAG_PRESENT = 0x01
CH_FLAG_TRUNCATED = 0x02
CH_FLAG_RAILED = 0x04

MAX_CH = 16


def rint(f):
    """C's lroundf(): round half away from zero."""
    return int(math.floor(f + 0.5)) if f >= 0 else -int(math.floor(-f + 0.5))


def apply_input_polarity(channels, invert_mask):
    """Mirror of applyInputPolarity(): negate every channel whose bit is set.

    A channel that idles HIGH and dips on an event (the primary / 9th here) is
    inverted so the rising-edge peak logic measures it like the others.
    """
    for ch in range(MAX_CH):
        if invert_mask & (1 << ch):
            channels[ch] = -channels[ch]


def classify_peak(cfg, max_value, width_samples, area, rise_samples):
    """Mirror of EventDetector::classifyPeak().

    `rise_samples` is the length of the leading edge (event start -> maximum); the
    two shape guards judge that edge, not the average slope over the whole width.
    """
    if max_value < cfg.min_max_value:
        return mc.REASON_MAX_TOO_SMALL
    if width_samples < cfg.min_width_samples:
        return mc.REASON_WIDTH_TOO_NARROW
    if width_samples > cfg.max_width_samples:
        return mc.REASON_WIDTH_TOO_WIDE
    if area < cfg.min_area:
        return mc.REASON_AREA_TOO_SMALL
    rise = rise_samples if rise_samples else 1     # 0 = a vertical edge
    if rise < cfg.min_rise_samples or max_value / rise > cfg.max_slope:
        return mc.REASON_SHORT_FOR_HEIGHT
    return mc.REASON_OK


class AggConfig:
    def __init__(self):
        self.detector = mc.Config()
        self.channel_mask = 0x0000FFFF
        self.mode = MODE_LINKED
        self.primary = 0xFF
        # PER_CHANNEL only: how far two channels' judged widths may differ and still
        # be taken as measuring the same event, in percent of the wider one (0 = the
        # test is off).  Mirrors EventAggregator::Config::width_agree_pct.
        self.width_agree_pct = 50
        # saturation ("rail") guard: |sample| >= rail_level is clipped at full scale
        self.rail_level = 32000
        self.rail_width = 8
        self.rail_cooldown = 1000


class EventAggregator:
    def __init__(self, cfg):
        self.configure(cfg)

    def configure(self, cfg):
        self.cfg = cfg
        self.primary = 0xFF
        if cfg.channel_mask != 0:
            if 0 <= cfg.primary < MAX_CH and self.in_mask(cfg.primary):
                self.primary = cfg.primary
            else:
                for ch in range(MAX_CH):
                    if self.in_mask(ch):
                        self.primary = ch
                        break
        self.reset()

    def reset(self):
        self.counter = 0
        self.det = [mc.EventDetector(self.cfg.detector) for _ in range(MAX_CH)]
        # Each channel's input stage (mirrors ImpulseFilter in event_detector.h): the
        # window measures the same impulse-free stream the detectors judge.
        self.filt = [mc.ImpulseFilter(self.cfg.detector.impulse_filter_taps)
                     for _ in range(MAX_CH)]
        self.win_open = False
        z = [0] * MAX_CH
        self.win_base, self.win_on, self.win_off = list(z), list(z), list(z)
        self.win_max, self.win_area = list(z), list(z)
        self.win_max_ix, self.win_rose_ix = list(z), list(z)
        self.win_rose, self.win_high = [False] * MAX_CH, [False] * MAX_CH
        self.win_below = [0] * MAX_CH
        self.win_back = [False] * MAX_CH
        self.pc_pending = 0
        self.pc_start_us = 0
        self.pc_start_ix = 0
        self.pc_metrics = [None] * MAX_CH
        self.pc_done = [False] * MAX_CH
        self.pc_in = [False] * MAX_CH
        # Its last event was closed by the max_width_samples fault guard (unplugged /
        # floating input): held out of the group until it closes one normally again.
        self.pc_fault = [False] * MAX_CH
        self.rail_count = [0] * MAX_CH
        self.rail_holdoff = [0] * MAX_CH
        self.rail_skip = [False] * MAX_CH
        self.ch_railed = [False] * MAX_CH
        self.rail_notified = False

    def in_mask(self, ch):
        return (self.cfg.channel_mask & (1 << ch)) != 0

    def update(self, channels, t_us, index):
        # ---- input conditioning: the impulse filter of every masked channel ----
        # (mirrors EventAggregator::update(): fed unconditionally, even for a channel
        # that is railed or held out of detection, so it always follows the stream)
        for ch in range(MAX_CH):
            if self.in_mask(ch):
                self.filt[ch].step(channels[ch])

        # ---- saturation ("rail") guard: runs before any detection -------------
        rail_new = False     # a channel was declared railed on THIS sample
        any_busy = False     # some channel is currently railed / held off
        for ch in range(MAX_CH):
            self.rail_skip[ch] = False
            if not self.in_mask(ch):
                continue
            mag = -channels[ch] if channels[ch] < 0 else channels[ch]

            if self.rail_holdoff[ch] > 0:
                self.rail_skip[ch] = True          # held out of detection
                any_busy = True
                if mag >= self.cfg.rail_level:
                    self.rail_holdoff[ch] = self.cfg.rail_cooldown   # still bright
                else:
                    self.rail_holdoff[ch] -= 1
                    if self.rail_holdoff[ch] == 0:
                        self.det[ch].rebaseline(channels[ch])        # settled: resume
                continue

            if mag >= self.cfg.rail_level:
                self.rail_skip[ch] = True
                any_busy = True
                self.rail_count[ch] += 1
                if self.rail_count[ch] >= self.cfg.rail_width:
                    self.rail_count[ch] = 0
                    self.rail_holdoff[ch] = self.cfg.rail_cooldown
                    self.ch_railed[ch] = True
                    rail_new = True
            else:
                self.rail_count[ch] = 0

        if self.cfg.mode == MODE_LINKED:
            rec = self.update_linked(channels, t_us, index)
        else:
            rec = self.update_per_channel(channels, t_us, index)

        # ---- rail notification: exactly one bundle per rail episode ----------
        if rail_new and not self.rail_notified:
            self.rail_notified = True
            if self.cfg.mode == MODE_LINKED:
                self.win_open = False
            rec = self.build_rail_record(t_us)
        if not any_busy:
            self.rail_notified = False
            for ch in range(MAX_CH):
                self.ch_railed[ch] = False

        if rec is not None:
            rec["bundle"]["index"] = self.counter
            self.counter += 1
        return rec

    # -- LINKED: the primary times the event for everybody -------------------
    def update_linked(self, channels, t_us, index):
        if self.primary == 0xFF:
            return None

        for ch in range(MAX_CH):
            if ch == self.primary or not self.in_mask(ch) or self.rail_skip[ch]:
                continue
            self.det[ch].update(channels[ch], t_us, index)   # keep the reference alive

        # A railed primary cannot time the window: update() flushes it as a rail
        # notification, so just report "nothing closed" here.
        if self.rail_skip[self.primary]:
            return None

        was_in = self.det[self.primary].in_event()
        ref = self.det[self.primary].update(channels[self.primary], t_us, index)
        now_in = self.det[self.primary].in_event()

        if not self.win_open:
            if not was_in and now_in:
                # The primary's window just opened - it left its noise band on this
                # sample, which is where the event it may become starts (see
                # EventDetector::in_event()).
                self.open_window()
                self.accumulate(self.primary, self.filt[self.primary].value, index)
            return None

        for ch in range(MAX_CH):
            if self.in_mask(ch):
                self.accumulate(ch, self.filt[ch].value, index)

        if ref is not None:
            # The primary's event closed: that is the end of the window for everybody.
            rec = self.build_linked_record(ref)
            self.win_open = False
            return rec

        # The primary is out of its window and closed no event: what opened it was a band
        # breakout that never became one, so the window goes with the detector's.
        if was_in and not now_in:
            self.win_open = False
        return None

    def open_window(self):
        for ch in range(MAX_CH):
            self.win_max[ch] = 0
            self.win_area[ch] = 0
            self.win_rose[ch] = False
            self.win_high[ch] = False
            self.win_max_ix[ch] = 0
            self.win_rose_ix[ch] = 0
            self.win_below[ch] = 0
            self.win_back[ch] = False
            if not self.in_mask(ch):
                continue
            base = rint(self.det[ch].mean)
            sigma = self.det[ch].sigma
            self.win_base[ch] = base
            # The same expressions the detector uses when its own event begins (the
            # rounded baseline, thresholds off it), so a channel the detector closed is
            # also seen as closed here - see openWindow() in event_aggregator.h.
            self.win_on[ch] = rint(base + self.cfg.detector.k_on * sigma)
            self.win_off[ch] = rint(base + self.cfg.detector.k_off * sigma)
        self.win_open = True

    def accumulate(self, ch, x, index):
        amp = x - self.win_base[ch]
        if amp > self.win_max[ch]:
            self.win_max[ch] = amp
            self.win_max_ix[ch] = index
        if amp > 0:
            self.win_area[ch] += amp
        if not self.win_rose[ch]:
            # First crossing inside the window: the channel joins the measurement here
            # and the leading edge is counted from this sample.
            if x > self.win_on[ch]:
                self.win_rose[ch] = True
                self.win_rose_ix[ch] = index
                self.win_below[ch] = 0
                self.win_back[ch] = False
        elif self.win_back[ch] and x > self.win_on[ch]:
            # Closed, then crossed again: the window holds a *new* excursion now.
            self.win_below[ch] = 0
            self.win_back[ch] = False
        # CH_FLAG_TRUNCATED = the window ended while the channel was still well inside
        # its (latest) excursion.  "Back down" needs end_debounce consecutive samples
        # below the off-threshold (exactly the detector's own end rule) and then
        # latches, so a noise run at the window's edge cannot reopen an excursion that
        # finished inside the window.
        if self.win_rose[ch] and not self.win_back[ch]:
            if x < self.win_off[ch]:
                if self.win_below[ch] < self.cfg.detector.end_debounce:
                    self.win_below[ch] += 1
                if self.win_below[ch] >= self.cfg.detector.end_debounce:
                    self.win_back[ch] = True
            else:
                self.win_below[ch] = 0
        # "Well inside", i.e. at the last sample still half its own hysteresis band
        # above the off-threshold - not merely a count above it.  The two AD7606
        # modules are separate converters: the same waveform crosses their
        # off-thresholds a few samples apart, and without this margin each event
        # would flag the channel whose tail happens to sit higher.
        band = (self.win_on[ch] - self.win_off[ch]) // 2
        self.win_high[ch] = (self.win_rose[ch] and not self.win_back[ch]
                             and x > self.win_off[ch] + band)

    def build_linked_record(self, ref):
        chans = []
        for ch in range(MAX_CH):
            if not self.in_mask(ch):
                continue
            flags = 0
            if self.win_rose[ch]:
                flags |= CH_FLAG_PRESENT
                if self.win_high[ch]:
                    flags |= CH_FLAG_TRUNCATED
                # This channel's own leading edge inside the window: from the sample
                # it crossed its on-threshold to the sample it peaked at.  A maximum
                # that came before the crossing (only the tail of that channel was
                # captured) counts as a step, i.e. 0.
                rise = (self.win_max_ix[ch] - self.win_rose_ix[ch]
                        if self.win_max_ix[ch] >= self.win_rose_ix[ch] else 0)
                reason = classify_peak(self.cfg.detector, self.win_max[ch],
                                       ref["duration_samples"], self.win_area[ch], rise)
            else:
                reason = mc.REASON_MAX_TOO_SMALL
            chans.append({
                "channel": ch, "flags": flags, "reason": reason,
                "baseline": self.win_base[ch], "max_value": self.win_max[ch],
                # One event, one width - and it is the *duration* the primary's detector
                # judged with, not the window: a channel judged here is judged with the
                # same length the primary's own classifier used, exactly as
                # buildLinkedRecord() does in event_aggregator.h.
                "width_samples": ref["duration_samples"],
                "width_us": ref["duration_us"], "area": self.win_area[ch],
            })
        bundle = {
            "start_us": ref["start_us"],
            "end_us": ref["start_us"] + ref["width_us"],
            "width_samples": ref["width_samples"], "width_us": ref["width_us"],
            "primary": self.primary, "mode": MODE_LINKED,
            "n_channels": len(chans), "valid": ref["valid"], "reason": ref["reason"],
        }
        return {"bundle": bundle, "ch": chans}

    # -- PER_CHANNEL: every channel times itself -----------------------------
    def update_per_channel(self, channels, t_us, index):
        opened = 0
        closed = 0
        counted = 0     # of those closed, the ones that were part of the group
        for ch in range(MAX_CH):
            if not self.in_mask(ch):
                continue
            if self.rail_skip[ch]:
                # Railed/held off: cannot close on its own, so drop it from the open
                # group (its rail bundle is emitted by update()) to avoid a hang.
                if self.pc_in[ch]:
                    self.pc_in[ch] = False
                    if self.pc_pending > 0:
                        self.pc_pending -= 1
                continue
            # A channel the fault guard closed last time is not measuring this event: it
            # is held out of the group until it closes an event normally again (pc_fault).
            # Its own detector keeps running - the fault still gets its own row, and
            # closing normally is how it is seen to recover - but it cannot hold the group
            # open.  Mirrors updatePerChannel() in event_aggregator.h.
            held = self.pc_fault[ch]
            was_in = self.det[ch].in_event()
            m = self.det[ch].update(channels[ch], t_us, index)
            if not was_in and self.det[ch].in_event():
                opened |= (1 << ch)
                if not held:
                    self.pc_in[ch] = True
            if m is not None:
                closed |= (1 << ch)
                if self.pc_in[ch]:
                    counted |= (1 << ch)
                self.pc_metrics[ch] = m
                self.pc_in[ch] = False
                # Did the fault guard close it?  Then it never came back to its
                # baseline, so its width is the guard's bound and not the event's.
                self.pc_fault[ch] = (m["width_samples"] >=
                                     self.cfg.detector.max_width_samples)
            elif was_in and not self.det[ch].in_event():
                # The channel's own window closed without an event: a band breakout that
                # never became one.  It was never in an event, so it leaves the group (see
                # update_per_channel() in event_aggregator.h).
                if self.pc_in[ch]:
                    self.pc_in[ch] = False
                    if self.pc_pending > 0:
                        self.pc_pending -= 1

        done = False
        if closed != 0:
            for ch in range(MAX_CH):
                if (closed >> ch) & 1:
                    self.pc_done[ch] = True
                    # Only a channel that really was in the group may be counted down: a
                    # held-out one (pc_fault) closes without ever having taken a place in
                    # it, and a stray decrement would complete a group that is still open.
                    if (counted >> ch) & 1 and self.pc_pending > 0:
                        self.pc_pending -= 1
            done = (self.pc_pending == 0)

        rec = self.build_per_channel_record(t_us, index) if done else None

        # A channel that opens on the very sample another group closed starts the
        # next group (pc_pending is 0 again at this point).  A held-out channel opens
        # the window when nothing else is open - so the fault report it will produce is
        # about the event it actually saw - but it does not take a place in the group:
        # pc_in is what "in the group" means here.
        if opened != 0:
            window_set = False
            for ch in range(MAX_CH):
                if (opened >> ch) & 1:
                    if self.pc_pending == 0 and not window_set:
                        self.pc_start_us = t_us
                        self.pc_start_ix = index
                        window_set = True
                    if self.pc_in[ch]:
                        self.pc_pending += 1
        return rec

    def build_per_channel_record(self, t_us, index):
        # Who speaks for the bundle (see choose_per_channel_verdict): the primary while it
        # measured this event, else the lowest channel that did.  It has to be decided
        # before the loop below consumes pc_done - that is what the C++ does too
        # (buildPerChannelRecord clears pc_done on its way out).
        verdict_ch = self.choose_per_channel_verdict()

        chans = []
        for ch in range(MAX_CH):
            if not self.in_mask(ch):
                continue
            cm = {"channel": ch, "flags": 0, "reason": mc.REASON_MAX_TOO_SMALL,
                  "baseline": 0, "max_value": 0, "width_samples": 0,
                  "width_us": 0, "area": 0}
            if self.pc_done[ch]:
                m = self.pc_metrics[ch]
                cm["baseline"] = m["baseline"]
                cm["max_value"] = m["max_value"]
                # Its own judged width: the channel's excursion read back through its
                # pulse's envelope (duration_samples/duration_us), not the excursion
                # itself - widths may legitimately differ, that is the point of this mode.
                cm["width_samples"] = m["duration_samples"]
                cm["width_us"] = m["duration_us"]
                cm["area"] = m["area"]
                cm["flags"] = CH_FLAG_PRESENT
                cm["reason"] = m["reason"]
                # A channel the fault guard closed was cut off, not measured - which is
                # what CH_FLAG_TRUNCATED says in linked mode too.  Its reason already reads
                # WIDTH_TOO_WIDE when the envelope correction pushed the duration past the
                # bound, but a fault-closed step is refused by the shape guard instead
                # (SHORT_FOR_HEIGHT), and the flag has to name the fault either way.
                if m["reason"] == mc.REASON_WIDTH_TOO_WIDE or self.pc_guard_closed(ch):
                    cm["flags"] |= CH_FLAG_TRUNCATED
            self.pc_done[ch] = False
            chans.append(cm)

        if verdict_ch != 0xFF:
            valid = 1 if self.pc_metrics[verdict_ch]["reason"] == mc.REASON_OK else 0
            reason = self.pc_metrics[verdict_ch]["reason"]
        else:
            # Nothing measured the event: the bundle carries no verdict (as before).
            valid, reason = 0, mc.REASON_MAX_TOO_SMALL

        bundle = {
            "start_us": self.pc_start_us, "end_us": t_us,
            "width_samples": index - self.pc_start_ix, "width_us": t_us - self.pc_start_us,
            "primary": verdict_ch, "mode": MODE_PER_CHANNEL,
            "n_channels": len(chans), "valid": valid, "reason": reason,
        }
        self.pc_pending = 0
        return {"bundle": bundle, "ch": chans}

    # -- PER_CHANNEL: which channel's verdict speaks for the bundle ----------
    # Mirrors choosePerChannelVerdict() and its helpers in event_aggregator.h.  A channel
    # that never came back to its baseline (the detector's max_width_samples fault guard
    # closed its event) did see *an* event, but not the one the wired channels saw, so it
    # must not speak for the bundle - nor may a channel whose judged width no other
    # measuring channel agrees with (see width_agree_pct).
    @staticmethod
    def widths_agree(a, b, pct):
        if pct == 0:
            return True
        hi, lo = (a, b) if a >= b else (b, a)
        if hi == 0:
            return lo == 0
        return (hi - lo) * 100 <= hi * pct

    def pc_measured_ok(self, ch):
        return self.pc_done[ch] and self.pc_metrics[ch]["reason"] == mc.REASON_OK

    def pc_guard_closed(self, ch):
        # The excursion, not the judged duration: the guard closes a channel exactly
        # max_width_samples after its event started, so an excursion that reached the
        # bound is a guard closure and nothing else.
        return (self.pc_done[ch] and
                self.pc_metrics[ch]["width_samples"] >= self.cfg.detector.max_width_samples)

    def pc_speaks(self, ch):
        if not self.pc_measured_ok(ch) or self.pc_guard_closed(ch):
            return False
        any_other = False
        for o in range(MAX_CH):
            if o == ch or not self.in_mask(o):
                continue
            if not self.pc_measured_ok(o) or self.pc_guard_closed(o):
                continue                       # not a measurement, so it cannot outvote
            any_other = True
            if self.widths_agree(self.pc_metrics[ch]["duration_samples"],
                                 self.pc_metrics[o]["duration_samples"],
                                 self.cfg.width_agree_pct):
                return True
        # Nobody measured the event with it, so nothing outvoted it - the single connected
        # channel of a partly unplugged harness is exactly this case.
        return not any_other

    def choose_per_channel_verdict(self):
        if 0 <= self.primary < MAX_CH and self.pc_speaks(self.primary):
            return self.primary
        for ch in range(MAX_CH):
            if self.in_mask(ch) and self.pc_speaks(ch):
                return ch
        # Every channel that measured the event is fault-closed or outvoted: report the
        # lowest one rather than pretending the event has no verdict at all.
        for ch in range(MAX_CH):
            if self.in_mask(ch) and self.pc_measured_ok(ch):
                return ch
        return 0xFF        # nothing measured: the bundle carries no verdict (as before)

    # -- RAIL: one notification bundle per rail episode ----------------------
    def build_rail_record(self, t_us):
        chans = []
        for ch in range(MAX_CH):
            if not self.in_mask(ch):
                continue
            flags = 0
            reason = mc.REASON_MAX_TOO_SMALL
            if self.ch_railed[ch]:
                flags = CH_FLAG_RAILED
                reason = mc.REASON_RAILED
            chans.append({
                "channel": ch, "flags": flags, "reason": reason,
                "baseline": rint(self.det[ch].mean), "max_value": 0,
                "width_samples": 0, "width_us": 0, "area": 0,
            })
        bundle = {
            "start_us": t_us, "end_us": t_us, "width_samples": 0, "width_us": 0,
            "primary": self.primary, "mode": self.cfg.mode,
            "n_channels": len(chans), "valid": 0, "reason": mc.REASON_RAILED,
        }
        return {"bundle": bundle, "ch": chans}


# ===========================================================================
# signal helpers (mirroring the C++ host test)
# ===========================================================================
def gaussian_peak(i, center, width, amp):
    d = i - center
    return amp * math.exp(-(d * d) / (2.0 * width * width))


def sample(n, dt_us, channels_fn, seed=1234):
    """[(values[16], t_us), ...] - one snapshot per sample index."""
    random.seed(seed)
    out = []
    for i in range(n):
        vals = [0] * MAX_CH
        for ch, level in channels_fn(i, random.gauss).items():
            vals[ch] = rint(level)
        out.append((vals, i * dt_us))
    return out


def run(agg, samples):
    events = []
    for idx, (vals, t) in enumerate(samples):
        rec = agg.update(vals, t, idx)
        if rec is not None:
            events.append(rec)
    return events


def ch_by_id(chans, ch):
    for c in chans:
        if c["channel"] == ch:
            return c
    return None


# ===========================================================================
# scenario 1: LINKED, one primary channel times everybody
# ===========================================================================
def test_linked():
    cfg = AggConfig()
    cfg.mode = MODE_LINKED
    cfg.channel_mask = 0x0103          # ch0, ch1, ch8 - ch2 stays out on purpose
    cfg.primary = 8

    def levels(i, gauss):
        base0 = 2000.0 + gauss() * 10.0
        base1 = 2000.0 + gauss() * 10.0
        base8 = 2000.0 + gauss() * 10.0
        for c in (6000, 13000, 20000):
            base8 += gaussian_peak(i, c, 200, 1600.0)     # primary
            base0 += gaussian_peak(i, c, 200, 800.0)      # same shape, half height
        return {0: base0, 1: base1, 8: base8, 2: 5000.0}  # ch2: not in the mask

    agg = EventAggregator(cfg)
    events = run(agg, sample(26000, 10, levels))
    failures = []

    if agg.primary != 8:
        failures.append("primary resolved to %s, expected 8" % agg.primary)
    if len(events) != 3:
        failures.append("expected 3 events, got %d" % len(events))

    for rec in events:
        b = rec["bundle"]
        if b["n_channels"] != 3 or [c["channel"] for c in rec["ch"]] != [0, 1, 8]:
            failures.append("event %d: channels %s, expected [0, 1, 8]"
                            % (b["index"], [c["channel"] for c in rec["ch"]]))
            continue
        if b["mode"] != MODE_LINKED or b["primary"] != 8 or b["valid"] != 1:
            failures.append("event %d: mode/primary/valid = %d/%d/%d"
                            % (b["index"], b["mode"], b["primary"], b["valid"]))
        c0, c1, c8 = ch_by_id(rec["ch"], 0), ch_by_id(rec["ch"], 1), ch_by_id(rec["ch"], 8)
        # One event, one width: every channel inherits the primary's judged duration (the
        # window widened for the pulse's envelope, see EventDetector::envelope_duration) -
        # never "the channel width equals the window", which is the number it was measured
        # *inside* (see buildLinkedRecord in event_aggregator.h).
        for c in (c0, c1, c8):
            if c["width_samples"] != c8["width_samples"]:
                failures.append("event %d ch%d: width %d != %d - one event, one width"
                                % (b["index"], c["channel"], c["width_samples"],
                                   c8["width_samples"]))
        if c8["width_samples"] < b["width_samples"]:
            failures.append("event %d: judged width %d is shorter than the window %d"
                            % (b["index"], c8["width_samples"], b["width_samples"]))
        if c0["flags"] != CH_FLAG_PRESENT:
            failures.append("event %d ch0: flags 0x%02x, expected PRESENT"
                            % (b["index"], c0["flags"]))
        if not 500 <= c0["max_value"] <= 1100:
            failures.append("event %d ch0: height %d, expected ~800"
                            % (b["index"], c0["max_value"]))
        if not 1300 <= c8["max_value"] <= 1900:
            failures.append("event %d ch8: height %d, expected ~1600"
                            % (b["index"], c8["max_value"]))
        if c0["max_value"] >= c8["max_value"]:
            failures.append("event %d: ch0 (%d) not smaller than ch8 (%d)"
                            % (b["index"], c0["max_value"], c8["max_value"]))
        if c0["area"] <= 0 or c8["area"] <= int(c0["area"] * 1.5):
            failures.append("event %d: ch0 area %d not clearly below ch8 area %d"
                            % (b["index"], c0["area"], c8["area"]))
        if c1["flags"] != 0 or c1["reason"] != mc.REASON_MAX_TOO_SMALL:
            failures.append("event %d ch1: flags 0x%02x reason %d, expected absent"
                            % (b["index"], c1["flags"], c1["reason"]))
        if not 0 <= c1["max_value"] < 160:
            failures.append("event %d ch1: height %d, expected noise only (<160)"
                            % (b["index"], c1["max_value"]))

    print("LINKED: %d events" % len(events))
    for rec in events:
        b = rec["bundle"]
        rows = "  ".join("ch%d:%s h=%d w=%d" % (c["channel"], "P" if c["flags"] else "-",
                                                c["max_value"], c["width_samples"])
                         for c in rec["ch"])
        print("  #%d valid=%d window=%dsmp/%dus  %s"
              % (b["index"], b["valid"], b["width_samples"], b["width_us"], rows))
    return failures


# ===========================================================================
# scenario 2: LINKED, a late channel is clipped by the window
# ===========================================================================
def test_linked_clip():
    cfg = AggConfig()
    cfg.mode = MODE_LINKED
    cfg.channel_mask = 0x0103
    cfg.primary = 8

    def levels(i, gauss):
        base0 = 2000.0 + gauss() * 10.0
        base1 = 2000.0 + gauss() * 10.0
        base8 = 2000.0 + gauss() * 10.0
        base8 += gaussian_peak(i, 6000, 200, 1600.0)  # primary: window 5450..6600
        base0 += gaussian_peak(i, 7000, 200, 1600.0)  # arrives after the window closes
        return {0: base0, 1: base1, 8: base8}

    agg = EventAggregator(cfg)
    events = run(agg, sample(12000, 10, levels))
    failures = []
    if len(events) != 1:
        failures.append("clip: expected 1 event, got %d" % len(events))
        return failures

    rec = events[0]
    b = rec["bundle"]
    c0 = ch_by_id(rec["ch"], 0)
    c8 = ch_by_id(rec["ch"], 8)
    if c0["flags"] != (CH_FLAG_PRESENT | CH_FLAG_TRUNCATED):
        failures.append("clip: ch0 flags 0x%02x, expected PRESENT|TRUNCATED" % c0["flags"])
    if not 0 < c0["max_value"] < 600:
        failures.append("clip: ch0 height %d, expected the window edge (~200)"
                        % c0["max_value"])
    if c0["max_value"] >= c8["max_value"]:
        failures.append("clip: ch0 (%d) not clipped below ch8 (%d)"
                        % (c0["max_value"], c8["max_value"]))
    if c0["width_samples"] != c8["width_samples"]:
        failures.append("clip: ch0 width %d != ch8 width %d - one event, one width"
                        % (c0["width_samples"], c8["width_samples"]))
    print("LINKED clip: #%d window=%dsmp  ch8 h=%d  ch0 h=%d flags=0x%02x"
          % (b["index"], b["width_samples"], c8["max_value"], c0["max_value"], c0["flags"]))
    return failures


# ===========================================================================
# scenario 3: PER_CHANNEL, every channel times itself
# ===========================================================================
def test_per_channel():
    cfg = AggConfig()
    cfg.mode = MODE_PER_CHANNEL
    cfg.channel_mask = 0x0303          # ch0, ch1, ch8, ch9 - ch2..ch7 stay out
    cfg.primary = 0                    # the channel whose verdict names the event

    def levels(i, gauss):
        base0 = 2000.0 + gauss() * 5.0
        base1 = 2000.0 + gauss() * 5.0
        base8 = 2000.0 + gauss() * 5.0
        base9 = 2000.0 + gauss() * 10.0
        for c in (6000, 13000):
            base0 += gaussian_peak(i, c, 200, 1600.0)        # broad
            base1 += gaussian_peak(i, c + 100, 60, 1200.0)   # narrow, inside ch0
            base8 += gaussian_peak(i, c + 20, 150, 1400.0)
            base9 += gaussian_peak(i, c - 30, 100, 1000.0)
        return {0: base0, 1: base1, 8: base8, 9: base9}

    agg = EventAggregator(cfg)
    events = run(agg, sample(26000, 10, levels))
    failures = []

    if len(events) != 2:
        failures.append("per-channel: expected 2 events, got %d" % len(events))

    for rec in events:
        b = rec["bundle"]
        if b["mode"] != MODE_PER_CHANNEL:
            failures.append("event %d: mode %d, expected PER_CHANNEL" % (b["index"], b["mode"]))
        if b["n_channels"] != 4:
            failures.append("event %d: %d records, expected 4" % (b["index"], b["n_channels"]))
            continue
        c0, c1 = ch_by_id(rec["ch"], 0), ch_by_id(rec["ch"], 1)
        for c in rec["ch"]:
            if c["flags"] != CH_FLAG_PRESENT:
                failures.append("event %d ch%d: flags 0x%02x, expected PRESENT"
                                % (b["index"], c["channel"], c["flags"]))
            if c["reason"] != mc.REASON_OK:
                failures.append("event %d ch%d: reason %d, expected OK"
                                % (b["index"], c["channel"], c["reason"]))
        # every channel timed itself: the narrow one must be clearly narrower
        if not c1["width_samples"] < c0["width_samples"] * 0.6:
            failures.append("event %d: ch1 width %d not clearly below ch0 width %d"
                            % (b["index"], c1["width_samples"], c0["width_samples"]))
        if not 1 <= c1["width_us"] < c0["width_us"]:
            failures.append("event %d: ch1 width_us %d vs ch0 %d"
                            % (b["index"], c1["width_us"], c0["width_us"]))
        # The window is the group's span; each channel's width is that channel's own
        # duration.  Those are different quantities (see the note in test_linked): the
        # widest channel's duration reaches past the window its excursion fit in, so the
        # window is checked against the narrow channel - exactly like the C++ test.
        if b["width_samples"] < c1["width_samples"]:
            failures.append("event %d: window %d is shorter than the narrow channel %d"
                            % (b["index"], b["width_samples"], c1["width_samples"]))
        if not 1000 <= c1["max_value"] <= 1400:
            failures.append("event %d ch1: height %d, expected ~1200"
                            % (b["index"], c1["max_value"]))
        if c1["area"] <= 0 or c1["area"] >= c0["area"]:
            failures.append("event %d: ch1 area %d vs ch0 area %d"
                            % (b["index"], c1["area"], c0["area"]))

    print("PER_CHANNEL: %d events" % len(events))
    for rec in events:
        b = rec["bundle"]
        rows = "  ".join("ch%d:%dus/%dsmp" % (c["channel"], c["width_us"], c["width_samples"])
                         for c in rec["ch"])
        print("  #%d valid=%d window=%dus/%dsmp  %s"
              % (b["index"], b["valid"], b["width_us"], b["width_samples"], rows))
    return failures


# ===========================================================================
# scenario 4: LINKED, the primary rails -> one notification, then recovers
# ===========================================================================
def test_rail():
    cfg = AggConfig()
    cfg.mode = MODE_LINKED
    cfg.channel_mask = 0x0103          # ch0, ch1, ch8
    cfg.primary = 8

    def levels(i, gauss):
        base0 = 2000.0 + gauss() * 10.0
        base1 = 2000.0 + gauss() * 10.0
        base8 = 2000.0 + gauss() * 10.0
        if 4000 <= i < 4200:                       # bright burst: ch8 saturates
            base8 = 32700.0
        if 8000 <= i < 10000:                      # a normal peak after recovery
            base8 += gaussian_peak(i, 9000, 200, 1600.0)
        return {0: base0, 1: base1, 8: base8}

    agg = EventAggregator(cfg)
    events = run(agg, sample(14000, 10, levels))
    failures = []

    rails = [r for r in events if r["bundle"]["reason"] == mc.REASON_RAILED]
    valid = [r for r in events if r["bundle"]["valid"] == 1]
    if len(rails) != 1:
        failures.append("rail: %d rail bundles, expected exactly 1" % len(rails))
    if len(valid) != 1:
        failures.append("rail: %d valid events after recovery, expected 1" % len(valid))

    for rec in rails:
        b = rec["bundle"]
        if b["valid"] != 0:
            failures.append("rail: valid=%d, expected 0" % b["valid"])
        if b["n_channels"] != 3:
            failures.append("rail: %d records, expected 3" % b["n_channels"])
        c8 = ch_by_id(rec["ch"], 8)
        c0 = ch_by_id(rec["ch"], 0)
        if c8 is None or c8["flags"] != CH_FLAG_RAILED or c8["reason"] != mc.REASON_RAILED:
            failures.append("rail: ch8 not flagged RAILED")
        if c0 is not None and (c0["flags"] != 0 or c0["reason"] != mc.REASON_MAX_TOO_SMALL):
            failures.append("rail: ch0 should be absent in the rail bundle")

    if len(events) > 4:
        failures.append("rail: %d events total - detector is chattering after the rail"
                        % len(events))

    print("RAIL: %d events (%d rail, %d valid)" % (len(events), len(rails), len(valid)))
    return failures


# ===========================================================================
# scenario 5: an inverted channel (idles high, dips) is measured like the rest
# ===========================================================================
def test_invert():
    cfg = AggConfig()
    cfg.mode = MODE_LINKED
    cfg.channel_mask = (1 << 8)
    cfg.primary = 8

    DIP = 1600.0

    def levels(i, gauss):
        v = 25000.0 + gauss() * 10.0                # idles HIGH ...
        for c in (6000, 13000):
            v -= gaussian_peak(i, c, 200, DIP)      # ... and dips on an event
        return {8: v}

    agg = EventAggregator(cfg)
    failures = []
    events = []
    for idx, (vals, t) in enumerate(sample(20000, 10, levels)):
        apply_input_polarity(vals, 1 << 8)          # the fix under test
        rec = agg.update(vals, t, idx)
        if rec is not None:
            events.append(rec)

    # The two dips are the peaks under test.  Because the threshold is five sigma of
    # the *filtered* stream (which has a smaller variance than the raw samples, see
    # ImpulseFilter), a lone noise excursion can additionally be reported - but only
    # ever *rejected*: a one-sample excursion has no leading edge for the shape guard
    # to accept, which is exactly what that guard is for.  So the dips must come out
    # valid, and anything else must be a shape rejection.
    valid = [rec for rec in events if rec["bundle"]["valid"] == 1]
    if len(valid) != 2:
        failures.append("invert: %d valid events, expected 2 (the two dips)" % len(valid))
    for rec in events:
        b = rec["bundle"]
        c8 = ch_by_id(rec["ch"], 8)
        if b["valid"] == 1:
            if c8 is None or not (1200 <= c8["max_value"] <= 2000):
                failures.append("invert: event %d ch8 height %s, expected ~%d (dip depth)"
                                % (b["index"], None if c8 is None else c8["max_value"],
                                   int(DIP)))
        elif c8 is None or c8["max_value"] > 0.25 * DIP or \
                b["reason"] not in (mc.REASON_MAX_TOO_SMALL, mc.REASON_SHORT_FOR_HEIGHT,
                                    mc.REASON_WIDTH_TOO_NARROW):
            failures.append("invert: rejected event %d reason=%d height=%s - expected a "
                            "small noise excursion (max/shape bound)"
                            % (b["index"], b["reason"],
                               None if c8 is None else c8["max_value"]))
    print("INVERT: %d events (dips detected as peaks)" % len(events))
    for rec in events:
        b = rec["bundle"]
        c8 = ch_by_id(rec["ch"], 8)
        print("  #%d valid=%d reason=%d ch8 h=%d"
              % (b["index"], b["valid"], b["reason"], c8["max_value"]))
    return failures


# ===========================================================================
# scenario 6: PER_CHANNEL, an unplugged channel's fault speaks for nobody
# ===========================================================================
def test_per_channel_fault():
    """ch8's input is floating: it steps up once and never comes back, so it is the
    detector's own max_width_samples guard that closes its "event" - exactly
    max_width_samples after it opened, and ~25x wider than the pulses ch0 really saw
    (the bench with the 9th channel unplugged).  That fault must not become the
    bundle's verdict - ch0 measured the event, ch8 did not - nor may it pace the
    group: ch0's pulses are reported as they close, not 2 s later when ch8's fault
    finally does, because one broken input must not decide how often the M4 reports.
    Its own fault still gets a row: ch8 is flagged TRUNCATED, and that bundle carries
    no verdict (nothing measured it).
    """
    cfg = AggConfig()
    cfg.mode = MODE_PER_CHANNEL
    cfg.channel_mask = 0x0101           # ch0 and ch8 - ch8 is the unplugged one
    cfg.primary = 8                     # the configured channel is the broken one

    STEP_AT = 3500                      # after the 3000-sample quiet period
    guard = cfg.detector.max_width_samples

    def levels(i, gauss):
        base0 = 2000.0 + gauss() * 10.0
        for c in (24500, 26500):
            base0 += gaussian_peak(i, c, 150, 1600.0)
        base8 = 2000.0 + gauss() * 10.0
        if i >= STEP_AT:
            base8 += 800.0              # floating input: steps up and stays there
        return {0: base0, 8: base8}

    agg = EventAggregator(cfg)
    events = run(agg, sample(28000, 10, levels))
    failures = []

    good = [r for r in events if r["bundle"]["valid"] == 1]
    faults = [r for r in events
              if (ch_by_id(r["ch"], 8) or {}).get("flags", 0) & CH_FLAG_TRUNCATED]
    if len(good) != 2:
        failures.append("fault: %d valid pulses reported, expected 2 (ch0's own)"
                        % len(good))
    for rec in good:
        b = rec["bundle"]
        c0, c8 = ch_by_id(rec["ch"], 0), ch_by_id(rec["ch"], 8)
        if b["primary"] != 0:
            failures.append("fault: event %d spoke for ch%d, expected ch0 - the fault "
                            "outvoted it" % (b["index"], b["primary"]))
        if b["width_samples"] >= 2000:
            failures.append("fault: event %d reported the pulse with a %dsmp window - the "
                            "fault paced the group" % (b["index"], b["width_samples"]))
        if c0 is None or c0["flags"] != CH_FLAG_PRESENT or c0["reason"] != mc.REASON_OK:
            failures.append("fault: event %d ch0 is not a plain measurement (%s)"
                            % (b["index"], "missing" if c0 is None else
                               "flags 0x%02x reason %d" % (c0["flags"], c0["reason"])))
        if c8 is not None and c8["flags"] != 0:
            failures.append("fault: event %d: ch8's fault row rode in a good bundle "
                            "(flags 0x%02x)" % (b["index"], c8["flags"]))

    if len(faults) != 1:
        failures.append("fault: %d ch8 fault bundles, expected 1" % len(faults))
    for rec in faults:
        b = rec["bundle"]
        c8 = ch_by_id(rec["ch"], 8)
        # Nothing measured this event: ch8 was cut off by the guard and ch0 has not closed
        # a pulse yet, so the bundle has no verdict at all - as before.
        if b["primary"] != 0xFF:
            failures.append("fault: event %d spoke for ch%d - nothing measured it"
                            % (b["index"], b["primary"]))
        if b["valid"] != 0:
            failures.append("fault: event %d valid=%d - a fault is not a measurement"
                            % (b["index"], b["valid"]))
        if c8 is None or c8["reason"] == mc.REASON_OK:
            failures.append("fault: event %d ch8 reads as a measurement" % b["index"])
        if c8 is not None and c8["width_samples"] < guard:
            failures.append("fault: event %d ch8 width %d, expected at least the guard's "
                            "%d (it was held out, not measured)"
                            % (b["index"], c8["width_samples"], guard))

    if len(events) != len(good) + len(faults):
        failures.append("fault: %d events for %d pulses + %d faults"
                        % (len(events), len(good), len(faults)))

    print("PER_CHANNEL fault: %d events (%d valid pulses, %d ch8 fault)"
          % (len(events), len(good), len(faults)))
    for rec in events:
        b = rec["bundle"]
        rows = "  ".join("ch%d:%dus/%dsmp f=0x%02x"
                         % (c["channel"], c["width_us"], c["width_samples"], c["flags"])
                         for c in rec["ch"])
        print("  #%d valid=%d primary=%d window=%dsmp  %s"
              % (b["index"], b["valid"], b["primary"], b["width_samples"], rows))
    return failures


def main():
    failures = (test_linked() + test_linked_clip() + test_per_channel() +
                test_rail() + test_invert() + test_per_channel_fault())
    if failures:
        print("\nFAIL:")
        for f in failures:
            print("  -", f)
        raise SystemExit(1)
    print("\nPASS: aggregator groups the per-channel detectors as expected in both modes.")


if __name__ == "__main__":
    main()
