#!/usr/bin/env python3
"""
Algorithm model-check for EventAggregator in event_aggregator.h.

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


def classify_peak(cfg, max_value, width_samples, area):
    """Mirror of EventDetector::classifyPeak()."""
    if max_value < cfg.min_max_value:
        return mc.REASON_MAX_TOO_SMALL
    if width_samples < cfg.min_width_samples:
        return mc.REASON_WIDTH_TOO_NARROW
    if width_samples > cfg.max_width_samples:
        return mc.REASON_WIDTH_TOO_WIDE
    if area < cfg.min_area:
        return mc.REASON_AREA_TOO_SMALL
    slope = max_value / (width_samples if width_samples else 1)
    if slope > cfg.max_slope:
        return mc.REASON_SHORT_FOR_HEIGHT
    return mc.REASON_OK


class AggConfig:
    def __init__(self):
        self.detector = mc.Config()
        self.channel_mask = 0x0000FFFF
        self.mode = MODE_LINKED
        self.primary = 0xFF
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
        self.win_open = False
        z = [0] * MAX_CH
        self.win_base, self.win_on, self.win_off = list(z), list(z), list(z)
        self.win_max, self.win_area = list(z), list(z)
        self.win_rose, self.win_high = [False] * MAX_CH, [False] * MAX_CH
        self.pc_pending = 0
        self.pc_start_us = 0
        self.pc_start_ix = 0
        self.pc_metrics = [None] * MAX_CH
        self.pc_done = [False] * MAX_CH
        self.pc_in = [False] * MAX_CH
        self.rail_count = [0] * MAX_CH
        self.rail_holdoff = [0] * MAX_CH
        self.rail_skip = [False] * MAX_CH
        self.ch_railed = [False] * MAX_CH
        self.rail_notified = False

    def in_mask(self, ch):
        return (self.cfg.channel_mask & (1 << ch)) != 0

    def update(self, channels, t_us, index):
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

        was_in = self.det[self.primary].state == mc.EVENT
        ref = self.det[self.primary].update(channels[self.primary], t_us, index)
        now_in = self.det[self.primary].state == mc.EVENT

        if not self.win_open:
            if not was_in and now_in:
                self.open_window()
                self.accumulate(self.primary, channels[self.primary])
            return None

        for ch in range(MAX_CH):
            if self.in_mask(ch):
                self.accumulate(ch, channels[ch])

        if ref is None:
            return None

        rec = self.build_linked_record(ref)
        self.win_open = False
        return rec

    def open_window(self):
        for ch in range(MAX_CH):
            self.win_max[ch] = 0
            self.win_area[ch] = 0
            self.win_rose[ch] = False
            self.win_high[ch] = False
            if not self.in_mask(ch):
                continue
            base = self.det[ch].mean
            sigma = self.det[ch].sigma
            self.win_base[ch] = rint(base)
            self.win_on[ch] = rint(base + self.cfg.detector.k_on * sigma)
            self.win_off[ch] = rint(base + self.cfg.detector.k_off * sigma)
        self.win_open = True

    def accumulate(self, ch, x):
        amp = x - self.win_base[ch]
        if amp > self.win_max[ch]:
            self.win_max[ch] = amp
        if amp > 0:
            self.win_area[ch] += amp
        if not self.win_rose[ch] and x > self.win_on[ch]:
            self.win_rose[ch] = True
        self.win_high[ch] = x >= self.win_off[ch]

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
                reason = classify_peak(self.cfg.detector, self.win_max[ch],
                                       ref["width_samples"], self.win_area[ch])
            else:
                reason = mc.REASON_MAX_TOO_SMALL
            chans.append({
                "channel": ch, "flags": flags, "reason": reason,
                "baseline": self.win_base[ch], "max_value": self.win_max[ch],
                "width_samples": ref["width_samples"],   # one event, one width
                "width_us": ref["width_us"], "area": self.win_area[ch],
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
            was_in = self.det[ch].state == mc.EVENT
            m = self.det[ch].update(channels[ch], t_us, index)
            if not was_in and self.det[ch].state == mc.EVENT:
                opened |= (1 << ch)
                self.pc_in[ch] = True
            if m is not None:
                closed |= (1 << ch)
                self.pc_metrics[ch] = m
                self.pc_in[ch] = False

        done = False
        if closed != 0:
            for ch in range(MAX_CH):
                if (closed >> ch) & 1:
                    self.pc_done[ch] = True
                    if self.pc_pending > 0:
                        self.pc_pending -= 1
            done = (self.pc_pending == 0)

        rec = self.build_per_channel_record(t_us, index) if done else None

        # A channel that opens on the very sample another group closed starts the
        # next group (pc_pending is 0 again at this point).
        if opened != 0:
            for ch in range(MAX_CH):
                if (opened >> ch) & 1:
                    if self.pc_pending == 0:
                        self.pc_start_us = t_us
                        self.pc_start_ix = index
                    self.pc_pending += 1
        return rec

    def build_per_channel_record(self, t_us, index):
        chans = []
        verdict_ch = 0xFF
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
                cm["width_samples"] = m["width_samples"]   # its own width
                cm["width_us"] = m["width_us"]
                cm["area"] = m["area"]
                cm["flags"] = CH_FLAG_PRESENT
                cm["reason"] = m["reason"]
                if m["reason"] == mc.REASON_WIDTH_TOO_WIDE:
                    cm["flags"] |= CH_FLAG_TRUNCATED
                if verdict_ch == 0xFF or ch == self.primary:
                    verdict_ch = ch
            self.pc_done[ch] = False
            chans.append(cm)

        if verdict_ch != 0xFF:
            valid = 1 if self.pc_metrics[verdict_ch]["reason"] == mc.REASON_OK else 0
            reason = self.pc_metrics[verdict_ch]["reason"]
        else:
            valid, reason = 0, mc.REASON_MAX_TOO_SMALL

        bundle = {
            "start_us": self.pc_start_us, "end_us": t_us,
            "width_samples": index - self.pc_start_ix, "width_us": t_us - self.pc_start_us,
            "primary": verdict_ch, "mode": MODE_PER_CHANNEL,
            "n_channels": len(chans), "valid": valid, "reason": reason,
        }
        self.pc_pending = 0
        return {"bundle": bundle, "ch": chans}

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
        base0 = 2000.0 + gauss() * 5.0
        base1 = 2000.0 + gauss() * 5.0
        base8 = 2000.0 + gauss() * 5.0
        for c in (6000, 13000, 20000):
            base8 += gaussian_peak(i, c, 200, 800.0)      # primary
            base0 += gaussian_peak(i, c, 200, 400.0)      # same shape, half height
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
        for c in (c0, c1, c8):
            if c["width_samples"] != b["width_samples"]:
                failures.append("event %d ch%d: width %d != window %d"
                                % (b["index"], c["channel"], c["width_samples"],
                                   b["width_samples"]))
        if c0["flags"] != CH_FLAG_PRESENT:
            failures.append("event %d ch0: flags 0x%02x, expected PRESENT"
                            % (b["index"], c0["flags"]))
        if not 250 <= c0["max_value"] <= 550:
            failures.append("event %d ch0: height %d, expected ~400"
                            % (b["index"], c0["max_value"]))
        if not 650 <= c8["max_value"] <= 950:
            failures.append("event %d ch8: height %d, expected ~800"
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
        if not 0 <= c1["max_value"] < 80:
            failures.append("event %d ch1: height %d, expected noise only (<80)"
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
        base0 = 2000.0 + gauss() * 5.0
        base1 = 2000.0 + gauss() * 5.0
        base8 = 2000.0 + gauss() * 5.0
        base8 += gaussian_peak(i, 6000, 200, 800.0)   # primary: window 5450..6600
        base0 += gaussian_peak(i, 7000, 200, 800.0)   # arrives after the window closes
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
    if not 0 < c0["max_value"] < 300:
        failures.append("clip: ch0 height %d, expected the window edge (~100)"
                        % c0["max_value"])
    if c0["max_value"] >= c8["max_value"]:
        failures.append("clip: ch0 (%d) not clipped below ch8 (%d)"
                        % (c0["max_value"], c8["max_value"]))
    if c0["width_samples"] != b["width_samples"]:
        failures.append("clip: ch0 width %d != window %d"
                        % (c0["width_samples"], b["width_samples"]))
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
        base9 = 2000.0 + gauss() * 5.0
        for c in (6000, 13000):
            base0 += gaussian_peak(i, c, 200, 800.0)         # broad
            base1 += gaussian_peak(i, c + 100, 60, 600.0)    # narrow, inside ch0
            base8 += gaussian_peak(i, c + 20, 150, 700.0)
            base9 += gaussian_peak(i, c - 30, 100, 500.0)
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
        if b["width_samples"] < c0["width_samples"]:
            failures.append("event %d: window %d narrower than ch0 %d"
                            % (b["index"], b["width_samples"], c0["width_samples"]))
        if not 550 <= c1["max_value"] <= 700:
            failures.append("event %d ch1: height %d, expected ~600"
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
        base0 = 2000.0 + gauss() * 5.0
        base1 = 2000.0 + gauss() * 5.0
        base8 = 2000.0 + gauss() * 5.0
        if 4000 <= i < 4200:                       # bright burst: ch8 saturates
            base8 = 32700.0
        if 8000 <= i < 10000:                      # a normal peak after recovery
            base8 += gaussian_peak(i, 9000, 200, 800.0)
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

    DIP = 800.0

    def levels(i, gauss):
        v = 25000.0 + gauss() * 5.0                 # idles HIGH ...
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

    if len(events) != 2:
        failures.append("invert: %d events, expected 2" % len(events))
    for rec in events:
        b = rec["bundle"]
        c8 = ch_by_id(rec["ch"], 8)
        if b["valid"] != 1:
            failures.append("invert: event %d valid=%d, expected 1 (dip -> peak)"
                            % (b["index"], b["valid"]))
        if c8 is None or not (600 <= c8["max_value"] <= 1000):
            failures.append("invert: event %d ch8 height %s, expected ~%d (dip depth)"
                            % (b["index"], None if c8 is None else c8["max_value"], int(DIP)))

    print("INVERT: %d events (dips detected as peaks)" % len(events))
    for rec in events:
        b = rec["bundle"]
        c8 = ch_by_id(rec["ch"], 8)
        print("  #%d valid=%d ch8 h=%d" % (b["index"], b["valid"], c8["max_value"]))
    return failures


def main():
    failures = (test_linked() + test_linked_clip() + test_per_channel() +
                test_rail() + test_invert())
    if failures:
        print("\nFAIL:")
        for f in failures:
            print("  -", f)
        raise SystemExit(1)
    print("\nPASS: aggregator groups the per-channel detectors as expected in both modes.")


if __name__ == "__main__":
    main()
