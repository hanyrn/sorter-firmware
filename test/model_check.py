#!/usr/bin/env python3
"""
Algorithm model-check for the EventDetector in event_detector.h.

This is a 1:1 mirror of the C++ detector (same EMA statistics, same state
machine, same bounds).  It validates the detection logic and the tuning
constants against a synthetic baseline + noise + peaks + glitches signal when no
C++ compiler is available.  It is not a substitute for test/test_event_detector.cpp
(which exercises the real C++), only a cross-check.

Run:  python test/model_check.py
"""

import math
import random

REASON_OK = 0
REASON_MAX_TOO_SMALL = 1
REASON_WIDTH_TOO_NARROW = 2
REASON_WIDTH_TOO_WIDE = 3
REASON_AREA_TOO_SMALL = 4
REASON_SHORT_FOR_HEIGHT = 5
REASON_RAILED = 6

WARMUP, IDLE, EVENT = 0, 1, 2


class Config:
    def __init__(self):
        self.baseline_alpha = 0.001
        self.noise_alpha = 0.001
        self.k_on = 5.0
        self.k_off = 2.0
        self.k_gate = 3.0
        self.sigma_floor = 1.0
        self.warmup_samples = 3000
        self.end_debounce = 2
        self.min_max_value = 80
        self.min_width_samples = 3
        self.max_width_samples = 200000
        self.min_area = 0
        self.max_slope = 100.0


class EventDetector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.state = WARMUP
        self.samples_seen = 0
        self.mean = 0.0
        self.var = 0.0
        self.sigma = 0.0
        self.base_at_start = 0
        self.sigma_at_start = 0.0
        self.ev_start_us = 0
        self.ev_start_index = 0
        self.ev_samples = 0
        self.ev_peak_us = 0
        self.ev_max = 0
        self.ev_area = 0
        self.below_count = 0
        self.counter = 0
        self.primed = False

    def rebaseline(self, x):
        """Mirror of EventDetector::rebaseline(): re-seed the mean from the current
        sample and drop any event in progress, without re-running warm-up.  The
        noise estimate (var/sigma) is kept - saturating the input does not change
        the channel's noise floor."""
        self.mean = float(x)
        self.primed = True
        self.state = IDLE
        self.base_at_start = x
        self.sigma_at_start = self.sigma
        self.ev_start_us = 0
        self.ev_start_index = 0
        self.ev_samples = 0
        self.ev_peak_us = 0
        self.ev_max = 0
        self.ev_area = 0
        self.below_count = 0
        if self.samples_seen < self.cfg.warmup_samples:
            self.samples_seen = self.cfg.warmup_samples

    def _update_baseline(self, x):
        if not self.primed:
            self.mean = x
            self.var = 0.0
            self.primed = True
        else:
            dev = x - self.mean
            self.mean += self.cfg.baseline_alpha * dev
            self.var += self.cfg.noise_alpha * (dev * dev - self.var)
        if self.var < 0.0:
            self.var = 0.0
        self.sigma = math.sqrt(self.var)
        if self.sigma < self.cfg.sigma_floor:
            self.sigma = self.cfg.sigma_floor

    def update(self, x, t_us, index):
        self.samples_seen += 1

        if self.state == WARMUP:
            self._update_baseline(x)
            if self.samples_seen >= self.cfg.warmup_samples:
                self.state = IDLE
            return None

        if self.state == IDLE:
            if abs(x - self.mean) <= self.cfg.k_gate * self.sigma:
                self._update_baseline(x)
            thr = round(self.mean + self.cfg.k_on * self.sigma)
            if x > thr:
                self.state = EVENT
                self.base_at_start = round(self.mean)
                self.sigma_at_start = self.sigma
                self.ev_start_us = t_us
                self.ev_start_index = index
                self.ev_samples = 0
                self.ev_peak_us = t_us
                self.ev_max = x - self.base_at_start
                self.ev_area = self.ev_max if self.ev_max > 0 else 0
                self.below_count = 0
            return None

        amp = x - self.base_at_start
        if amp > self.ev_max:
            self.ev_max = amp
            self.ev_peak_us = t_us
        if amp > 0:
            self.ev_area += amp
        self.ev_samples += 1

        ret = round(self.base_at_start + self.cfg.k_off * self.sigma_at_start)
        if x < ret:
            self.below_count += 1
        else:
            self.below_count = 0

        if self.below_count >= self.cfg.end_debounce or \
           (index - self.ev_start_index) >= self.cfg.max_width_samples:
            return self._close(t_us, index)
        return None

    def _close(self, t_us, index):
        width_samples = index - self.ev_start_index
        m = {
            "index": self.counter,
            "start_us": self.ev_start_us,
            "peak_us": self.ev_peak_us,
            "width_us": t_us - self.ev_start_us,
            "width_samples": width_samples,
            "samples": self.ev_samples,
            "baseline": self.base_at_start,
            "noise": round(self.sigma_at_start),
            "threshold": round(self.base_at_start + self.cfg.k_on * self.sigma_at_start),
            "max_value": self.ev_max,
            "max_abs_value": self.base_at_start + self.ev_max,
            "area": self.ev_area,
            "valid": 1,
            "reason": REASON_OK,
        }
        self.counter += 1
        if self.ev_max < self.cfg.min_max_value:
            m["valid"], m["reason"] = 0, REASON_MAX_TOO_SMALL
        elif width_samples < self.cfg.min_width_samples:
            m["valid"], m["reason"] = 0, REASON_WIDTH_TOO_NARROW
        elif width_samples > self.cfg.max_width_samples:
            m["valid"], m["reason"] = 0, REASON_WIDTH_TOO_WIDE
        elif self.ev_area < self.cfg.min_area:
            m["valid"], m["reason"] = 0, REASON_AREA_TOO_SMALL
        else:
            slope = self.ev_max / (width_samples if width_samples else 1)
            if slope > self.cfg.max_slope:
                m["valid"], m["reason"] = 0, REASON_SHORT_FOR_HEIGHT
        self.state = IDLE
        return m

def build_signal(n, dt_us):
    """baseline 2000 +/- 5, three broad peaks, a 1-sample glitch and a 4-sample glitch."""
    random.seed(1234)
    baseline = 2000.0
    sigma = 5.0
    peaks = [(6000, 200, 800.0), (13000, 200, 800.0), (20000, 200, 800.0)]
    glitch1 = 9000
    glitch2_start = 16000
    samples = []
    for i in range(n):
        x = baseline + random.gauss(0.0, sigma)
        for (c, w, amp) in peaks:
            x += amp * math.exp(-((i - c) ** 2) / (2.0 * w * w))
        if i == glitch1:
            x += 900.0
        if glitch2_start <= i < glitch2_start + 4:
            x += 900.0
        samples.append((int(round(x)), i * dt_us))
    return samples


def main():
    det = EventDetector(Config())
    events = []
    for idx, (x, t) in enumerate(build_signal(26000, 10)):
        m = det.update(x, t, idx)
        if m is not None:
            events.append(m)

    print("closed events:", len(events))
    for m in events:
        print("  #%d valid=%d reason=%d max=%d width=%dsmp (%dus) area=%d"
              % (m["index"], m["valid"], m["reason"], m["max_value"],
                 m["width_samples"], m["width_us"], m["area"]))

    valid = [m for m in events if m["valid"] == 1]
    invalid = [m for m in events if m["valid"] == 0]

    failures = []
    if len(valid) != 3:
        failures.append("expected 3 valid peaks, got %d" % len(valid))
    if len(invalid) != 2:
        failures.append("expected 2 rejected glitches, got %d" % len(invalid))
    for m in invalid:
        if m["reason"] not in (REASON_WIDTH_TOO_NARROW, REASON_SHORT_FOR_HEIGHT):
            failures.append("unexpected rejection reason %d" % m["reason"])
    for m in valid:
        if not (650 <= m["max_value"] <= 950):
            failures.append("peak amplitude out of range: %d" % m["max_value"])
        if not (600 <= m["width_samples"] <= 1700):
            failures.append("peak width out of range: %d" % m["width_samples"])
    if abs(det.mean - 2000.0) > 25.0:
        failures.append("baseline did not track to ~2000 (got %.1f)" % det.mean)

    print("baseline after run: %.1f  sigma: %.2f" % (det.mean, det.sigma))
    if failures:
        print("\nFAIL:")
        for f in failures:
            print("  -", f)
        raise SystemExit(1)
    print("\nPASS: detector reproduces the expected peaks and rejects the glitches.")


if __name__ == "__main__":
    main()
