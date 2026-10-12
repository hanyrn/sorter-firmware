#!/usr/bin/env python3
"""
Algorithm model-check for the EventDetector in event_detector.h.

Run test/test_event_detector.cpp too; it exercises the real C++ and is authoritative
where the two disagree.

`width_samples` is the event's *window* (band breakout -> return to the band) - the number
the shaded box in test/plot_signals.py is drawn from - and `duration_samples` is the
number the firmware judges DET_MIN_WIDTH_SAMPLES against and reports as the channel's
width (the window widened for the pulse's own envelope; see
EventDetector::envelopeDuration and config.h's DET_MIN_WIDTH_SAMPLES note).  Both are
reported here so the plot can show the window and the number the board would send.

This is a 1:1 mirror of the C++ detector (same EMA statistics, same state
machine, same window rule, same bounds).  It validates the detection logic and the
tuning constants against a synthetic baseline + noise + peaks + glitches signal when no
C++ compiler is available.  It is not a substitute for test/test_event_detector.cpp
(which exercises the real C++), only a cross-check.

REWORK NOTE: the C++ detector is a four-phase machine (quiet period -> idle -> rising
candidate -> event) with a robust median/MAD seed, a rise test with dip tolerance,
suppression of noise events, a slope-bounded measurement path and an apex-settle rule.
This mirror implements the *window* rules in full - the band breakout that opens a window
and starts the event, the dip tolerance that drops it again, the duration read back at the
band, the bounds and the shape guards - but not the four phases themselves: it has no
rising state, no bounded path and no suppression (DET_DISCARDED).  The reference, though,
is the firmware's, and here it matters as much as the window: the quiet period ESTABLISHES
it exactly as EventDetector::primeFromSeed() does - the median of DET_QUIET_SEED_POINTS
samples spread over DET_QUIET_SAMPLES, with sigma from their MAD x 1.4826 - and every later
update is gated, up to DET_GATE_MAX counts.  That is the number the floor ladder reads: a
reference that followed the signals instead of the noise would put the trigger above the
very rungs this file is used to judge (50 % contamination is what the median/MAD pair
tolerates, and the ladder's two faintest rungs sit inside every quiet period).  Its window,
its duration and its verdicts therefore are the firmware's, while its event *counts* on a
noisy stream can still differ: test/test_event_detector.cpp and test/test_bench_detect.cpp
are authoritative where the two disagree.

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


def rint(f):
    """C's lroundf(): round half away from zero (Python's round() rounds half to
    even, which would quietly disagree with the firmware at exact .5 values)."""
    return int(math.floor(f + 0.5)) if f >= 0 else -int(math.floor(-f + 0.5))


def envelope_duration(max_value, threshold, width_samples):
    """Mirror of EventDetector::envelopeDuration() in event_detector.h: the window's
    span, widened for the pulse's own envelope.

    The signal is a rounded pulse - a raised cosine of duration W whose flanks reach
    the baseline at its ends - so the span it spends above the level `threshold` is
    W * acos(2 * threshold / A - 1) / pi (A = max_value, the apex above the baseline).
    Inverting that turns the measured span back into the event's duration, which is
    what DET_MIN_WIDTH_SAMPLES judges; `threshold` is the level the span was actually
    measured at (k_off * sigma - both ends of a window sit on that level).
    """
    if width_samples == 0 or max_value <= 0 or threshold <= 0:
        return width_samples
    if max_value > threshold:
        r = 2.0 * threshold / max_value - 1.0
        factor = (math.pi / math.acos(r)) if (-1.0 < r < 1.0) else 1.0
        if factor > 2.0:
            factor = 2.0
    else:
        # The level is at or above the apex: there is no flank left to read the
        # length from, so twice the span is the documented cap.
        factor = 2.0
    if factor <= 1.0:
        return width_samples
    return rint(width_samples * factor)


class Config:
    def __init__(self):
        self.baseline_alpha = 0.001
        self.noise_alpha = 0.001
        self.k_on = 5.0
        self.k_off = 2.0
        self.k_gate = 3.0
        self.sigma_floor = 1.0
        # The quiet period: detection is disarmed while it runs and the reference is
        # established from it, so it is what the whole ladder in this file measures
        # against.  3000 samples = 0.3 s at 10 kSPS.  Mirrors DET_QUIET_SAMPLES.
        self.quiet_samples = 3000
        # How many of those samples seed the reference, spread over the WHOLE period
        # (see _seed_every): the period only means something if its points cover it.
        # Mirrors DET_QUIET_SEED_POINTS (bounded by kQuietSeedMax = 256).
        self.quiet_seed_points = 128
        # Ceiling on the reference-update gate in counts, so a sigma that grew large can
        # never widen the gate enough to admit the signal it is supposed to reject.
        # Mirrors DET_GATE_MAX (0 = no ceiling).
        self.gate_max = 320
        self.end_debounce = 2
        # Input stage: length of the running median in front of the detector
        # (1 = off, 3 = drop a 1-sample spike, 5 = up to 2, 7 = up to 3, 9 = up to 4).
        # 7 because the spike has to be caught after the analog path: a 1-sample
        # impulse comes back from the AD7606 spread over 2-4 samples, which walks
        # straight through a 3-tap median.  Mirrors DET_IMPULSE_FILTER_TAPS.
        self.impulse_filter_taps = 7
        # A rise candidate may dip back into the noise band for up to this many
        # CONSECUTIVE filtered samples and still be a rise.  The same rule keeps a
        # window's opening edge alive (see EventDetector.update's IDLE case).
        # Mirrors DET_DIP_TOLERANCE.
        self.dip_tolerance = 3
        # Bounds, mirroring config.h's DET_* values (see the notes there for why these
        # numbers and not others).
        self.min_max_value = 160       # DET_MIN_MAX_VALUE
        self.min_width_samples = 35    # DET_MIN_WIDTH_SAMPLES, judged on the duration
        self.max_width_samples = 20000  # DET_MAX_WIDTH_SAMPLES: the fault guard, not a filter
        self.min_area = 0
        self.min_rise_samples = 20     # DET_MIN_RISE_SAMPLES: start -> maximum
        self.max_slope = 200.0         # counts/sample the leading edge may climb


class ImpulseFilter:
    """Mirror of ImpulseFilter in event_detector.h: a running median of the last
    `taps` raw samples, so a lone noise spike can neither start, end nor top an
    event.  The window is filled with the first sample, so the filter never emits a
    value it has no history for."""

    def __init__(self, taps=7):
        self.configure(taps)

    def configure(self, taps):
        if taps < 1:
            self.taps = 1
        elif taps > 9:
            self.taps = 9
        else:
            self.taps = taps | 1
        self.reset()

    def reset(self):
        self.ring = [0] * 9
        self.head = 0
        self.count = 0
        self.primed = False
        self.value = 0

    def step(self, x):
        if self.taps <= 1:
            self.value = x
            return x
        if not self.primed:
            for i in range(self.taps):
                self.ring[i] = x
            self.primed = True
        else:
            self.ring[self.head] = x
            self.head = (self.head + 1) % self.taps
        if self.count < self.taps:
            self.count += 1
        w = sorted(self.ring[:self.count])
        self.value = w[(self.count - 1) // 2]     # lower median, like the C++ mirror
        return self.value


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
        self.ev_peak_index = 0
        self.ev_max = 0
        self.ev_area = 0
        self.below_count = 0
        self.below_start_us = 0
        self.below_start_index = 0
        # The window's opening edge: the sample the trace left the noise band on.
        # A window is opened by the trigger crossing but *starts* here (see update()),
        # and it is the event's start - what inEvent() reports to the aggregator.
        self.pend_valid = False
        self.pend_us = 0
        self.pend_index = 0
        self.pend_area = 0
        self.pend_samples = 0
        self.counter = 0
        self.primed = False
        self.filtered = 0
        # The quiet period's seed (mirrors seed_/seed_n_/seed_skip_ in event_detector.h):
        # the reference is established from these points, once, when the period ends.
        self.seed = []
        self.seed_skip = 0
        self.filt = ImpulseFilter(self.cfg.impulse_filter_taps)

    def rebaseline(self, x):
        """Forget the event in progress and re-point the mean at the current sample,
        without touching the noise estimate (var/sigma): saturating the input does not
        change the channel's noise floor.  The impulse filter is reset with it (its
        window still holds clipped samples) and the sample is used as it is.

        NOT a mirror of the firmware: this is the *harness's* recovery, used by
        test/model_check_aggregator.py to resume after a rail.  The firmware discards the
        whole reference instead and re-establishes it from a fresh quiet period
        (EventDetector::restartQuietPeriod) - detection there stays blind for
        DET_QUIET_SAMPLES samples after the call, because a stale reference is exactly
        what the quiet period exists to avoid.
        """
        self.filt.reset()
        self.filtered = x
        self.mean = float(x)
        self.primed = True
        self.state = IDLE
        self.base_at_start = x
        self.sigma_at_start = self.sigma
        self.ev_start_us = 0
        self.ev_start_index = 0
        self.ev_samples = 0
        self.ev_peak_us = 0
        self.ev_peak_index = 0
        self.ev_max = 0
        self.ev_area = 0
        self.below_count = 0
        self.below_start_us = 0
        self.below_start_index = 0
        self.pend_valid = False
        self.pend_us = 0
        self.pend_index = 0
        self.pend_area = 0
        self.pend_samples = 0
        if self.samples_seen < self.cfg.quiet_samples:
            self.samples_seen = self.cfg.quiet_samples

    def _gate_width(self):
        """Mirror of EventDetector::gateWidth(): the reference-update gate is relative to
        the current sigma, but capped in absolute counts, so a sigma that (accidentally)
        grew large can never widen the gate enough to admit the signal it is meant to
        reject."""
        g = self.cfg.k_gate * self.sigma
        if self.cfg.gate_max and g > self.cfg.gate_max:
            return float(self.cfg.gate_max)
        return g

    def _update_baseline(self, x, update_noise=True):
        """Mirror of EventDetector::updateBaseline(): maintain the reference from one
        sample, but only while that sample is inside the gate.  The gate is what makes
        the reference stable: a rising peak is outside it (the trigger sits at
        k_on > k_gate sigma) and cannot drag the baseline up, and one outlier moves the
        mean by a fraction of a count at most.  Called from IDLE only - the reference is
        ESTABLISHED by the quiet period (see _prime_from_seed), never maintained while a
        candidate or an event is open.

        `update_noise` is false while a window is open: the gate alone does not protect
        sigma there, because a faint peak's whole lower flank is *inside* the gate, so it
        would feed its own rise into the estimate and the trigger (5 sigma) would climb
        with it until the peak stayed under it.  An excursion may move the mean - the mean
        is what tracks the slow wander - but never sigma."""
        if abs(x - self.mean) > self._gate_width():
            return
        dev = x - self.mean
        self.mean += self.cfg.baseline_alpha * dev
        if not update_noise:
            return                        # an excursion may move the mean, never sigma
        self.var += self.cfg.noise_alpha * (dev * dev - self.var)
        if self.var < 0.0:
            self.var = 0.0
        self.sigma = math.sqrt(self.var)
        if self.sigma < self.cfg.sigma_floor:
            self.sigma = self.cfg.sigma_floor

    # ---- the quiet period -------------------------------------------------------
    # How many points the quiet period seeds from (bounded by kQuietSeedMax), and how
    # many samples apart they are taken: the points have to be spread over the WHOLE
    # period for the period to mean anything (see Config::quiet_seed_points).
    def _seed_points(self):
        n = int(self.cfg.quiet_seed_points)
        if n < 1:
            n = 1
        if n > 256:                       # kQuietSeedMax
            n = 256
        return n

    def _seed_every(self):
        pts = self._seed_points()
        return (self.cfg.quiet_samples // pts) if self.cfg.quiet_samples > pts else 1

    def _collect_seed(self, x):
        """Mirror of EventDetector::collectSeed(): collect one seed point every
        _seed_every() samples (the first sample of the period is always taken, so even a
        pathological quiet_samples of 0 leaves _prime_from_seed something to work with).
        The samples that are NOT collected are used for nothing at all - the quiet period
        is not measured, it establishes the reference."""
        if len(self.seed) >= self._seed_points():
            return                        # the whole period is covered
        if not self.seed:
            self.seed.append(x)           # always take the first
            return
        every = self._seed_every()
        if every > 1:
            self.seed_skip += 1
            if self.seed_skip < every:
                return
        self.seed_skip = 0
        self.seed.append(x)

    def _prime_from_seed(self):
        """Mirror of EventDetector::primeFromSeed(): the reference is the *noise*, so
        both parts of it are estimated robustly from the quiet period's seed points - the
        mean is their median and sigma is their MAD (median |x - median|) scaled by
        1.4826, the standard robust estimator for a normal distribution.

        A median/MAD pair tolerates up to 50 % contamination, which is what makes a quiet
        period usable even when a pulse (or a chunk of one) lands inside it - and pulses
        do: the floor ladder's two faintest rungs are drawn inside the period on purpose.
        Seeding from an EMA (or a mean) instead lets those rungs decide sigma, and a sigma
        a few counts too high is the difference between taking the 120-count rung and
        leaving it silent (see the ladder table in config.h)."""
        w = sorted(self.seed) if self.seed else [rint(self.mean)]
        n = len(w)
        med = w[n // 2]
        d = sorted(abs(v - med) for v in w)
        spread = 1.4826 * float(d[n // 2])
        self.mean = float(med)
        self.var = spread * spread
        self.sigma = spread if spread >= self.cfg.sigma_floor else float(self.cfg.sigma_floor)
        self.primed = True

    def update(self, x, t_us, index):
        self.samples_seen += 1

        # Input conditioning: everything below works on the impulse-filtered sample,
        # never on the raw one (mirrors ImpulseFilter in event_detector.h).
        x = self.filt.step(x)
        self.filtered = x

        if self.state == WARMUP:
            # The quiet period: detection is disarmed and the reference is ESTABLISHED
            # from what is seen here (see _collect_seed/_prime_from_seed).  It is seeded
            # exactly once, on the sample that ends the period, so the detector is always
            # primed before it is armed.
            self._collect_seed(x)
            if self.samples_seen >= self.cfg.quiet_samples:
                self._prime_from_seed()
                self.state = IDLE
            return None

        if self.state == IDLE:
            # Refresh the reference only while the trace is INSIDE the noise band: the gate
            # inside _update_baseline() caps how far a sample may sit from the baseline,
            # but a faint peak's whole lower flank is inside that gate (the band opens at
            # k_off < k_gate sigma), so a peak that never leaves the band would feed its
            # own rise into sigma and the trigger (5 sigma) would climb with it - that is
            # the floor the ladder walks (measured: sigma 27.0 -> 38.4 counts across one
            # 120-count rung at 600 samples).  Once the window opens the excursion is the
            # signal and must not decide the *noise*; the mean keeps tracking (it is what
            # follows the slow wander).  (One sample of lag, deliberately: pend_valid is
            # what the *previous* sample left behind.)
            self._update_baseline(x, not self.pend_valid)
            # Both levels are taken off the rounded baseline, i.e. exactly the value the
            # event latches as base_at_start and reports as threshold= (see
            # event_detector.h).
            base = rint(self.mean)
            thr  = rint(base + self.cfg.k_on * self.sigma)
            # The trigger crossing starts the *candidate*, not the event: where the event
            # begins is decided below (the window), and the crossing sample is measured
            # there - so it is checked first, and the window's own bookkeeping stops short
            # of it.  One sample, counted once.
            if x > thr:
                self.state = EVENT
                self.base_at_start = base
                self.sigma_at_start = self.sigma
                # The event starts where the trace left the noise band - the window the
                # samples before this one opened; a crossing that came without one (the
                # trace jumped straight through both levels) starts the event here.
                self.ev_start_us = self.pend_us if self.pend_valid else t_us
                self.ev_start_index = self.pend_index if self.pend_valid else index
                self.ev_samples = self.pend_samples if self.pend_valid else 0
                self.ev_peak_us = t_us
                self.ev_peak_index = index
                self.ev_max = x - self.base_at_start
                self.ev_area = self.pend_area if self.pend_valid else 0
                # ... and the crossing sample itself is measured here, once.
                self.ev_samples += 1
                if self.ev_max > 0:
                    self.ev_area += self.ev_max
                self.below_count = 0
                return None

            # The window of the next event opens where the trace leaves the noise band
            # (+k_off * sigma), not where it crosses the trigger (+k_on * sigma): the
            # trigger is what decides the excursion is worth reporting, while the window
            # *is* the excursion - and both of its ends then sit on the same level, which
            # is also the level the duration is read back at (see envelope_duration).  The
            # onset is remembered - together with the flank area and sample count in front
            # of the crossing - while the trace stays out of the band, and it is dropped
            # once the trace has been back inside the band for more than dip_tolerance
            # consecutive samples.  That is the same rule the rise test applies, and it is
            # why a band crossing on its own cannot open a window.
            band = rint(base + self.cfg.k_off * self.sigma)
            if x > band:
                if not self.pend_valid:
                    self.pend_valid = True
                    self.pend_us = t_us
                    self.pend_index = index
                    self.pend_area = 0
                    self.pend_samples = 0
                amp = x - base
                if amp > 0:
                    self.pend_area += amp
                self.pend_samples += 1
                self.below_count = 0
            else:
                self.below_count += 1
                if self.below_count > self.cfg.dip_tolerance:
                    self.pend_valid = False
            return None

        amp = x - self.base_at_start
        if amp > self.ev_max:
            self.ev_max = amp
            self.ev_peak_us = t_us
            self.ev_peak_index = index
        if amp > 0:
            self.ev_area += amp
        self.ev_samples += 1

        ret = rint(self.base_at_start + self.cfg.k_off * self.sigma_at_start)
        if x < ret:
            if self.below_count == 0:
                # Where the run below the band began: that sample is where the event ends,
                # so the end debounce cannot lengthen the reported width (see _close).
                self.below_start_us = t_us
                self.below_start_index = index
            self.below_count += 1
        else:
            self.below_count = 0

        if self.below_count >= self.cfg.end_debounce:
            return self._close(self.below_start_us, self.below_start_index)
        if (index - self.ev_start_index) >= self.cfg.max_width_samples:
            return self._close(t_us, index)
        return None

    def in_event(self):
        """Mirror of EventDetector::inEvent(): an event or a rise candidate is in
        progress, or the window the event will start from is open - i.e. from the sample
        the trace left the noise band to the sample the event closed on (see update()).
        With the two-phase mirror above, "an event in progress" is the EVENT state; the
        window that is open before it is `pend_valid`."""
        return self.state == EVENT or self.pend_valid

    def _close(self, t_us, index):
        width_samples = index - self.ev_start_index
        rise = self.ev_peak_index - self.ev_start_index
        # The number the bounds are judged with, and the number the link carries as the
        # channel's width: the window's span read back through the pulse's envelope.
        duration = envelope_duration(
            self.ev_max, rint(self.cfg.k_off * self.sigma_at_start), width_samples)
        m = {
            "index": self.counter,
            "start_us": self.ev_start_us,
            "peak_us": self.ev_peak_us,
            "width_us": t_us - self.ev_start_us,
            "width_samples": width_samples,
            "duration_samples": duration,
            "duration_us": ((t_us - self.ev_start_us) * duration // width_samples
                            if width_samples else 0),
            "samples": self.ev_samples,
            "rise_samples": rise,
            "baseline": self.base_at_start,
            "noise": rint(self.sigma_at_start),
            "threshold": rint(self.base_at_start + self.cfg.k_on * self.sigma_at_start),
            "max_value": self.ev_max,
            "max_abs_value": self.base_at_start + self.ev_max,
            "area": self.ev_area,
            "valid": 1,
            "reason": REASON_OK,
        }
        self.counter += 1
        # The same ladder as EventDetector::classifyPeak(): the bounds, then the two
        # shape guards - the leading edge has to be long enough (a step is not a peak)
        # and must not climb faster than max_slope counts/sample.  The width bound is
        # judged on the DURATION (see the note at the top of this file).
        if self.ev_max < self.cfg.min_max_value:
            m["valid"], m["reason"] = 0, REASON_MAX_TOO_SMALL
        elif duration < self.cfg.min_width_samples:
            m["valid"], m["reason"] = 0, REASON_WIDTH_TOO_NARROW
        elif duration > self.cfg.max_width_samples:
            m["valid"], m["reason"] = 0, REASON_WIDTH_TOO_WIDE
        elif self.ev_area < self.cfg.min_area:
            m["valid"], m["reason"] = 0, REASON_AREA_TOO_SMALL
        else:
            edge = rise if rise else 1          # 0 = a vertical edge
            if edge < self.cfg.min_rise_samples or \
               self.ev_max / edge > self.cfg.max_slope:
                m["valid"], m["reason"] = 0, REASON_SHORT_FOR_HEIGHT
        self.state = IDLE
        self.pend_valid = False
        return m

def build_signal(n, dt_us):
    """baseline 2000 +/- 10, three broad peaks, a 1-sample glitch and a 4-sample glitch."""
    random.seed(1234)
    baseline = 2000.0
    sigma = 10.0
    peaks = [(6000, 200, 1600.0), (13000, 200, 1600.0), (20000, 200, 1600.0)]
    glitch1 = 9000
    glitch2_start = 16000
    samples = []
    for i in range(n):
        x = baseline + random.gauss(0.0, sigma)
        for (c, w, amp) in peaks:
            x += amp * math.exp(-((i - c) ** 2) / (2.0 * w * w))
        if i == glitch1:
            x += 1800.0
        if glitch2_start <= i < glitch2_start + 4:
            x += 1800.0
        samples.append((int(round(x)), i * dt_us))
    return samples


def main():
    cfg = Config()
    det = EventDetector(cfg)
    events = []
    for idx, (x, t) in enumerate(build_signal(26000, 10)):
        m = det.update(x, t, idx)
        if m is not None:
            events.append(m)

    print("closed events:", len(events))
    for m in events:
        print("  #%d valid=%d reason=%d max=%d width=%dsmp (%dus) rise=%dsmp area=%d"
              % (m["index"], m["valid"], m["reason"], m["max_value"],
                 m["width_samples"], m["width_us"], m["rise_samples"], m["area"]))

    valid = [m for m in events if m["valid"] == 1]
    invalid = [m for m in events if m["valid"] == 0]

    failures = []
    if len(valid) != 3:
        failures.append("expected 3 valid peaks, got %d" % len(valid))
    # The 1-sample glitch is removed by the impulse filter before it can become an
    # event at all; only the 4-sample one reaches the ladder, where the shape guard
    # (a step rises instantly) rejects it.
    #
    # This mirror has no DET_DISCARDED (see the REWORK NOTE at the top), so a dither
    # excursion the firmware would suppress outright can additionally appear here as a
    # `rejected MAX_TOO_SMALL` record.  Those are tolerated - the glitch above is the
    # event under test, and it is judged by the two checks below - but nothing else is.
    glitches = [m for m in invalid if m["reason"] != REASON_MAX_TOO_SMALL]
    if len(glitches) != 1:
        failures.append("expected 1 rejected glitch, got %d" % len(glitches))
    for m in glitches:
        if m["reason"] not in (REASON_WIDTH_TOO_NARROW, REASON_SHORT_FOR_HEIGHT):
            failures.append("unexpected rejection reason %d" % m["reason"])
        if m["rise_samples"] > 2:
            failures.append("a flat glitch was reported with a rise of %d samples"
                            % m["rise_samples"])
    for m in invalid:
        if m["reason"] == REASON_MAX_TOO_SMALL and m["max_value"] >= cfg.min_max_value:
            failures.append("a suppressed excursion reported max %d, above %d"
                            % (m["max_value"], cfg.min_max_value))
    for m in valid:
        if not (1300 <= m["max_value"] <= 1900):
            failures.append("peak amplitude out of range: %d" % m["max_value"])
        if not (600 <= m["width_samples"] <= 1700):
            failures.append("peak width out of range: %d" % m["width_samples"])
        if m["rise_samples"] < 300:
            failures.append("a real peak reported a %d-sample leading edge"
                            % m["rise_samples"])
    if abs(det.mean - 2000.0) > 25.0:
        failures.append("baseline did not track to ~2000 (got %.1f)" % det.mean)

    print("baseline after run: %.1f  sigma: %.2f" % (det.mean, det.sigma))
    if failures:
        print("\nFAIL:")
        for f in failures:
            print("  -", f)
        raise SystemExit(1)
    print("\nPASS: detector reproduces the expected peaks, the impulse never becomes an")
    print("      event, and the flat glitch is rejected with a step-shaped edge.")


if __name__ == "__main__":
    main()
