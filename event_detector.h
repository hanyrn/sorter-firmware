// event_detector.h
//
// Hardware-independent event ("peak") detector.
//
// Signal model
// ------------
//   * It runs on ONE channel: the signal sits at a low, slowly varying baseline
//     with Gaussian noise fluctuations on top of it.  EventAggregator owns one
//     detector per AD7606 channel and assembles the per-channel results into a
//     single event bundle.
//   * An event starts when the signal rises above  baseline + k_on * sigma.
//   * The event reaches a maximum and then decays back into the noise band; it
//     is closed when the signal falls back below  baseline + k_off * sigma
//     (k_off < k_on => hysteresis) for a couple of consecutive samples.
//
// What is tracked
// ---------------
//   * baseline : running mean (EMA), updated ONLY while no event is in progress.
//   * noise    : standard deviation (EMA of the variance), likewise frozen
//                during an event so a long peak cannot corrupt the reference.
//
// Metrics per closed event (see PeakMetrics): maximum value, area, width.
// False-peak rejection uses the bounds in Config (amplitude, width, area, and
// the "width too short for its height" shape test).
//
// This file has no Arduino dependency and is therefore unit-testable on a host.

#pragma once

#include <stdint.h>
#include <math.h>
#include "peak_metrics.h"

class EventDetector {
public:
    struct Config {
        // ---- baseline / noise tracking (idle only) ----
        // Slow EMA weights: the tracker must be much slower than an event so a
        // rising/falling peak cannot drag the reference with it.
        float    baseline_alpha     = 0.001f;  // EMA weight for the running mean
        float    noise_alpha        = 0.001f;  // EMA weight for the running variance
        float    k_on               = 5.0f;    // start:  x > baseline + k_on  * sigma
        float    k_off              = 2.0f;    // end:    x < baseline + k_off * sigma
        float    k_gate             = 3.0f;    // reference-update gate: only samples
                                               // within k_gate*sigma of the baseline
                                               // refresh the statistics, so a rising
                                               // peak cannot drag them upward
        uint32_t sigma_floor        = 1;       // minimum sigma (counts) to avoid a zero threshold
        uint32_t warmup_samples     = 3000;    // samples before detection is armed
        uint32_t end_debounce       = 2;       // consecutive "below" samples to close an event

        // ---- metric bounds used to invalidate false peaks ----
        int32_t  min_max_value      = 80;      // peak amplitude (counts) must reach this
        uint32_t min_width_samples  = 3;       // width must be at least this many samples
        uint32_t max_width_samples  = 200000;  // width must be at most this (timeout guard)
        uint64_t min_area           = 0;       // integrated area must reach this
        float    max_slope          = 100.0f;  // amplitude / width_samples must not exceed this
    };

    EventDetector() { reset(); }
    explicit EventDetector(const Config& c) : cfg_(c) { reset(); }

    // Feed one sample of the channel this detector is tracking.
    //   x     : channel sample value (AD7606 counts)
    //   t_us  : timestamp in microseconds
    //   index : monotonically increasing sample index (used for width in samples)
    //   out   : filled when this call closes an event
    // Returns true exactly when an event has just been closed.
    bool update(int32_t x, uint32_t t_us, uint32_t index, PeakMetrics& out) {
        samples_seen_++;
        switch (state_) {
            case STATE_WARMUP:
                updateBaseline((float)x);
                if (samples_seen_ >= cfg_.warmup_samples) { state_ = STATE_IDLE; }
                return false;

            case STATE_IDLE: {
                // Refresh the reference only while inside the noise band.
                // Otherwise a slowly rising peak would drag the baseline/sigma up
                // with it and the threshold would never be crossed.
                if (fabsf((float)x - mean_) <= cfg_.k_gate * sigma_) {
                    updateBaseline((float)x);
                }
                const int32_t thr = (int32_t)lroundf(mean_ + cfg_.k_on * sigma_);
                if (x > thr) { startEvent(x, t_us, index); }
                return false;
            }

            case STATE_EVENT: {
                const int32_t amp = x - base_at_start_;
                if (amp > ev_max_) { ev_max_ = amp; ev_peak_us_ = t_us; }
                if (amp > 0) { ev_area_ += (uint64_t)amp; }
                ev_samples_++;

                const int32_t ret = (int32_t)lroundf(base_at_start_ + cfg_.k_off * sigma_at_start_);
                if (x < ret) {
                    if (++below_count_ >= cfg_.end_debounce) { closeEvent(t_us, index, out); return true; }
                } else {
                    below_count_ = 0;
                }
                if ((index - ev_start_index_) >= cfg_.max_width_samples) { closeEvent(t_us, index, out); return true; }
                return false;
            }
        }
        return false;
    }

    void reset() {
        state_          = STATE_WARMUP;
        samples_seen_   = 0;
        mean_           = 0.0f;
        var_            = 0.0f;
        sigma_          = 0.0f;
        base_at_start_  = 0;
        sigma_at_start_ = 0.0f;
        ev_start_us_    = 0;
        ev_start_index_ = 0;
        ev_samples_     = 0;
        ev_peak_us_     = 0;
        ev_max_         = 0;
        ev_area_        = 0;
        below_count_    = 0;
        event_counter_  = 0;
        primed_         = false;
    }

    void configure(const Config& c) { cfg_ = c; reset(); }

    // Re-seed the running mean from the current sample and drop any event in
    // progress, so the detector resumes immediately (without re-running warm-up).
    // Used when a channel has been saturated (railed): its readings were
    // meaningless, so the stale reference is discarded instead of being allowed to
    // bias the next event.
    //
    // The noise estimate (var_/sigma_) is deliberately KEPT: saturating the input
    // does not change the channel's noise floor, and a freshly seeded (too small)
    // sigma would put the threshold far too low and make the detector chatter on
    // noise until the slow EMA re-converged. `samples_seen_` is left at (or pushed
    // to) the warm-up count so the detector does not fall back to STATE_WARMUP.
    void rebaseline(int32_t x) {
        mean_           = (float)x;
        primed_         = true;
        state_          = STATE_IDLE;
        base_at_start_  = x;
        sigma_at_start_ = sigma_;
        ev_start_us_    = 0;
        ev_start_index_ = 0;
        ev_samples_     = 0;
        ev_peak_us_     = 0;
        ev_max_         = 0;
        ev_area_        = 0;
        below_count_    = 0;
        if (samples_seen_ < cfg_.warmup_samples) { samples_seen_ = cfg_.warmup_samples; }
    }

    const Config& config() const { return cfg_; }
    Config&       config()       { return cfg_; }

    bool  inEvent()  const { return state_ == STATE_EVENT; }
    float baseline() const { return mean_; }
    float sigma()    const { return sigma_; }

    // The rejection ladder used to judge a closed peak. Exposed as a static so
    // EventAggregator can judge metrics it measured itself (a channel's height /
    // area inside another channel's window) with exactly the same rules.
    // Returns REASON_OK when the peak satisfies every bound.
    static uint8_t classifyPeak(const Config& cfg, int32_t max_value,
                                uint32_t width_samples, uint64_t area) {
        if (max_value < cfg.min_max_value)         { return REASON_MAX_TOO_SMALL; }
        if (width_samples < cfg.min_width_samples) { return REASON_WIDTH_TOO_NARROW; }
        if (width_samples > cfg.max_width_samples) { return REASON_WIDTH_TOO_WIDE; }
        if (area < cfg.min_area)                   { return REASON_AREA_TOO_SMALL; }
        const float slope = (float)max_value / (float)(width_samples ? width_samples : 1u);
        if (slope > cfg.max_slope)                 { return REASON_SHORT_FOR_HEIGHT; }
        return REASON_OK;
    }

private:
    enum State { STATE_WARMUP, STATE_IDLE, STATE_EVENT };

    void updateBaseline(float x) {
        if (!primed_) {
            // Seed from the very first sample so a slow EMA still converges fast.
            mean_   = x;
            var_    = 0.0f;
            primed_ = true;
        } else {
            const float dev = x - mean_;
            mean_ += cfg_.baseline_alpha * dev;
            var_  += cfg_.noise_alpha * (dev * dev - var_);
        }
        if (var_ < 0.0f) { var_ = 0.0f; }
        sigma_ = sqrtf(var_);
        if (sigma_ < (float)cfg_.sigma_floor) { sigma_ = (float)cfg_.sigma_floor; }
    }

    void startEvent(int32_t x, uint32_t t_us, uint32_t index) {
        state_          = STATE_EVENT;
        base_at_start_  = (int32_t)lroundf(mean_);
        sigma_at_start_ = sigma_;
        ev_start_us_    = t_us;
        ev_start_index_ = index;
        ev_samples_     = 0;
        ev_peak_us_     = t_us;
        ev_max_         = x - base_at_start_;
        ev_area_        = (ev_max_ > 0) ? (uint64_t)ev_max_ : 0u;
        below_count_    = 0;
    }

    void closeEvent(uint32_t t_us, uint32_t index, PeakMetrics& out) {
        const uint32_t width_samples = index - ev_start_index_;

        out.index         = event_counter_++;
        out.start_us      = ev_start_us_;
        out.peak_us       = ev_peak_us_;
        out.width_us      = t_us - ev_start_us_;
        out.width_samples = width_samples;
        out.samples       = ev_samples_;
        out.baseline      = base_at_start_;
        out.noise         = (int32_t)lroundf(sigma_at_start_);
        out.threshold     = (int32_t)lroundf(base_at_start_ + cfg_.k_on * sigma_at_start_);
        out.max_value     = ev_max_;
        out.max_abs_value = base_at_start_ + ev_max_;
        out.area          = ev_area_;

        // Verdict from the shared bound ladder (the same one EventAggregator uses
        // for metrics it measured over another channel's window).
        out.reason        = classifyPeak(cfg_, ev_max_, width_samples, ev_area_);
        out.valid         = metricsValid(out.reason) ? 1u : 0u;

        // Resume idle tracking. The baseline was intentionally NOT updated during
        // the event, so a long/strong peak cannot drag it upwards.
        state_ = STATE_IDLE;
    }

    Config   cfg_;
    State    state_          = STATE_WARMUP;
    uint32_t samples_seen_   = 0;
    float    mean_           = 0.0f;
    float    var_            = 0.0f;
    float    sigma_          = 0.0f;
    int32_t  base_at_start_  = 0;
    float    sigma_at_start_ = 0.0f;
    uint32_t ev_start_us_    = 0;
    uint32_t ev_start_index_ = 0;
    uint32_t ev_samples_     = 0;
    uint32_t ev_peak_us_     = 0;
    int32_t  ev_max_         = 0;
    uint64_t ev_area_        = 0;
    uint32_t below_count_    = 0;
    uint32_t event_counter_  = 0;
    bool     primed_         = false;
};
