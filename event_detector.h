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
//   * The reference (baseline + noise) is established during a QUIET PERIOD at the
//     start of the run and is only ever *maintained* afterwards: it is never
//     re-seeded and never allowed to take a step (see updateBaseline()).
//   * One event is found in four phases (see update()):
//
//         QUIET  ->  IDLE  ->  RISING  ->  EVENT
//
//       QUIET : the quiet period.  Detection is disarmed and the reference is
//               established from the samples seen here.
//       IDLE  : two things are watched here.  The *window* of the next event opens
//               the moment the signal leaves the noise band (baseline + k_off *
//               sigma) and survives for as long as it stays out of it, which is
//               the span the width is measured over and the span EventAggregator
//               times every channel inside; the *candidate* starts where the
//               signal rises above baseline + k_on * sigma (the trigger).  The
//               event the candidate becomes starts back at the window's opening
//               sample - or on the crossing itself, if the trace jumped straight
//               through both levels.
//       RISING: the rise test.  A real rise lasts some time, so the candidate has
//               to survive `min_rise_samples` samples, and it may dip back below
//               baseline + k_off * sigma  for up to `dip_tolerance` consecutive
//               samples while it does.  A candidate that does not survive is a
//               noise event: it is SUPPRESSED - nothing is reported for it, not
//               even a rejected record.
//       EVENT : the excursion is an event.  Height, area and the leading edge are
//               measured until the signal has settled back into the baseline band;
//               that sample closes the event and every metric and the verdict are
//               final there (one event, one report, nothing truncated).
//
// What is tracked
// ---------------
//   * baseline : running mean (EMA), updated ONLY while no event is in progress.
//   * noise    : standard deviation (EMA of the variance), likewise frozen
//                during an event so a long peak cannot corrupt the reference.
//   * Both are seeded ONCE, from the quiet period (see updateBaseline), and every
//     later update is gated - so the reference cannot take a step in either
//     direction: a rising peak cannot drag it up, and a single outlier cannot move
//     it at all.
//
// Input conditioning
// ------------------
// Every sample passes through an impulse filter first (see ImpulseFilter below):
// a noise spike is not a peak, so it must be able to neither start an event, nor
// end one early, nor become one.  The state machine, the baseline/noise tracker,
// the width, the area and the reported maximum are all measured on that filtered
// stream, so `max_value` is the maximum of the underlying signal instead of the
// highest single sample the noise happened to produce.
//
// Metrics per closed event (see PeakMetrics): maximum value, area, width and the
// length of the leading edge.  False-peak rejection uses the bounds in Config:
// amplitude, width and area (a peak has to reach them), and the two shape guards -
// the leading edge must be long enough (a step is not a peak) and must not climb
// faster than max_slope counts/sample.
//
// Two different things can happen to a false peak, and they are not the same:
//   * it died during the rise test (a spike, a blip): SUPPRESSED, nothing is
//     reported - see DetectorResult::DET_DISCARDED;
//   * it survived the rise test but failed one of the bounds above: it is a real
//     excursion and IS reported, as a rejected record carrying the reason
//     (REASON_MAX_TOO_SMALL / WIDTH_TOO_NARROW / ...).  That is what lets the
//     operator see "this thing is 1.5 ms wide" instead of nothing at all.
//
// This file has no Arduino dependency and is therefore unit-testable on a host.

#pragma once

#include <stdint.h>
#include <math.h>
#include "peak_metrics.h"

// ---------------------------------------------------------------------------
// ImpulseFilter - the detector's input stage (see "Input conditioning" above)
// ---------------------------------------------------------------------------
// One sample of a real signal can be blown up by an impulse (EMI, a spark, a
// cosmic ray).  Such a spike is taller than any real peak, lasts a sample or two,
// and is not part of the signal at all.
//
// The filter is a running median of the last `taps` samples.  A median ignores the
// extremes of its window, so up to taps/2 consecutive spike samples are removed
// while a real peak - whose neighbours are nearly as high as it is - passes
// through almost unchanged (a rounded apex loses a count or two, a flat top none).
// The price is a fixed delay of taps/2 samples, which shifts every timestamp by
// that much but changes no width (both ends of an event move together).
//
//   taps = 1 : off (raw passthrough)
//   taps = 3 : drops a 1-sample spike (the model's own impulse noise)
//   taps = 5 : drops a 1- or 2-sample spike
//   taps = 7 : drops up to 3 spike samples - what an impulse looks like after the
//              analog front end has smeared it over a couple of sample periods
//              (see the note on the front end in the README's bench section)
//
// Only odd values up to kMaxTaps are used; anything else is rounded down to the
// nearest odd value.  Before the first sample (or after reset()) the window is
// filled with that sample, so the filter never emits a value it has no history
// for - which also means a spike in the very first sample cannot get through.
//
// The cost of a longer window is delay (taps/2 samples) and a slightly blunted
// very narrow feature: a real peak is hundreds of samples wide, so a 7-tap median
// moves its apex by a count or two, while a 1-3 sample spike disappears entirely.
struct ImpulseFilter {
    static const uint8_t kMaxTaps = 9u;

    void configure(uint8_t taps) {
        if (taps < 1u)                          { taps_ = 1u; }
        else if (taps > (uint8_t)kMaxTaps)      { taps_ = (uint8_t)kMaxTaps; }
        else                                    { taps_ = (uint8_t)(taps | 1u); }
        reset();
    }

    void reset() {
        for (uint8_t i = 0; i < kMaxTaps; i++) { ring_[i] = 0; }
        head_   = 0u;
        count_  = 0u;
        primed_ = false;
    }

    // Feed one raw sample; returns the filtered one (and remembers it).
    int32_t step(int32_t x) {
        if (taps_ <= 1u) { value_ = x; return value_; }

        if (!primed_) {                       // no history: start on this sample
            for (uint8_t i = 0; i < taps_; i++) { ring_[i] = x; }
            primed_ = true;
        } else {
            ring_[head_] = x;
            head_ = (uint8_t)((head_ + 1u) % taps_);
        }
        if (count_ < taps_) { count_++; }

        // Median of the (at most kMaxTaps) buffered samples: insertion sort, tiny n.
        int32_t w[kMaxTaps];
        for (uint8_t i = 0; i < count_; i++) { w[i] = ring_[i]; }
        for (uint8_t i = 1u; i < count_; i++) {
            const int32_t key = w[i];
            int8_t j = (int8_t)i - 1;
            while (j >= 0 && w[j] > key) { w[j + 1] = w[j]; j--; }
            w[j + 1] = key;
        }
        value_ = w[(count_ - 1u) / 2u];       // lower median: a pair never picks a spike
        return value_;
    }

    // The value the last step() returned (what the detector is working on).
    int32_t value() const { return value_; }

    uint8_t taps() const { return taps_; }

private:
    int32_t ring_[kMaxTaps] = {};   // value-initialised; reset() fills it anyway
    int32_t value_          = 0;
    uint8_t taps_           = 3u;
    uint8_t head_           = 0u;
    uint8_t count_          = 0u;
    bool    primed_         = false;
};

// What one call to EventDetector::update() produced.
enum DetectorResult : uint8_t {
    DET_NONE      = 0, // nothing to report
    DET_CLOSED    = 1, // an event closed; `out` holds its final metrics
    DET_DISCARDED = 2, // a rise candidate failed the rise test and was suppressed:
                       // no event is reported for it at all.  A caller that had
                       // already started following the candidate (EventAggregator,
                       // which opens its window on the crossing) has to drop it.
};

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

        // ---- the quiet period, and the rise test ----------------------------
        // The quiet period is what ESTABLISHES the reference: detection is disarmed
        // for these samples and the baseline/noise are seeded from them (see
        // updateBaseline), so nothing later can take a step in the reference - it is
        // only ever maintained afterwards.
        // 3000 samples = 0.3 s at 10 kSPS.
        uint32_t quiet_samples      = 3000;
        // The rise test (state RISING): a real rise lasts some time, so a candidate
        // has to survive this many samples before the excursion counts as an event.
        // 20 samples = 2 ms at 10 kSPS - anything happening in the 1-2 ms range is
        // not an event (see DET_MIN_RISE_SAMPLES in config.h).
        uint32_t min_rise_samples   = 20;
        // ... and while it does, the signal may dip back into (or below) the baseline
        // band for up to this many *consecutive* samples.  A dip that lasts longer
        // kills the candidate and it is suppressed: that is how a lone spike, a
        // 1-sample dropout or a 0.5 ms glitch disappears without leaving a record
        // behind.  3 samples = 0.3 ms at 10 kSPS.
        uint32_t dip_tolerance      = 3;
        // End of an event: the bounded signal has to stay below
        // baseline + k_off * sigma for this many consecutive samples.
        // 2 samples = 0.2 ms at 10 kSPS.
        uint32_t end_debounce       = 2;

        // ---- how the quiet period seeds the reference, and the gate ceiling --
        // The reference (baseline + sigma) is what every level in this file is
        // measured against, so it must converge to the *noise* - never to the signal.
        // During the quiet period the detector collects `quiet_seed_points` samples,
        // spread evenly over the whole period, and seeds the reference from their
        // median and their (p84 - p16) / 2.  Two reasons for a median and not a mean
        // or an EMA (see updateBaseline):
        //   * neither a single outlier (a spike) nor one peak that happens to land in
        //     the quiet period can decide the reference;
        //   * it tolerates up to 50% contamination, so the quiet period does not have
        //     to be perfectly silent - only *mostly* quiet.  A signal that is high for
        //     most of the period still seeds it wrongly: the answer to that is a
        //     longer quiet period (DET_QUIET_SAMPLES in config.h), not a smarter
        //     estimator.
        // Spreading the points over the whole period (instead of seeding from its
        // first N samples) is what makes the period count: 128 points taken across
        // 0.3 s say far more about the noise floor than 128 samples inside its first
        // 13 ms.
        // The reference is seeded exactly ONCE, when the quiet period ends, and from
        // then on it is only maintained.
        // One more guard: every later update is gated - only samples within
        // k_gate * sigma of the baseline may touch the statistics, and the gate is
        // capped in absolute counts, so it can never grow wider than gate_max even if
        // sigma does.  Keeping the gate narrower than the trigger is what stops a
        // signal that sits in the band from feeding the reference indefinitely.
        uint32_t quiet_seed_points  = 128;     // points the quiet period seeds from
        uint32_t gate_max           = 320;     // gate ceiling in counts; 0 = no ceiling

        // ---- input conditioning ----
        // Length of the running median in front of the detector: 1 = off, 3 = drop a
        // 1-sample noise spike, 5 = up to 2, 7 = up to 3 (the shipped value, see
        // DET_IMPULSE_FILTER_TAPS in config.h and the note above ImpulseFilter).
        uint8_t  impulse_filter_taps = 7;

        // ---- metric bounds used to judge a closed excursion ------------------
        int32_t  min_max_value      = 160;     // peak amplitude (counts) must reach this
        uint32_t min_width_samples  = 35;      // the event's *duration* must be at least
                                               // this many samples.  35 = 3.5 ms at
                                               // 10 kSPS: the shortest event worth
                                               // reporting (see DET_MIN_WIDTH_SAMPLES in
                                               // config.h and envelopeDuration() below,
                                               // which is what "duration" is measured as).
        uint32_t max_width_samples  = 20000;   // width must be at most this.  Not a filter:
                                               // a signal that never comes back is a fault,
                                               // and this closes it so the detector cannot
                                               // hang on a stuck channel.  20000 = 2 s at
                                               // 10 kSPS, i.e. 4x the longest event
                                               // expected (500 ms).
        uint64_t min_area           = 0;       // integrated area must reach this
        // max_slope bounds the *measurement path* (see "Bounded slope" at the top of
        // this file) and doubles as the shape guard in classifyPeak(): no value may be
        // tracked as the apex - and no descent may be read as "back at the baseline" -
        // if the signal moved faster than this.  200 counts/sample = 2e6 counts/s at
        // 10 kSPS: 5x the fastest edge a real event can have (1000 counts in 5 ms) and
        // far below what a spike does (1500 counts in a single sample).
        float    max_slope          = 200.0f;
    };

    EventDetector() { reset(); }
    explicit EventDetector(const Config& c) : cfg_(c) { reset(); }

    // Feed one sample of the channel this detector is tracking.
    //   x     : channel sample value (AD7606 counts)
    //   t_us  : timestamp in microseconds
    //   index : monotonically increasing sample index (used for width in samples)
    //   out   : filled when this call closes an event
    // Returns what the sample decided: nothing (DET_NONE, the usual case), an event
    // that just closed (DET_CLOSED - `out` holds its final metrics), or a rise
    // candidate that failed and was suppressed (DET_DISCARDED - nothing is reported).
    DetectorResult update(int32_t x, uint32_t t_us, uint32_t index, PeakMetrics& out) {
        samples_seen_++;

        // Input conditioning: everything below - the statistics, the thresholds and
        // every reported metric - works on the impulse-filtered sample, never on the
        // raw one (see ImpulseFilter).
        x = filt_.step(x);
        filtered_ = x;

        switch (state_) {
            case STATE_QUIET:
                // The quiet period: detection is disarmed and the reference is
                // established from what is seen here.  It is seeded exactly once, on
                // the sample that ends the period, so the detector is always primed
                // before it is armed (an unprimed detector would run on mean == 0).
                collectSeed((int32_t)lroundf((float)x));
                if (samples_seen_ >= cfg_.quiet_samples) {
                    primeFromSeed();
                    state_ = STATE_IDLE;
                }
                return DET_NONE;

            case STATE_IDLE: {
                // Refresh the reference only while the trace is INSIDE the noise band.
                // The gate inside updateBaseline() caps how far a sample may sit from the
                // baseline, but a faint peak's whole lower flank is *inside* that gate (the
                // band opens at k_off < k_gate sigma), so a peak that never leaves the band
                // would feed its own rise into sigma: measured on the floor ladder at 600
                // samples, sigma walks 27 -> 38.4 counts across one 120-count rung, which
                // lifts the trigger (5 sigma) from +136 to +192 counts - above that rung's own
                // apex, so it is never crossed.  Once the window opens, the excursion is the
                // signal and must not decide the *noise*: sigma is left alone until the
                // trace is back in the band.  The mean, though, keeps tracking (same gate):
                // it is what follows the slow wander, and freezing it lets a wide rung's
                // own drift pull the trace away from the reference instead (measured: a
                // 1545-sample rung then stays "open" for the whole 20000-sample fault guard).
                // (One sample of lag, deliberately: pend_valid_ is what the *previous*
                // sample left behind.)
                updateBaseline((float)x, !pend_valid_);
                // Both levels are taken off the *rounded* baseline, i.e. exactly the
                // value the event will latch as base_at_start_ and report as
                // threshold= - and exactly what EventAggregator freezes for the shared
                // window (see openWindow()).  Rounding the float mean twice instead
                // could shift a crossing by one count.
                const int32_t base = (int32_t)lroundf(mean_);
                const int32_t thr  = (int32_t)lroundf((float)base + cfg_.k_on * sigma_);
                // The trigger crossing starts the *candidate*, not the event: where the
                // event begins is decided below, and the crossing sample is measured by
                // startCandidate()/measure() - so it is checked first, and the window's
                // own bookkeeping stops short of it, so no sample is counted twice.
                if (x > thr) { startCandidate(x, t_us, index); return DET_NONE; }

                // The window of the next event opens where the trace leaves the noise
                // band (+k_off * sigma), not where it crosses the trigger (+k_on * sigma):
                // the trigger is what decides the excursion is worth reporting, while the
                // window *is* the excursion - and both of its ends then sit on the same
                // level, which is also the level the duration is read back at (see
                // envelopeDuration).  The onset is remembered - together with the flank
                // area and sample count in front of the crossing - while the trace stays
                // out of the band, and it is dropped once the trace has been back inside
                // the band for more than dip_tolerance consecutive samples.  That is the
                // same rule the rise test applies, and it is why a band crossing on its
                // own cannot open a window.
                const int32_t band = (int32_t)lroundf((float)base + cfg_.k_off * sigma_);
                if (x > band) {
                    if (!pend_valid_) {
                        pend_valid_   = true;
                        pend_us_      = t_us;
                        pend_index_   = index;
                        pend_area_    = 0;
                        pend_samples_ = 0;
                    }
                    const int32_t amp = x - base;
                    if (amp > 0) { pend_area_ += (uint64_t)amp; }
                    pend_samples_++;
                    below_count_ = 0;
                } else {
                    below_count_++;
                    if (below_count_ > cfg_.dip_tolerance) { pend_valid_ = false; }
                }
                return DET_NONE;
            }

            case STATE_RISING: {
                // The rise test.  The candidate is measured from here on, so a rise that
                // survives it keeps its full leading edge - and that edge is the pulse's
                // own, because startCandidate() seeded the event's start, its area and its
                // sample count from the window's opening sample.
                measure(x, t_us, index);
                // A real rise dips: up to dip_tolerance CONSECUTIVE samples may fall
                // back into the baseline band.  This test runs on the *filtered
                // sample*, not on the bounded path: the rule is literally "a dip of
                // one to three datapoints is ignored", and a dip is a dip however
                // fast it is.  A longer run means the excursion was over - the
                // "crossing" was a spike, a blip or a dropout - and the candidate is
                // suppressed without leaving a record behind.
                if (belowRun(x, t_us, index) > cfg_.dip_tolerance) {
                    discardCandidate();
                    return DET_DISCARDED;
                }
                // Stuck signal: the candidate neither confirms nor dies.  Promote and
                // close it, so the fault is reported (REASON_WIDTH_TOO_WIDE) instead
                // of hiding in a candidate that would live for ever.
                if ((index - ev_start_index_) >= cfg_.max_width_samples) {
                    state_ = STATE_EVENT;
                    closeEvent(t_us, index, out);
                    return DET_CLOSED;
                }
                // The rise has to last: the candidate must span min_rise_samples
                // samples before the excursion counts as an event.
                if ((index - ev_start_index_ + 1u) >= cfg_.min_rise_samples) {
                    state_ = STATE_EVENT;
                    // The end test counts its own run, on its own signal (the bounded
                    // path), so a dip that the rise test tolerated cannot be inherited
                    // as part of the run that ends the event.
                    below_count_ = 0;
                }
                return DET_NONE;
            }

            case STATE_EVENT: {
                measure(x, t_us, index);
                // The event ends where the signal has settled back into the baseline
                // band - and it ends *there*, not end_debounce samples later: the
                // close is reported on the sample the run started on, so the width is
                // the excursion and not the excursion plus the debounce.  Every metric
                // and the verdict are final at that sample: one event, one report.
                const int32_t path = (int32_t)lroundf(path_);
                if (belowRun(path, t_us, index) >= cfg_.end_debounce) {
                    // A noise event: the excursion never reached the amplitude bound, so
                    // it was never a candidate for a peak - suppress it instead of
                    // reporting it (the same rule the rise test applies to a spike).
                    // Without this, every dither excursion that happens to outlive the
                    // rise test becomes a `rejected MAX_TOO_SMALL` line on the link.
                    if (ev_max_ < cfg_.min_max_value) {
                        discardCandidate();
                        return DET_DISCARDED;
                    }
                    closeEvent(below_start_us_, below_start_index_, out);
                    return DET_CLOSED;
                }
                // Stuck signal (see STATE_RISING): a real event is at most
                // max_width_samples long, and a channel that never comes back must not
                // block detection for ever.
                if ((index - ev_start_index_) >= cfg_.max_width_samples) {
                    closeEvent(t_us, index, out);
                    return DET_CLOSED;
                }
                return DET_NONE;
            }
        }
        return DET_NONE;
    }

    void reset() {
        state_          = STATE_QUIET;
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
        ev_peak_index_  = 0;
        ev_max_         = 0;
        ev_area_        = 0;
        below_count_    = 0;
        below_start_us_    = 0;
        below_start_index_ = 0;
        pend_valid_     = false;
        pend_us_        = 0;
        pend_index_     = 0;
        pend_area_      = 0;
        pend_samples_   = 0;
        path_           = 0.0f;
        clipped_        = 0;
        jumped_         = 0;
        last_jump_index_ = 0;
        discarded_      = 0;
        event_counter_  = 0;
        for (uint32_t i = 0; i < kQuietSeedMax; i++) { seed_[i] = 0; }
        seed_n_         = 0;
        seed_skip_      = 0;
        filtered_       = 0;
        filt_.configure(cfg_.impulse_filter_taps);
    }

    void configure(const Config& c) { cfg_ = c; reset(); }

    // Drop everything and re-establish the reference from a fresh quiet period.
    // Used when a channel has been saturated (railed): its readings were meaningless,
    // so the stale reference is discarded instead of being allowed to bias the next
    // event.
    //
    // The detector re-enters the quiet period instead of re-seeding from the one
    // sample that happens to follow the episode: in this design the reference is
    // established from a quiet period and never taken from a single sample, and the
    // saturation told us nothing about the noise floor either (a stale, large sigma
    // would sit far above the real noise and a freshly seeded, too small one would
    // put the trigger below it and make the detector chatter).  Detection is blind
    // for quiet_samples samples after a rail call - that is the price of getting the
    // reference right; DET_RAIL_COOLDOWN in config.h delays the call itself until the
    // input has settled again.
    void restartQuietPeriod() {
        filt_.reset();          // its window still holds clipped samples
        filtered_          = 0;
        state_             = STATE_QUIET;
        samples_seen_      = 0;
        seed_n_            = 0;
        seed_skip_         = 0;
        base_at_start_     = 0;
        sigma_at_start_    = 0.0f;
        ev_start_us_       = 0;
        ev_start_index_    = 0;
        ev_samples_        = 0;
        ev_peak_us_        = 0;
        ev_peak_index_     = 0;
        ev_max_            = 0;
        ev_area_           = 0;
        below_count_       = 0;
        below_start_us_    = 0;
        below_start_index_ = 0;
        pend_valid_        = false;
        pend_us_           = 0;
        pend_index_        = 0;
        pend_area_         = 0;
        pend_samples_      = 0;
        path_              = 0.0f;
        clipped_           = 0;
        jumped_            = 0;
        last_jump_index_   = 0;
        // mean_/var_/sigma_ keep their last value until the new quiet period seeds
        // them: they are not used while the detector is unarmed (nothing can open a
        // window or judge a peak), and leaving them alone keeps baseline()/sigma()
        // meaningful for the report that accompanies a rail.
    }

    const Config& config() const { return cfg_; }
    Config&       config()       { return cfg_; }

    // True while an event OR a rise candidate is in progress, or while the window the
    // event will start from is open - i.e. from the sample the signal left the noise band
    // to the sample the event closed on (see update()'s IDLE case).  This is what
    // EventAggregator follows to time its shared window: the window has to open on the
    // event's own opening sample, not on the trigger crossing and not on the sample the
    // rise test happens to pass, or every other channel would lose the leading edge.
    bool  inEvent()     const {
        return state_ == STATE_EVENT || state_ == STATE_RISING || pend_valid_;
    }
    // True while the rise test is running (the candidate may still be suppressed).
    bool  inCandidate() const { return state_ == STATE_RISING; }
    // True once the quiet period is over: detection is armed.
    bool  armed()       const { return state_ != STATE_QUIET; }
    // How many rise candidates this run has suppressed, and how many samples had to be
    // clipped to max_slope (see measure()).  Diagnostics only - neither is shipped.
    uint32_t discardedCandidates() const { return discarded_; }
    uint32_t clippedSamples()      const { return clipped_; }
    float baseline() const { return mean_; }
    float sigma()    const { return sigma_; }

    // The filtered value of the last sample fed to update() (see ImpulseFilter).
    // EventAggregator measures the height of each channel inside the shared window
    // on this value, so the window reports the same spike-free signal the channel's
    // own detector judged.
    int32_t filtered() const { return filtered_; }

    // The event's *duration*, derived from the measured excursion: the width the metric
    // bounds are judged with, and the number reported as the event's width.
    //
    // The measurement can only ever see the window - the span from the sample the trace
    // left the noise band to the sample it returned into it (hysteresis included,
    // end_debounce excluded) - and that span is not the pulse's length: the trace reaches
    // the band a little way up the pulse's flank on the way in and leaves it a little way
    // up on the way out, so the span is the length *minus* the two flank pieces that never
    // cleared the band.  Measured on the bench (test/test_bench_detect.exe --cycles 12):
    // the window covers 117 samples (1.2 ms) less than the pulse the generator drew, on
    // average (|max| 435), so a floor compared straight against it rejected real events as
    // "too narrow" (see the DET_MIN_WIDTH_SAMPLES note in config.h).
    //
    // What this application's signal is, to within a few percent, is a rounded pulse: a
    // raised cosine of duration W whose flanks reach the baseline at its ends.  Its height
    // is A, the level it is read against is T, and the span it spends above T is
    // (W / pi) * acos(2T/A - 1), i.e. W * acos(2T/A - 1) / pi.  Inverting that gives the
    // duration the measured span implies, which is what "the shortest event worth
    // reporting" has to be compared against.  T is the level both ends of the window sit
    // on - the off-threshold, k_off * sigma, which is what closeEvent() passes - and
    // reading the span back at the very level it was measured between is what makes the
    // inversion come out at W: over the same 12 cycles the duration lands within 12
    // samples of the drawn width on average (|max| 165) where the span itself is out by
    // 117, and no event over this floor is judged under it.
    //
    // Three properties are deliberate:
    //   * a bright event (A >> T) gets a factor of 1.0, so nothing changes for it;
    //   * the factor is capped at 2.0: a pulse whose apex barely clears the band makes
    //     the relation degenerate (infinitely wide), and reading it as at most twice
    //     its excursion keeps one noisy crossing from turning a blip into an "event";
    //   * the factor is never below 1.0, so this can only *keep* an event the excursion
    //     alone would have thrown away - never lose one it would have kept.
    // It is an estimate, not a measurement, and it can overshoot: a flat-topped
    // (rectangular) event of the same span reads up to twice as long, a descent the
    // bounded path spun out (an edge faster than DET_MAX_SLOPE) leaves the excursion
    // itself longer than the pulse, and each crossing carries a sample or two of noise -
    // the bench's 4.6 ms / 1072-count needle comes back drawn 46, window 44, duration 50
    // samples, i.e. the correction puts back a little more than the band cut off.
    // Overshooting is the safe direction for a floor (a length read too long cannot lose
    // an event; the other direction is what the bench checks a resolution away from the
    // bound, kWidthResolve in test/test_bench_detect.cpp), and the events that overshoot
    // most are the ones DET_MIN_RISE_SAMPLES / DET_MAX_SLOPE judge, not this: the shape,
    // not the length, is what is wrong with them.
    //
    // Three properties are deliberate:
    //   * a bright event (A >> T) gets a factor of 1.0, so nothing changes for it;
    //   * the factor is capped at 2.0: a pulse that only just reaches the trigger makes
    //     the relation degenerate (infinitely wide), and reading it as at most twice
    //     its excursion keeps one noisy crossing from turning a blip into an "event";
    //   * the factor is never below 1.0, so this can only *keep* an event the excursion
    //     alone would have thrown away - never lose one it would have kept.
    // It is an estimate, not a measurement, and it can overshoot: a flat-topped
    // (rectangular) event of the same span reads up to twice as long, a descent the
    // bounded path spun out (an edge faster than DET_MAX_SLOPE) leaves the excursion
    // itself longer than the pulse, and each crossing carries a sample or two of noise.
    // Overshooting is the safe direction for a floor (a length read too long cannot lose
    // an event; the other direction is what the bench checks a resolution away from the
    // bound, kWidthResolve in test/test_bench_detect.cpp), and the events that overshoot
    // most are the ones DET_MIN_RISE_SAMPLES / DET_MAX_SLOPE judge, not this: the shape,
    // not the length, is what is wrong with them.
    static uint32_t envelopeDuration(int32_t max_value, int32_t threshold,
                                     uint32_t width_samples) {
        // Both levels are counts ABOVE the baseline, like every other bound here.
        if (width_samples == 0u || max_value <= 0 || threshold <= 0) {
            return width_samples;
        }
        double factor;
        if (max_value > threshold) {
            const double r = 2.0 * (double)threshold / (double)max_value - 1.0;
            factor = (r < 1.0 && r > -1.0)
                         ? (3.14159265358979323846 / acos(r))
                         : 1.0;
            if (factor > 2.0) { factor = 2.0; }
        } else {
            // The trigger is the apex: there is no flank left to read the length from.
            factor = 2.0;
        }
        if (factor <= 1.0) { return width_samples; }
        const double widened = (double)width_samples * factor;
        return (widened >= 4294967295.0) ? 0xFFFFFFFFu : (uint32_t)lround(widened);
    }

    // The rejection ladder used to judge a closed peak. Exposed as a static so
    // EventAggregator can judge metrics it measured itself (a channel's height /
    // area inside another channel's window) with exactly the same rules.
    // `width_samples` is the event's DURATION (envelopeDuration above), not the
    // excursion the measurement saw: the bound it is checked against - "the shortest
    // event worth reporting" - is a statement about how long the event was, which is
    // exactly what the excursion cannot tell you.
    // `rise_samples` is the length of the leading edge: the number of samples from
    // the event start to its maximum (0 = the first sample is already the top), as
    // measured on the slope-bounded path - so a peak whose apex arrives too quickly
    // (a step, a spike that survived, an edge the path had to clip) is rejected here.
    // Returns REASON_OK when the peak satisfies every bound.
    static uint8_t classifyPeak(const Config& cfg, int32_t max_value,
                                uint32_t width_samples, uint64_t area,
                                uint32_t rise_samples, uint32_t apex_unsettled = 0u) {
        if (max_value < cfg.min_max_value)         { return REASON_MAX_TOO_SMALL; }
        // `width_samples` is the duration (see the note above), so this is exactly the
        // "shortest event worth reporting" bound the config documents.  The maximum
        // below is the stuck-channel fault guard; the duration is never shorter than
        // the excursion it came from, so a channel that hit that guard - and was closed
        // by it - still reports WIDTH_TOO_WIDE here.
        if (width_samples < cfg.min_width_samples) { return REASON_WIDTH_TOO_NARROW; }
        if (width_samples > cfg.max_width_samples) { return REASON_WIDTH_TOO_WIDE; }
        if (area < cfg.min_area)                   { return REASON_AREA_TOO_SMALL; }

        // Shape: a peak has to *climb* to its maximum, and the climb has to take
        // time - an apex that arrives within min_rise_samples of the crossing was
        // not measured on a signal that rose like a real event does ("something that
        // happens in one or two milliseconds is not an event").
        // The two guards below are one idea from two sides: the edge must last
        // (min_rise_samples) and it must not have moved faster than max_slope.  For
        // the detector's own events the slope part is already guaranteed by the
        // measurement path (measure() clips anything faster), so it bites on the
        // values EventAggregator measures inside a window - and it is what used to be
        // missing when the average slope over the whole width was judged instead: a
        // 6-sample rectangle of 1000 counts averages 167 counts/sample, i.e. it looked
        // "slow enough" although it rises instantly.
        const uint32_t rise = rise_samples ? rise_samples : 1u;   // 0 = vertical edge
        if (rise < cfg.min_rise_samples)           { return REASON_SHORT_FOR_HEIGHT; }
        if ((float)max_value / (float)rise > cfg.max_slope) { return REASON_SHORT_FOR_HEIGHT; }
        // ... and the apex must not have been spiked into existence (see
        // EventDetector::measure / apexUnsettled): the height is bounded either way, but
        // an apex the *signal* jumped to is a spike or a burst of impulses, not a peak.
        // A spike that lands on the *front* of a real peak costs that event nothing -
        // its own apex arrives well after the last jump.  `apex_unsettled` is 0 for a
        // caller that does not track it (the shared window judges its channels on the
        // bounds and the two guards above).
        if (apex_unsettled != 0u)                  { return REASON_SHORT_FOR_HEIGHT; }
        return REASON_OK;
    }

private:
    enum State { STATE_QUIET, STATE_IDLE, STATE_RISING, STATE_EVENT };

    // Bound on the quiet period's seed buffer (see Config::quiet_seed_points).
    static const uint16_t kQuietSeedMax = 256u;

    // Maintain the reference from one sample.  Called from IDLE only, never while a
    // candidate or an event is open: the reference is established once, by the quiet
    // period (see collectSeed/primeFromSeed), and from then on it is only maintained.
    // Every update is gated - only samples within gateWidth() of the mean may touch
    // the statistics, and the gate itself is bounded.  That gate is what makes the
    // reference stable: a rising peak is outside it (the trigger sits at
    // k_on > k_gate sigma) and cannot drag the baseline upwards, and one outlier moves
    // the mean by a fraction of a count at most.
    //
    // `update_noise` is false while the detector has a window open - i.e. from the sample
    // the trace left the noise band until it is back inside it (see STATE_IDLE).  The gate
    // alone does not protect sigma there: a faint peak's whole lower flank is *inside* the
    // gate, so it feeds its own rise into the estimate and the trigger climbs with it
    // until the peak stays under it (the floor the ladder walks).  An excursion may move
    // the mean - the mean is what tracks the slow wander - but never sigma.
    void updateBaseline(float x, bool update_noise = true) {
        if (fabsf(x - mean_) > gateWidth()) { return; }

        const float dev = x - mean_;
        mean_ += cfg_.baseline_alpha * dev;
        if (!update_noise) { return; }   // an excursion may move the mean, never sigma
        var_  += cfg_.noise_alpha * (dev * dev - var_);
        if (var_ < 0.0f) { var_ = 0.0f; }
        sigma_ = sqrtf(var_);
        if (sigma_ < (float)cfg_.sigma_floor) { sigma_ = (float)cfg_.sigma_floor; }
    }

    // ---- the quiet period -------------------------------------------------------
    // How many points the quiet period seeds from (bounded by the seed buffer), and how
    // many samples apart they are taken: the points have to be spread over the WHOLE
    // period for the period to mean anything (see Config::quiet_seed_points).
    uint16_t quietSeedPoints() const {
        uint32_t n = cfg_.quiet_seed_points;
        if (n < 1u) { n = 1u; }
        if (n > (uint32_t)kQuietSeedMax) { n = (uint32_t)kQuietSeedMax; }
        return (uint16_t)n;
    }
    uint32_t quietSeedEvery() const {
        const uint32_t pts = (uint32_t)quietSeedPoints();
        return (cfg_.quiet_samples > pts) ? (cfg_.quiet_samples / pts) : 1u;
    }

    // Collect one seed point every quietSeedEvery() samples (the first sample of the
    // period is always taken, so even a pathological quiet_samples of 0 leaves
    // primeFromSeed() something to work with).  The samples that are NOT collected are
    // only used for nothing at all - the quiet period is not measured, it establishes
    // the reference.
    void collectSeed(int32_t x) {
        if (seed_n_ >= quietSeedPoints()) { return; }   // the whole period is covered
        if (seed_n_ == 0u) { seed_[seed_n_++] = x; return; }   // always take the first
        const uint32_t every = quietSeedEvery();
        if (every > 1u && ++seed_skip_ < every) { return; }
        seed_skip_ = 0u;
        seed_[seed_n_++] = x;
    }

    // The reference-update gate: relative to the current sigma, but bounded in
    // absolute counts so an (accidentally) large sigma can never widen the gate
    // enough to admit the signal it is supposed to reject.
    float gateWidth() const {
        const float g = cfg_.k_gate * sigma_;
        if (cfg_.gate_max > 0u && g > (float)cfg_.gate_max) { return (float)cfg_.gate_max; }
        return g;
    }

    // The reference is the *noise*, so both parts of it are estimated robustly from the
    // quiet period's seed points: the mean is their median, and sigma is their MAD
    // (median |x - median|) scaled by 1.4826, the standard robust estimator for a
    // normal distribution.
    // A median/MAD pair tolerates up to 50% contamination, which is what makes a quiet
    // period usable even when a pulse (or a chunk of one) lands inside it.  The
    // (p84 - p16) / 2 pair that was used here before breaks down at 16% contamination:
    // with a pulse covering a quarter of the quiet period, sigma came out hundreds of
    // counts too high - and a sigma that large puts the trigger (base + k_on*sigma)
    // above every real peak, so the detector goes blind instead of noisy.  That is the
    // exact failure this seeding exists to prevent.
    void primeFromSeed() {
        int32_t w[kQuietSeedMax];
        uint32_t n = (uint32_t)seed_n_;
        if (n == 0u) {
            w[0] = (int32_t)lroundf(mean_);   // nothing collected: keep what we have
            n    = 1u;
        } else {
            for (uint32_t i = 0; i < n; i++) { w[i] = seed_[i]; }
        }
        for (uint32_t i = 1u; i < n; i++) {
            const int32_t key = w[i];
            int32_t j = (int32_t)i - 1;
            while (j >= 0 && w[j] > key) { w[j + 1] = w[j]; j--; }
            w[j + 1] = key;
        }
        const int32_t med = w[n / 2u];

        int32_t d[kQuietSeedMax];
        for (uint32_t i = 0; i < n; i++) {
            const int32_t v = w[i] - med;
            d[i] = (v < 0) ? -v : v;
        }
        for (uint32_t i = 1u; i < n; i++) {
            const int32_t key = d[i];
            int32_t j = (int32_t)i - 1;
            while (j >= 0 && d[j] > key) { d[j + 1] = d[j]; j--; }
            d[j + 1] = key;
        }
        const float spread = 1.4826f * (float)d[n / 2u];

        mean_  = (float)med;
        var_   = spread * spread;
        sigma_ = (spread < (float)cfg_.sigma_floor) ? (float)cfg_.sigma_floor : spread;
    }

    // Was the apex spiked into existence?  The apex is trusted only when the signal had
    // stopped jumping at least min_rise_samples *before* it: a real peak whose front
    // carries a spike is fine (the spike is hundreds of samples earlier, and the apex is
    // the peak's own), while a burst of impulses has its apex sitting inside the burst -
    // the last jump is a sample or two away - and is not a peak at all.
    bool apexUnsettled() const {
        return (ev_peak_index_ - last_jump_index_) < cfg_.min_rise_samples;
    }

    // The level the signal has to come back down to, frozen at the event start:
    // baseline + k_off * sigma, with k_off < k_on so the two levels form a hysteresis
    // band instead of a single level.
    int32_t offThreshold() const {
        return (int32_t)lroundf(base_at_start_ + cfg_.k_off * sigma_at_start_);
    }

    // A rise candidate starts where the signal crossed the on-threshold, but the event it
    // may become started earlier: at the sample the signal left the noise band, which is
    // the window STATE_IDLE remembers (see update()'s IDLE case).  Its time and index are
    // the event's start, so the window the aggregator times every other channel inside and
    // the window the bounds are judged over are the *same* excursion - the leading edge is
    // the pulse's own, measured from the band and not from the trigger.  A crossing that
    // came without a window (the trace jumped through both levels at once) starts the
    // event on its own sample.  The crossing sample itself is measured straight away - it
    // is already part of the excursion, and not measuring it would throw away the first
    // sample of every fast rise - and the pending window deliberately stops short of it,
    // so no sample is counted twice.
    void startCandidate(int32_t x, uint32_t t_us, uint32_t index) {
        state_          = STATE_RISING;
        base_at_start_  = (int32_t)lroundf(mean_);
        sigma_at_start_ = sigma_;
        ev_start_us_    = pend_valid_ ? pend_us_    : t_us;
        ev_start_index_ = pend_valid_ ? pend_index_ : index;
        ev_samples_     = pend_valid_ ? pend_samples_ : 0;
        ev_peak_us_     = t_us;
        ev_peak_index_  = index;
        ev_max_         = 0;
        ev_area_        = pend_valid_ ? pend_area_ : 0;
        below_count_    = 0;
        below_start_us_    = t_us;
        below_start_index_ = index;
        clipped_        = 0;
        jumped_         = 0;
        last_jump_index_ = index;
        // The measurement path starts on the baseline, not on this sample: the path is
        // continuous, so even the crossing sample has to respect max_slope.  Without
        // that, a step (or the sample an impulse lands on) would register its full
        // height with a leading edge of 0 samples and bypass the bound entirely.
        path_           = (float)base_at_start_;
        measure(x, t_us, index);
    }

    // Advance the slope-bounded measurement path and fold this sample into the event's
    // metrics (see "Bounded slope" at the top of this file):
    //   * the path may move at most max_slope counts per sample, so a lone impulse is
    //     clipped instead of tracked (it can never become the event's maximum) and a
    //     very fast change is spread over the samples it would really have taken
    //     rather than being read as a step.  Every clipped sample is counted
    //     (clippedSamples()) - it is ignored, but not silently;
    //   * only samples INSIDE the event are measured, i.e. the bounded path at or
    //     above the baseline band.  The sample the signal returns to the baseline on
    //     is the END of the event, not part of it, and a tolerated dip does not
    //     contribute either (the signal was at the baseline there).
    void measure(int32_t x, uint32_t t_us, uint32_t index) {
        const float prev      = path_;
        const float step      = (float)x - prev;
        const bool  in_event  = ((int32_t)lroundf(prev) >= offThreshold());
        if (step > cfg_.max_slope)       { path_ = prev + cfg_.max_slope; clipped_++; }
        else if (step < -cfg_.max_slope) { path_ = prev - cfg_.max_slope; clipped_++; }
        else                             { path_ = (float)x; }

        const int32_t p = (int32_t)lroundf(path_);
        if (p < offThreshold()) { return; }

        // A jump: the event was already established (the path was inside the excursion)
        // and the raw moved more than max_slope ahead of it - the *signal* jumped, even
        // though the path keeps the measured value bounded.  The path is why a spike
        // cannot become the reported height on its own; the jump is remembered so the
        // *apex* can be judged: an apex that sits right on top of a jump was spiked
        // into existence (see closeEvent and classifyPeak).
        const bool jumped = (step > cfg_.max_slope && in_event);
        if (jumped) { jumped_++; }

        const int32_t amp = p - base_at_start_;
        if (amp > ev_max_) {
            ev_max_        = amp;
            ev_peak_us_    = t_us;
            ev_peak_index_ = index;
        }
        if (jumped && index <= ev_peak_index_) { last_jump_index_ = index; }
        if (amp > 0) { ev_area_ += (uint64_t)amp; }
        ev_samples_++;
    }

    // Track the run of samples the bounded path spends below the baseline band and
    // return its length.  below_start_* remembers where the run began - that sample is
    // where an event ends, so the debounce cannot lengthen the reported width.
    uint32_t belowRun(int32_t p, uint32_t t_us, uint32_t index) {
        if (p < offThreshold()) {
            if (below_count_ == 0u) { below_start_us_ = t_us; below_start_index_ = index; }
            below_count_++;
        } else {
            below_count_ = 0u;
        }
        return below_count_;
    }

    // The rise test failed: the candidate was a spike, a blip or a dropout.  Nothing
    // is reported for it and the detector goes back to watching the baseline - this is
    // the "suppress the noise event" rule.
    void discardCandidate() {
        discarded_++;
        state_       = STATE_IDLE;
        ev_samples_  = 0;
        ev_max_      = 0;
        ev_area_     = 0;
        below_count_ = 0;
        // The window the candidate came from is gone with it: its opening edge cannot
        // start a later event, a fresh window opens on the trace's next band breakout.
        pend_valid_   = false;
        pend_us_      = 0;
        pend_index_   = 0;
        pend_area_    = 0;
        pend_samples_ = 0;
        clipped_     = 0;
        jumped_      = 0;
        path_        = 0.0f;
        // The reference keeps its value: nothing in this file re-seeds it.
    }

    // Close the event on the sample it ended on and hand its final metrics to the
    // caller.  `end_us`/`end_index` are the sample the path returned into the noise band
    // on (or the current sample if a stuck signal hit max_width_samples), so the
    // *excursion* (width_us/width_samples) is the measurement itself and the end
    // debounce is not folded into it.  The event's *duration*
    // (duration_us/duration_samples) is derived from that excursion - it is the width
    // the bounds are judged with and the number the operator is shown as the event's
    // width (see envelopeDuration).  Nothing is deferred: this is the report for this
    // event.
    void closeEvent(uint32_t end_us, uint32_t end_index, PeakMetrics& out) {
        const uint32_t width_samples = end_index - ev_start_index_;
        // The duration the bounds are judged with: the excursion, widened for the pulse's
        // own envelope (see envelopeDuration).  The level it is read back at is the level
        // both ends of that excursion sit on - the off-threshold, k_off * sigma above the
        // frozen baseline - and not the trigger: the window opens where the trace leaves
        // the noise band and closes where it returns into it, so inverting the envelope at
        // the *same* level at both ends is what turns the span back into the pulse's own
        // length (the note there works the identities out).  The correction is applied to
        // *every* event, including the ones whose descent the bounded path spun out (a
        // clipped edge reads longer than it was): the estimate can then overshoot the
        // pulse's length, which is the safe direction for a floor and is why
        // DET_MAX_SLOPE / DET_MIN_RISE_SAMPLES, not this, judge those events.
        const uint32_t duration = envelopeDuration(
            ev_max_, offThreshold() - base_at_start_, width_samples);

        out.index         = event_counter_++;
        out.start_us      = ev_start_us_;
        out.peak_us       = ev_peak_us_;
        out.width_us      = end_us - ev_start_us_;
        out.width_samples = width_samples;
        // The same span in microseconds, widened by the same factor, so the two never
        // disagree about what the width is (a caller may report either).
        out.duration_samples = duration;
        out.duration_us   = (width_samples > 0u)
                                ? (uint32_t)((uint64_t)(end_us - ev_start_us_) *
                                             (uint64_t)duration / (uint64_t)width_samples)
                                : 0u;
        out.samples       = ev_samples_;
        out.rise_samples  = ev_peak_index_ - ev_start_index_;
        out.baseline      = base_at_start_;
        out.noise         = (int32_t)lroundf(sigma_at_start_);
        out.threshold     = (int32_t)lroundf(base_at_start_ + cfg_.k_on * sigma_at_start_);
        out.max_value     = ev_max_;
        out.max_abs_value = base_at_start_ + ev_max_;
        out.area          = ev_area_;

        // Verdict from the shared bound ladder (the same one EventAggregator uses
        // for metrics it measured over another channel's window).
        out.reason        = classifyPeak(cfg_, ev_max_, duration, ev_area_,
                                         out.rise_samples, apexUnsettled() ? 1u : 0u);
        out.valid         = metricsValid(out.reason) ? 1u : 0u;

        // Resume idle tracking.  The baseline was intentionally NOT updated during the
        // event, so a long or strong peak cannot drag it upwards; `samples` counts only
        // the samples that were inside the event (see measure()).  The window closed with
        // the event: the next one opens on the trace's next band breakout (see update()'s
        // IDLE case).
        state_       = STATE_IDLE;
        below_count_ = 0;
        pend_valid_  = false;
    }

    Config   cfg_;
    State    state_          = STATE_QUIET;
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
    uint32_t ev_peak_index_  = 0;
    int32_t  ev_max_         = 0;
    uint64_t ev_area_        = 0;
    uint32_t below_count_    = 0;   // length of the current run below the noise band: the
                                    // event's end test, and the window's dip tolerance in
                                    // IDLE (see update())
    uint32_t below_start_us_    = 0;   // where that run began: the sample an event ends
    uint32_t below_start_index_ = 0;
    // The window's opening edge: the sample the trace left the noise band on (see
    // update()'s IDLE case).  A window is *opened* by the trigger crossing but *starts*
    // here, so these are the event's own start - and what inEvent() reports to
    // EventAggregator, which times its shared window with it.
    bool     pend_valid_     = false;
    uint32_t pend_us_        = 0;      // timestamp of the opening edge
    uint32_t pend_index_     = 0;      // ... and its sample index
    uint64_t pend_area_      = 0;      // flank area in front of the crossing
    uint32_t pend_samples_   = 0;      // ... and how many samples that is
    float    path_           = 0.0f;   // the slope-bounded measurement path
    uint32_t clipped_        = 0;      // samples the path had to clip (diagnostics)
    uint32_t jumped_         = 0;      // clips where the raw jumped ahead inside the event
    uint32_t last_jump_index_ = 0;     // where the signal last jumped ahead of the path
    uint32_t discarded_      = 0;      // rise candidates suppressed (diagnostics)
    uint32_t event_counter_  = 0;
    int32_t  seed_[kQuietSeedMax];  // the quiet period's seed points
    uint16_t seed_n_         = 0;
    uint32_t seed_skip_      = 0;      // samples since the last seed point was taken
    ImpulseFilter filt_;            // input stage: drops lone noise spikes
    int32_t  filtered_       = 0;   // ... and the value it last produced
};
