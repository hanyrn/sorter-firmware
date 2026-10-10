// event_aggregator.h
//
// Groups the per-channel samples read from the two AD7606 modules into ONE event
// record.
//
// event_detector.h detects events in a single signal, so EventAggregator owns one
// EventDetector per channel: every channel gets its own baseline, noise floor and
// threshold, and therefore its own height / width / area.  What the M7 receives
// per event is:
//
//   * the event window (start / end / width) plus the event verdict, and
//   * one record per active channel (height, area, width, baseline, verdict).
//
// How the per-channel results are tied together into one event is selectable with
// the mode (SORTER_METRIC_MODE in config.h):
//
//   METRIC_MODE_LINKED (default)
//       The primary channel (SORTER_PRIMARY_CHANNEL) times the event: it opens the
//       shared window on the sample its trace leaves its own noise band (where the
//       event it is about to report starts - see EventDetector::inEvent()) and
//       closes it when it falls back into the band.  Every active channel's height
//       and area are measured INSIDE that window only - a channel that rises
//       earlier or later is clipped to it - and every channel reports the primary's
//       width, so one event has exactly one width.  A channel that never rises is
//       reported as absent.
//
//   METRIC_MODE_PER_CHANNEL
//       Every channel is timed on its own: own start, own end, own width, so the
//       widths legitimately differ from channel to channel.  The event is reported
//       once ALL channels that entered it have closed again; a channel that never
//       drops back is closed by the detector's own max_width_samples guard, so an
//       event can never hang.
//
// This file has no Arduino dependency (same as event_detector.h) and is therefore
// unit-testable on a host (test/test_event_aggregator.cpp).
//
// One detail is shared by both modes: the samples are pushed through the same
// impulse filter the single-channel detector uses (see ImpulseFilter in
// event_detector.h) before anything is measured, so the height and the leading
// edge a channel reports cannot be set by a lone noise spike.
//
// Usage (M4 acquisition loop):
//   EventAggregator agg(makeAggregatorConfig());
//   EventRecord     rec;
//   for (;;) {
//       g_adc.readAll(channels);
//       if (agg.update(channels, micros(), index, rec)) {
//           sendBundle(rec.bundle, rec.ch);      // metrics_sink.h
//       }
//       index++;
//   }

#pragma once

#include <stdint.h>
#include <math.h>
#include "peak_metrics.h"
#include "event_detector.h"

// Per-channel input polarity - applied to the raw AD7606 snapshot before any
// detection.  Some channels (the primary / 9th channel here, index 8) idle HIGH
// and DIP when an event occurs, i.e. they are the complement of the other
// channels.  Negating such a channel turns its dip into a rise, so the very same
// rising-edge peak detector and the same bounds work unchanged; its reported
// height and area then read as the dip depth rather than a rise.  Bit i of
// `invert_mask` selects channel i.
inline void applyInputPolarity(int32_t* channels, uint32_t invert_mask) {
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if ((invert_mask & (1UL << ch)) != 0UL) { channels[ch] = -channels[ch]; }
    }
}

class EventAggregator {
public:
    struct Config {
        EventDetector::Config detector;       // tuning shared by all 16 detectors
        uint32_t channel_mask = 0x0000FFFFu;  // channels that are measured/reported
        MetricMode mode       = METRIC_MODE_LINKED;
        uint8_t  primary      = 0xFFu;        // window reference; 0xFF = first active

        // ---- saturation ("rail") guard ------------------------------------
        // A sample with |x| >= rail_level is beyond the ADC range, where the
        // reading clips at full scale and nothing can be measured.  A channel is
        // declared railed after rail_width consecutive saturated samples; on that
        // sample its detector is re-baselined and the channel is held out of
        // detection for rail_cooldown samples.  The event/link is notified once
        // per rail episode (REASON_RAILED + CH_FLAG_RAILED) so the operator knows
        // the source is too bright.
        int32_t  rail_level    = 32000;       // |sample| >= this = saturated (counts)
        uint32_t rail_width    = 8;           // consecutive saturated samples to rail
        uint32_t rail_cooldown = 1000;        // samples held out of detection after rail
    };

    EventAggregator() { configure(Config()); }
    explicit EventAggregator(const Config& c) { configure(c); }

    // Apply a configuration (or reconfiguration) and clear all state.
    void configure(const Config& c);
    void reset();

    // Feed one simultaneous snapshot of every AD7606 channel.
    //   channels : AD7606_NUM_CHANNELS signed sample values
    //   t_us     : timestamp of the snapshot, in microseconds
    //   index    : monotonically increasing snapshot index (width in samples)
    //   out      : filled when this call completes an event
    // Returns true exactly when an event bundle has just been completed.
    bool update(const int32_t* channels, uint32_t t_us, uint32_t index, EventRecord& out);

    // Channel that times the event (resolved from Config::primary and the mask).
    uint8_t primaryChannel() const { return primary_; }

private:
    bool inMask(uint8_t ch) const { return (cfg_.channel_mask & (1UL << ch)) != 0UL; }

    // `skip[ch]` = true for a channel that is railed / held off, so the mode
    // helpers feed it no samples and never let it time or extend an event.
    bool updateLinked(const int32_t* channels, uint32_t t_us, uint32_t index,
                      EventRecord& out, const bool* skip);
    bool updatePerChannel(const int32_t* channels, uint32_t t_us, uint32_t index,
                          EventRecord& out, const bool* skip);
    // PER_CHANNEL: true when at least one channel of the group that just finished has
    // metrics to report.  A group in which every channel's candidate was suppressed has
    // nothing to report and is dropped (see updatePerChannel).
    bool anyPcDone() const;

    void openWindow();                                   // LINKED: freeze the references
    void accumulate(uint8_t ch, int32_t x, uint32_t index);  // LINKED: clip one channel
    void buildLinkedRecord(const PeakMetrics& ref, EventRecord& out);
    void buildPerChannelRecord(uint32_t t_us, uint32_t index, EventRecord& out);
    void buildRailRecord(uint32_t t_us, EventRecord& out);  // saturation notification

    Config   cfg_;
    uint8_t  primary_       = 0xFFu;
    uint32_t event_counter_ = 0;

    EventDetector det_[SORTER_MAX_CHANNELS];
    // Each channel's input stage (see ImpulseFilter): the window measures the same
    // impulse-free stream the channel's detector judges.  Fed for every masked
    // channel on every sample, before any detection.
    ImpulseFilter filt_[SORTER_MAX_CHANNELS];

    // ---- saturation ("rail") guard -------------------------------------------------
    uint32_t rail_count_[SORTER_MAX_CHANNELS]   = {0};  // consecutive saturated samples
    uint32_t rail_holdoff_[SORTER_MAX_CHANNELS] = {0};  // samples left out of detection
    bool     rail_skip_[SORTER_MAX_CHANNELS]    = {false};
    bool     ch_railed_[SORTER_MAX_CHANNELS]    = {false}; // railed since last recovery
    bool     rail_notified_ = false;                    // one notification per episode

    // ---- LINKED: the shared window, and the per-channel values measured in it ----
    bool     win_open_ = false;
    int32_t  win_base_[SORTER_MAX_CHANNELS];  // frozen baseline at the window start
    int32_t  win_on_[SORTER_MAX_CHANNELS];    // frozen on-threshold (base + k_on*sigma)
    int32_t  win_off_[SORTER_MAX_CHANNELS];   // frozen off-threshold (base + k_off*sigma)
    int32_t  win_max_[SORTER_MAX_CHANNELS];   // height above win_base_[]
    uint32_t win_max_ix_[SORTER_MAX_CHANNELS]; // index of that maximum
    uint32_t win_rose_ix_[SORTER_MAX_CHANNELS];// index it crossed its on-threshold
    uint64_t win_area_[SORTER_MAX_CHANNELS];  // integral above win_base_[]
    bool     win_rose_[SORTER_MAX_CHANNELS];  // crossed its own on-threshold
    bool     win_high_[SORTER_MAX_CHANNELS];  // still well inside its excursion at the
                                              // last sample (see accumulate() /
                                              // CH_FLAG_TRUNCATED)
    uint8_t  win_below_[SORTER_MAX_CHANNELS]; // consecutive samples below the off-threshold
    bool     win_back_[SORTER_MAX_CHANNELS];  // ... long enough to close the excursion
    float    win_path_[SORTER_MAX_CHANNELS];  // slope-bounded measurement path (see
                                              // accumulate())
    uint32_t win_jump_ix_[SORTER_MAX_CHANNELS];  // where this channel's signal last jumped
                                              // ahead of that path (see accumulate())

    // ---- PER_CHANNEL: the group of channels whose events are being collected ----
    uint8_t      pc_pending_  = 0;            // channels currently inside their own event
    uint32_t     pc_start_us_ = 0;            // when the first channel of the group opened
    uint32_t     pc_start_ix_ = 0;
    PeakMetrics  pc_metrics_[SORTER_MAX_CHANNELS];
    bool         pc_done_[SORTER_MAX_CHANNELS];
    bool         pc_in_[SORTER_MAX_CHANNELS]; // channel is inside the open group
};

// ===========================================================================
// implementation
// ===========================================================================

inline void EventAggregator::configure(const Config& c) {
    cfg_ = c;

    // Resolve the window reference: the configured channel when it is part of the
    // mask, else the first channel of the mask, else none (no channel selected).
    primary_ = 0xFFu;
    if (cfg_.channel_mask != 0u) {
        if (c.primary < SORTER_MAX_CHANNELS && inMask(c.primary)) {
            primary_ = c.primary;
        } else {
            for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
                if (inMask(ch)) { primary_ = ch; break; }
            }
        }
    }

    reset();
}

inline void EventAggregator::reset() {
    event_counter_ = 0;
    win_open_      = false;
    pc_pending_    = 0;
    pc_start_us_   = 0;
    pc_start_ix_   = 0;
    rail_notified_ = false;

    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        win_base_[ch] = 0;
        win_on_[ch]   = 0;
        win_off_[ch]  = 0;
        win_max_[ch]  = 0;
        win_max_ix_[ch]  = 0;
        win_rose_ix_[ch] = 0;
        win_below_[ch] = 0;
        win_back_[ch]  = false;
        win_path_[ch]  = 0.0f;
        win_jump_ix_[ch] = 0;
        win_area_[ch] = 0;
        win_rose_[ch] = false;
        win_high_[ch] = false;
        pc_done_[ch]  = false;
        pc_in_[ch]    = false;
        rail_count_[ch]   = 0;
        rail_holdoff_[ch] = 0;
        rail_skip_[ch]    = false;
        ch_railed_[ch]    = false;
    }
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        det_[ch].configure(cfg_.detector);
        filt_[ch].configure(cfg_.detector.impulse_filter_taps);
    }
}

inline bool EventAggregator::update(const int32_t* channels, uint32_t t_us,
                                    uint32_t index, EventRecord& out) {
    // ---- saturation ("rail") guard: runs before any detection -----------------
    // A channel at/above the converter rail reads a clipped, meaningless value.
    // Such a channel is taken out of detection: while it stays bright it is held,
    // and once the signal returns to range the (now stale) reference is discarded
    // and re-seeded from the settled sample, so detection resumes cleanly instead
    // of computing events from a saturated baseline.
    bool rail_new = false;   // a channel was declared railed on THIS sample
    bool any_busy = false;   // some channel is currently railed / held off

    // ---- input conditioning ----------------------------------------------------
    // Every masked channel's sample is pushed through its own impulse filter (see
    // ImpulseFilter) before anything is measured, exactly like the channel's own
    // detector does.  This runs unconditionally - even for a channel that is railed
    // or held out of detection - so the filter always follows the live stream and
    // can never hand out a value from before the event it is measuring.  The window
    // therefore reports the same spike-free signal the detectors judge: a lone
    // impulse can neither open/close a window nor become a channel's height.
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (!inMask(ch)) { continue; }
        filt_[ch].step(channels[ch]);
    }

    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        rail_skip_[ch] = false;
        if (!inMask(ch)) { continue; }

        const int32_t mag = (channels[ch] < 0) ? -channels[ch] : channels[ch];

        if (rail_holdoff_[ch] > 0u) {
            rail_skip_[ch] = true;   // held out of detection
            any_busy       = true;
            if (mag >= cfg_.rail_level) {
                rail_holdoff_[ch] = cfg_.rail_cooldown;   // still bright: keep holding
            } else if (--rail_holdoff_[ch] == 0u) {
                // Settled: the reference is re-established from a fresh quiet period
                // (the detector discards the stale one and re-seeds itself - see
                // EventDetector::restartQuietPeriod).  Detection on this channel is
                // blind for that period, which is why the hold-off above comes first.
                det_[ch].restartQuietPeriod();
            }
            continue;
        }

        if (mag >= cfg_.rail_level) {
            rail_skip_[ch] = true;   // nothing valid to detect in this sample
            any_busy       = true;
            if (++rail_count_[ch] >= cfg_.rail_width) {
                rail_count_[ch]   = 0u;
                rail_holdoff_[ch] = cfg_.rail_cooldown;
                ch_railed_[ch]    = true;
                rail_new          = true;
            }
        } else {
            rail_count_[ch] = 0u;
        }
    }

    const bool closed = (cfg_.mode == METRIC_MODE_LINKED)
                            ? updateLinked(channels, t_us, index, out, rail_skip_)
                            : updatePerChannel(channels, t_us, index, out, rail_skip_);

    // ---- rail notification -----------------------------------------------------
    // Exactly one bundle per rail episode: the first sample that declares a rail
    // reports it (SIGNAL TOO BRIGHT) and any open window is flushed; the
    // notification re-arms only once every channel has recovered, so a stuck
    // bright source cannot spam the link.
    bool rail_emit = false;
    if (rail_new && !rail_notified_) {
        rail_notified_ = true;
        if (cfg_.mode == METRIC_MODE_LINKED) { win_open_ = false; }  // drop the window
        buildRailRecord(t_us, out);
        rail_emit = true;
    }
    if (!any_busy) {   // all channels back in range: re-arm and forget the episode
        rail_notified_ = false;
        for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) { ch_railed_[ch] = false; }
    }

    const bool emit = closed || rail_emit;
    if (emit) { out.bundle.index = event_counter_++; }
    return emit;
}

// ---------------------------------------------------------------------------
// LINKED: one channel (the primary) times the event for everybody
// ---------------------------------------------------------------------------
inline bool EventAggregator::updateLinked(const int32_t* channels, uint32_t t_us,
                                          uint32_t index, EventRecord& out,
                                          const bool* skip) {
    if (primary_ == 0xFFu) { return false; }   // nothing selected: nothing to time

    // Every active channel keeps its own tracker running, so all baselines and
    // noise floors stay valid.  In this mode only the primary's event state is
    // used, so the other detectors' own results are ignored.  A railed/held-off
    // channel is skipped: its samples are not fed to its detector at all.
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (ch == primary_ || !inMask(ch) || skip[ch]) { continue; }
        PeakMetrics ignored;
        det_[ch].update(channels[ch], t_us, index, ignored);
    }

    // A railed primary cannot time the window: any open window is flushed as a
    // rail notification by update(), so just report "nothing closed" here.
    if (skip[primary_]) { return false; }

    const bool was_in = det_[primary_].inEvent();
    PeakMetrics ref;
    const DetectorResult prim = det_[primary_].update(channels[primary_], t_us, index, ref);
    const bool now_in         = det_[primary_].inEvent();

    if (!win_open_) {
        if (!was_in && now_in) {
            // The primary's window just opened - it left its noise band on this sample,
            // which is where the event it may become starts (see EventDetector::inEvent()
            // and the IDLE case in its update()).  This sample opens the shared window and
            // is the first sample every channel is measured from, so the window contains
            // the leading edge of every channel - not the trigger crossing and not the
            // sample the rise test happens to pass.
            openWindow();
            accumulate(primary_, filt_[primary_].value(), index);
        }
        return false;
    }

    if (prim == DET_DISCARDED) {
        // The primary's rise candidate failed its rise test, so what looked like the
        // start of an event was a spike, a blip or a dropout.  The window goes with it
        // and nothing at all is reported for this excursion (the "suppress the noise
        // event" rule - see DetectorResult).
        win_open_ = false;
        return false;
    }

    // The window above was opened where the primary's trace left the noise band (see
    // EventDetector::inEvent()), and a breakout that never became an event - the trace
    // dipped back into the band for longer than the dip tolerance, so the detector
    // dropped its window and no candidate ever started - leaves this one holding a window
    // the detector no longer has.  It goes with the primary's: nothing is reported for it
    // and the next breakout opens a fresh one.  (A candidate that was suppressed is
    // handled above; a window that *did* become an event closes through the DET_CLOSED
    // path below - where `was_in` is true and `now_in` false too, so the verdict is what
    // tells the two apart.)
    if (prim != DET_CLOSED && was_in && !now_in) {
        win_open_ = false;
        return false;
    }

    // Window open: measure every active channel inside it.
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (inMask(ch)) { accumulate(ch, filt_[ch].value(), index); }
    }

    if (prim != DET_CLOSED) { return false; }

    // The primary's event is over (it settled back to its baseline, or was closed by
    // the max_width guard): that is the end of the window for everybody.
    buildLinkedRecord(ref, out);
    win_open_ = false;
    return true;
}

inline void EventAggregator::openWindow() {
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        win_max_[ch]  = 0;
        win_area_[ch] = 0;
        win_rose_[ch] = false;
        win_high_[ch] = false;
        win_below_[ch] = 0;
        win_back_[ch]  = false;
        win_max_ix_[ch]  = 0;
        win_rose_ix_[ch] = 0;

        if (!inMask(ch)) { continue; }

        // Freeze each channel's reference at the window start - the same thing the
        // single detector does when its own window opens, and with the *same*
        // expression (the rounded baseline, then the thresholds off it), so a channel
        // that the detector closed is also seen as closed here.  Rounding the float
        // mean twice instead could shift a threshold by one count and make the
        // primary's CH_FLAG_TRUNCATED disagree with its own event.  (The detector
        // latches its own copy of the same expression on the trigger crossing, a sample
        // or two later; the baseline tracker's gate holds the mean between the two - a
        // trace outside the noise band does not update it - so the two agree in all but
        // a count.)
        const int32_t base  = (int32_t)lroundf(det_[ch].baseline());
        const float   sigma = det_[ch].sigma();
        win_base_[ch] = base;
        win_on_[ch]   = (int32_t)lroundf((float)base + cfg_.detector.k_on * sigma);
        win_off_[ch]  = (int32_t)lroundf((float)base + cfg_.detector.k_off * sigma);
        // The channel's slope-bounded measurement path for this window, starting on
        // the sample the window opened on (see accumulate()).
        win_path_[ch] = (float)filt_[ch].value();
        win_jump_ix_[ch] = 0;
    }
    win_open_ = true;
}

inline void EventAggregator::accumulate(uint8_t ch, int32_t x, uint32_t index) {
    // The window measures the same slope-bounded signal the channel's own detector
    // measures (see "Bounded slope" in event_detector.h): no single sample can set this
    // channel's height for the window, and a lone impulse can neither open its
    // excursion nor close it early.  Same bound as the detector's, so a channel judged
    // here agrees with the same channel judged on its own.
    const float prev      = win_path_[ch];
    const float step      = (float)x - prev;
    const bool  in_event  = ((int32_t)lroundf(prev) >= win_off_[ch]);
    if (step > cfg_.detector.max_slope)       { win_path_[ch] = prev + cfg_.detector.max_slope; }
    else if (step < -cfg_.detector.max_slope) { win_path_[ch] = prev - cfg_.detector.max_slope; }
    else                                      { win_path_[ch] = (float)x; }
    const int32_t p = (int32_t)lroundf(win_path_[ch]);
    // ... and the same "was the apex spiked into existence" judgement the channel's own
    // detector makes, with the same rule, so a channel judged here and the same channel
    // judged on its own agree (see EventDetector::measure / apexUnsettled).
    const bool jumped = (step > cfg_.detector.max_slope && in_event);

    const int32_t amp = p - win_base_[ch];
    if (amp > win_max_[ch]) {
        win_max_[ch]    = amp;
        win_max_ix_[ch] = index;
    }
    if (jumped && index <= win_max_ix_[ch]) { win_jump_ix_[ch] = index; }
    if (amp > 0)            { win_area_[ch] += (uint64_t)amp; }
    if (!win_rose_[ch]) {
        // First crossing inside the window: this is where the channel joins the
        // measurement, and the sample the leading edge is counted from.
        if (p > win_on_[ch]) {
            win_rose_[ch]    = true;
            win_rose_ix_[ch] = index;
            win_below_[ch]   = 0;
            win_back_[ch]    = false;
        }
    } else if (win_back_[ch] && p > win_on_[ch]) {
        // The channel closed its excursion and crossed again (a noise blip followed
        // by the real peak, say): the window now holds a *new* excursion, and *that*
        // one is what may still be open when the window ends.
        win_below_[ch] = 0;
        win_back_[ch]  = false;
    }

    // CH_FLAG_TRUNCATED = the window ended while the channel was still well inside
    // its (latest) excursion.  "Back down" is decided exactly like the detector
    // closes an event (cfg.detector.end_debounce consecutive samples below the
    // off-threshold) and the verdict latches, so a noise run near the window's edge
    // cannot reopen an excursion that really did finish inside the window.
    //
    // "Well inside" and not "below the off-threshold at the last sample": the two
    // AD7606 modules are separate converters with their own offsets and noise
    // floors, so the *same* waveform crosses two channels' off-thresholds a few
    // samples apart.  Without the margin below, every event would flag the channel
    // whose tail happens to be a count or two higher - `[truncated]` on a healthy
    // measurement (see the module-mismatch case in test/test_bench_detect.cpp).
    // The margin is half the channel's own hysteresis band (k_on - k_off) * sigma,
    // i.e. the level at which the signal is no longer "in the band" but "in the
    // event"; a channel that really was cut off mid-peak (a second peak rising
    // inside the window) sits far above it.
    if (win_rose_[ch] && !win_back_[ch]) {
        if (p < win_off_[ch]) {
            if (win_below_[ch] < cfg_.detector.end_debounce) { win_below_[ch]++; }
            if (win_below_[ch] >= cfg_.detector.end_debounce) { win_back_[ch] = true; }
        } else {
            win_below_[ch] = 0;
        }
    }
    const int32_t band = (win_on_[ch] - win_off_[ch]) / 2;
    win_high_[ch] = (win_rose_[ch] && !win_back_[ch] && (p > win_off_[ch] + band));
}

inline void EventAggregator::buildLinkedRecord(const PeakMetrics& ref, EventRecord& out) {
    uint8_t n = 0;
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (!inMask(ch)) { continue; }

        // Built in a local first: out.ch belongs to a packed struct, so its elements
        // sit at odd addresses and must not be bound to a reference.
        ChannelMetrics cm;
        cm.channel       = ch;
        cm.flags         = 0;
        cm.baseline      = win_base_[ch];
        cm.max_value     = win_max_[ch];
        // One event, one width - and it is the DURATION the primary's detector judged
        // with, not the window: every channel inherits the same number, because the
        // primary timed the event for all of them (see EventDetector::envelopeDuration).
        // The bundle below keeps the window (ref.width_samples/width_us), which is the
        // span those channels were measured inside.
        cm.width_samples = ref.duration_samples;
        cm.width_us      = ref.duration_us;
        cm.area          = win_area_[ch];

        if (win_rose_[ch]) {
            cm.flags |= CH_FLAG_PRESENT;
            if (win_high_[ch]) { cm.flags |= CH_FLAG_TRUNCATED; }  // still high at window end
            // Judged with the same bounds as any other peak, but on values that were
            // measured inside the primary's window - and with the *inherited* width
            // above, which is the primary's judged duration (the event's length as the
            // machine estimated it; see EventDetector::envelopeDuration).  The leading
            // edge is this channel's own: from the sample it crossed its on-threshold
            // to the sample it peaked at.  A maximum that came *before* the crossing
            // (the channel was still falling when it joined the window, so only its
            // tail was captured) counts as a step and is let through as 0 - the shape
            // guard in classifyPeak then judges it like any other step.
            const uint32_t rise = (win_max_ix_[ch] >= win_rose_ix_[ch])
                                      ? (win_max_ix_[ch] - win_rose_ix_[ch])
                                      : 0u;
            const uint32_t unsettled =
                ((win_max_ix_[ch] - win_jump_ix_[ch]) < cfg_.detector.min_rise_samples)
                    ? 1u : 0u;
            cm.reason = EventDetector::classifyPeak(cfg_.detector, cm.max_value,
                                                    cm.width_samples, cm.area, rise,
                                                    unsettled);
        } else {
            // Never rose inside the window: there is nothing to measure here.
            cm.reason = REASON_MAX_TOO_SMALL;
        }

        out.ch[n++] = cm;
    }

    out.bundle.start_us      = ref.start_us;
    out.bundle.end_us        = ref.start_us + ref.width_us;
    out.bundle.width_samples = ref.width_samples;
    out.bundle.width_us      = ref.width_us;
    out.bundle.primary       = primary_;
    out.bundle.mode          = (uint8_t)METRIC_MODE_LINKED;
    out.bundle.n_channels    = n;
    out.bundle.valid         = ref.valid;
    out.bundle.reason        = ref.reason;
}

// ---------------------------------------------------------------------------
// PER_CHANNEL: every channel times itself, the bundle waits for the last one
// ---------------------------------------------------------------------------
inline bool EventAggregator::updatePerChannel(const int32_t* channels, uint32_t t_us,
                                              uint32_t index, EventRecord& out,
                                              const bool* skip) {
    // One bit per channel (0..15): must hold SORTER_MAX_CHANNELS bits, so a
    // uint8_t would silently drop channels 8..15 (module 1).
    uint16_t opened  = 0;  // channels whose own event started on this sample
    uint16_t closed  = 0;  // channels whose own event ended on this sample
    uint16_t dropped = 0;  // channels whose rise candidate was suppressed on this sample,
                           // or whose window closed without ever becoming one

    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (!inMask(ch)) { continue; }

        if (skip[ch]) {
            // Railed/held off: the channel cannot close on its own, so drop it from
            // the open group (its own rail bundle is emitted by update()) to keep
            // the group from hanging.
            if (pc_in_[ch]) { pc_in_[ch] = false; if (pc_pending_ > 0u) { pc_pending_--; } }
            continue;
        }

        const bool was_in = det_[ch].inEvent();
        PeakMetrics m;
        const DetectorResult r = det_[ch].update(channels[ch], t_us, index, m);
        if (!was_in && det_[ch].inEvent()) { opened |= (uint16_t)(1u << ch); pc_in_[ch] = true; }
        if (r == DET_CLOSED) {
            closed |= (uint16_t)(1u << ch);
            pc_metrics_[ch] = m;    // this channel's own start/end/width/height/area
            pc_in_[ch]      = false;
        } else if (r == DET_DISCARDED) {
            // Its rise candidate failed, so this channel was never in an event: it
            // has to leave the group, or the group would keep waiting for an event
            // that does not exist.  Nothing is recorded for it either - a suppressed
            // candidate is not even a rejected peak.
            dropped |= (uint16_t)(1u << ch);
            if (pc_in_[ch]) { pc_in_[ch] = false; if (pc_pending_ > 0u) { pc_pending_--; } }
        } else if (was_in && !det_[ch].inEvent()) {
            // The channel's own window closed without an event: what opened it was a band
            // breakout that never became one (the trace dipped back into the band for
            // longer than the dip tolerance, so the detector dropped the window and no
            // candidate ever started).  The channel was never in an event, so it leaves
            // the group exactly like a suppressed candidate above - and the group is
            // complete on the sample the last channel leaves it.
            dropped |= (uint16_t)(1u << ch);
            if (pc_in_[ch]) { pc_in_[ch] = false; if (pc_pending_ > 0u) { pc_pending_--; } }
        }
    }

    // Closing channels first: a channel whose event ends on this sample belongs to
    // the group that is ending, not to the one that starts on the same sample.  A
    // channel whose candidate was suppressed finishes its part of the group here too.
    bool group_done = false;
    if (closed != 0u || dropped != 0u) {
        for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
            if ((closed & (uint16_t)(1u << ch)) == 0u) { continue; }
            pc_done_[ch] = true;
            if (pc_pending_ > 0u) { pc_pending_--; }
        }
        group_done = (pc_pending_ == 0u);
    }

    if (group_done) {
        // Every channel of the group was suppressed: there is no event to report, so
        // the bundle is dropped instead of being sent out empty (the same "suppress
        // the noise event" rule the detectors apply - see DetectorResult).
        if (!anyPcDone()) {
            pc_pending_ = 0u;
            return false;
        }
        buildPerChannelRecord(t_us, index, out);
    }

    // Opening channels form the next group (pc_pending_ is 0 again after an event
    // was reported, so the first of them starts the new window).
    if (opened != 0u) {
        for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
            if ((opened & (uint16_t)(1u << ch)) == 0u) { continue; }
            if (pc_pending_ == 0u) { pc_start_us_ = t_us; pc_start_ix_ = index; }
            pc_pending_++;
        }
    }

    return group_done;
}

// True when the group that just finished has at least one channel with metrics.
inline bool EventAggregator::anyPcDone() const {
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (pc_done_[ch]) { return true; }
    }
    return false;
}

inline void EventAggregator::buildPerChannelRecord(uint32_t t_us, uint32_t index,
                                                   EventRecord& out) {
    uint8_t n          = 0;
    uint8_t verdict_ch = 0xFFu;   // channel whose verdict becomes the event verdict

    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (!inMask(ch)) { continue; }

        ChannelMetrics cm;
        cm.channel       = ch;
        cm.flags         = 0;
        cm.baseline      = 0;
        cm.max_value     = 0;
        cm.width_samples = 0;
        cm.width_us      = 0;
        cm.area          = 0;
        cm.reason        = REASON_MAX_TOO_SMALL;

        if (pc_done_[ch]) {
            const PeakMetrics& m = pc_metrics_[ch];
            cm.baseline      = m.baseline;
            cm.max_value     = m.max_value;
            cm.width_samples = m.duration_samples;   // its own duration: widths may differ
            cm.width_us      = m.duration_us;
            cm.area          = m.area;
            cm.flags         = CH_FLAG_PRESENT;
            cm.reason        = m.reason;
            if (m.reason == REASON_WIDTH_TOO_WIDE) { cm.flags |= CH_FLAG_TRUNCATED; }
            // The primary channel speaks for the event when it took part.
            if (verdict_ch == 0xFFu || ch == primary_) { verdict_ch = ch; }
        }
        pc_done_[ch] = false;   // ready for the next group
        out.ch[n++] = cm;
    }

    out.bundle.start_us      = pc_start_us_;
    out.bundle.end_us        = t_us;
    out.bundle.width_samples = index - pc_start_ix_;   // the span all channels fit in
    out.bundle.width_us      = t_us - pc_start_us_;
    out.bundle.primary       = (verdict_ch == 0xFFu) ? 0xFFu : verdict_ch;
    out.bundle.mode          = (uint8_t)METRIC_MODE_PER_CHANNEL;
    out.bundle.n_channels    = n;
    if (verdict_ch != 0xFFu) {
        out.bundle.valid  = metricsValid(pc_metrics_[verdict_ch].reason) ? 1u : 0u;
        out.bundle.reason = pc_metrics_[verdict_ch].reason;
    } else {
        out.bundle.valid  = 0u;
        out.bundle.reason = REASON_MAX_TOO_SMALL;
    }

    pc_pending_ = 0;
}

// ---------------------------------------------------------------------------
// RAIL: notification that a channel's input is saturated (source too bright)
// ---------------------------------------------------------------------------
// Emitted once per rail episode.  It carries one record per masked channel so the
// frame keeps the usual shape; only the channel(s) that railed are flagged
// (CH_FLAG_RAILED + REASON_RAILED).  The event verdict is not valid - this is a
// "fix the input" message, not a measurement.
inline void EventAggregator::buildRailRecord(uint32_t t_us, EventRecord& out) {
    uint8_t n = 0;
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (!inMask(ch)) { continue; }

        ChannelMetrics cm;
        cm.channel       = ch;
        cm.flags         = 0;
        cm.baseline      = (int32_t)lroundf(det_[ch].baseline());
        cm.max_value     = 0;
        cm.width_samples = 0;
        cm.width_us      = 0;
        cm.area          = 0;
        cm.reason        = REASON_MAX_TOO_SMALL;
        if (ch_railed_[ch]) {
            cm.flags  = CH_FLAG_RAILED;
            cm.reason = REASON_RAILED;
        }
        out.ch[n++] = cm;
    }

    out.bundle.start_us      = t_us;
    out.bundle.end_us        = t_us;
    out.bundle.width_samples = 0;
    out.bundle.width_us      = 0;
    out.bundle.primary       = primary_;
    out.bundle.mode          = (uint8_t)cfg_.mode;
    out.bundle.n_channels    = n;
    out.bundle.valid         = 0;
    out.bundle.reason        = REASON_RAILED;
}

