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
//       shared window when it rises above its own on-threshold and closes it when
//       it falls back below.  Every active channel's height and area are measured
//       INSIDE that window only - a channel that rises earlier or later is clipped
//       to it - and every channel reports the primary's width, so one event has
//       exactly one width.  A channel that never rises is reported as absent.
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

class EventAggregator {
public:
    struct Config {
        EventDetector::Config detector;       // tuning shared by all 16 detectors
        uint32_t channel_mask = 0x0000FFFFu;  // channels that are measured/reported
        MetricMode mode       = METRIC_MODE_LINKED;
        uint8_t  primary      = 0xFFu;        // window reference; 0xFF = first active
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

    bool updateLinked(const int32_t* channels, uint32_t t_us, uint32_t index, EventRecord& out);
    bool updatePerChannel(const int32_t* channels, uint32_t t_us, uint32_t index, EventRecord& out);

    void openWindow();                                   // LINKED: freeze the references
    void accumulate(uint8_t ch, int32_t x);              // LINKED: clip one channel
    void buildLinkedRecord(const PeakMetrics& ref, EventRecord& out);
    void buildPerChannelRecord(uint32_t t_us, uint32_t index, EventRecord& out);

    Config   cfg_;
    uint8_t  primary_       = 0xFFu;
    uint32_t event_counter_ = 0;

    EventDetector det_[SORTER_MAX_CHANNELS];

    // ---- LINKED: the shared window, and the per-channel values measured in it ----
    bool     win_open_ = false;
    int32_t  win_base_[SORTER_MAX_CHANNELS];  // frozen baseline at the window start
    int32_t  win_on_[SORTER_MAX_CHANNELS];    // frozen on-threshold (base + k_on*sigma)
    int32_t  win_off_[SORTER_MAX_CHANNELS];   // frozen off-threshold (base + k_off*sigma)
    int32_t  win_max_[SORTER_MAX_CHANNELS];   // height above win_base_[]
    uint64_t win_area_[SORTER_MAX_CHANNELS];  // integral above win_base_[]
    bool     win_rose_[SORTER_MAX_CHANNELS];  // crossed its own on-threshold
    bool     win_high_[SORTER_MAX_CHANNELS];  // above its own off-threshold, last sample

    // ---- PER_CHANNEL: the group of channels whose events are being collected ----
    uint8_t      pc_pending_  = 0;            // channels currently inside their own event
    uint32_t     pc_start_us_ = 0;            // when the first channel of the group opened
    uint32_t     pc_start_ix_ = 0;
    PeakMetrics  pc_metrics_[SORTER_MAX_CHANNELS];
    bool         pc_done_[SORTER_MAX_CHANNELS];
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

    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        win_base_[ch] = 0;
        win_on_[ch]   = 0;
        win_off_[ch]  = 0;
        win_max_[ch]  = 0;
        win_area_[ch] = 0;
        win_rose_[ch] = false;
        win_high_[ch] = false;
        pc_done_[ch]  = false;
    }
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        det_[ch].configure(cfg_.detector);
    }
}

inline bool EventAggregator::update(const int32_t* channels, uint32_t t_us,
                                    uint32_t index, EventRecord& out) {
    const bool closed = (cfg_.mode == METRIC_MODE_LINKED)
                            ? updateLinked(channels, t_us, index, out)
                            : updatePerChannel(channels, t_us, index, out);
    if (closed) { out.bundle.index = event_counter_++; }
    return closed;
}

// ---------------------------------------------------------------------------
// LINKED: one channel (the primary) times the event for everybody
// ---------------------------------------------------------------------------
inline bool EventAggregator::updateLinked(const int32_t* channels, uint32_t t_us,
                                          uint32_t index, EventRecord& out) {
    if (primary_ == 0xFFu) { return false; }   // nothing selected: nothing to time

    // Every active channel keeps its own tracker running, so all baselines and
    // noise floors stay valid.  In this mode only the primary's event state is
    // used, so the other detectors' own results are ignored.
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (ch == primary_ || !inMask(ch)) { continue; }
        PeakMetrics ignored;
        det_[ch].update(channels[ch], t_us, index, ignored);
    }

    const bool was_in = det_[primary_].inEvent();
    PeakMetrics ref;
    const bool ref_closed = det_[primary_].update(channels[primary_], t_us, index, ref);
    const bool now_in     = det_[primary_].inEvent();

    if (!win_open_) {
        if (!was_in && now_in) {
            // The primary just crossed: this sample opens the shared window and is
            // the first sample every channel is measured from.
            openWindow();
            accumulate(primary_, channels[primary_]);
        }
        return false;
    }

    // Window open: measure every active channel inside it.
    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (inMask(ch)) { accumulate(ch, channels[ch]); }
    }

    if (!ref_closed) { return false; }

    // The primary's event is over (it decayed below its off-threshold, or was
    // closed by the max_width guard): that is the end of the window for everybody.
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

        if (!inMask(ch)) { continue; }

        // Freeze each channel's reference at the window start - the same thing the
        // single detector does when its own event begins.
        const float base  = det_[ch].baseline();
        const float sigma = det_[ch].sigma();
        win_base_[ch] = (int32_t)lroundf(base);
        win_on_[ch]   = (int32_t)lroundf(base + cfg_.detector.k_on * sigma);
        win_off_[ch]  = (int32_t)lroundf(base + cfg_.detector.k_off * sigma);
    }
    win_open_ = true;
}

inline void EventAggregator::accumulate(uint8_t ch, int32_t x) {
    const int32_t amp = x - win_base_[ch];
    if (amp > win_max_[ch]) { win_max_[ch] = amp; }
    if (amp > 0)            { win_area_[ch] += (uint64_t)amp; }
    if (!win_rose_[ch] && x > win_on_[ch]) { win_rose_[ch] = true; }
    win_high_[ch] = (x >= win_off_[ch]);
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
        // One event, one width: the primary timed it, so every channel inherits it.
        cm.width_samples = ref.width_samples;
        cm.width_us      = ref.width_us;
        cm.area          = win_area_[ch];

        if (win_rose_[ch]) {
            cm.flags |= CH_FLAG_PRESENT;
            if (win_high_[ch]) { cm.flags |= CH_FLAG_TRUNCATED; }  // still high at window end
            // Judged with the same bounds as any other peak, but on values that were
            // measured inside the primary's window.
            cm.reason = EventDetector::classifyPeak(cfg_.detector, cm.max_value,
                                                    cm.width_samples, cm.area);
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
                                              uint32_t index, EventRecord& out) {
    // One bit per channel (0..15): must hold SORTER_MAX_CHANNELS bits, so a
    // uint8_t would silently drop channels 8..15 (module 1).
    uint16_t opened = 0;  // channels whose own event started on this sample
    uint16_t closed = 0;  // channels whose own event ended on this sample

    for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
        if (!inMask(ch)) { continue; }
        const bool was_in = det_[ch].inEvent();
        PeakMetrics m;
        const bool is_closed = det_[ch].update(channels[ch], t_us, index, m);
        if (!was_in && det_[ch].inEvent()) { opened |= (uint16_t)(1u << ch); }
        if (is_closed) {
            closed |= (uint16_t)(1u << ch);
            pc_metrics_[ch] = m;    // this channel's own start/end/width/height/area
        }
    }

    // Closing channels first: a channel whose event ends on this sample belongs to
    // the group that is ending, not to the one that starts on the same sample.
    bool group_done = false;
    if (closed != 0u) {
        for (uint8_t ch = 0; ch < SORTER_MAX_CHANNELS; ch++) {
            if ((closed & (uint16_t)(1u << ch)) == 0u) { continue; }
            pc_done_[ch] = true;
            if (pc_pending_ > 0u) { pc_pending_--; }
        }
        group_done = (pc_pending_ == 0u);
    }

    if (group_done) { buildPerChannelRecord(t_us, index, out); }

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
            cm.width_samples = m.width_samples;   // its own width: widths may differ
            cm.width_us      = m.width_us;
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
