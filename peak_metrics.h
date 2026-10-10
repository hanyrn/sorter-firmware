// peak_metrics.h
//
// Data types describing a single detected "peak" event, and the per-event bundle
// of per-channel records that the M4 sends to the M7.
//
// This header is deliberately free of any Arduino/hardware dependency so it can
// be shared verbatim by:
//   * the M4 firmware (producer of the metrics), and
//   * the M7 firmware (consumer of the metrics),
// as well as by the host-side unit test (test/test_event_detector.cpp).
//
// All fields use fixed-width types so the in-memory layout is identical on both
// cores of the STM32H747 and on the host; the wire structs are packed on top of
// that (see ChannelMetrics / EventBundle).

#pragma once

#include <stdint.h>

// Reason code stored in PeakMetrics::reason. REASON_OK means the event passed
// every bound and is considered a genuine peak. Anything else explains which
// bound rejected the candidate (i.e. why it was treated as a false peak).
enum PeakRejectReason : uint8_t {
    REASON_OK               = 0, // valid peak
    REASON_MAX_TOO_SMALL    = 1, // peak amplitude never rose far enough above baseline
    REASON_WIDTH_TOO_NARROW = 2, // event too brief (fewer than min_width_samples)
    REASON_WIDTH_TOO_WIDE   = 3, // event never returned to the noise band (timeout)
    REASON_AREA_TOO_SMALL   = 4, // integrated area below the minimum
    REASON_SHORT_FOR_HEIGHT = 5, // leading edge too short/too steep for its height:
                                 // a step (spike / flat glitch), not a peak
    REASON_RAILED           = 6, // input was at/above the converter rail (saturated),
                                 // so the reading is meaningless rather than a peak
};

struct PeakMetrics {
    uint32_t index;         // sequential event number (0,1,2,...)
    uint32_t start_us;      // the event's first sample: where the signal left its noise
                            // band (baseline + k_off*noise), which is the opening edge of
                            // the window the event's width is measured over
    uint32_t peak_us;       // timestamp of the maximum
    uint32_t width_us;      // the EXCURSION in microseconds: start -> the return to the
    uint32_t width_samples; // off-threshold, i.e. the span the measurement saw (the
                            // window the aggregator measures every other channel
                            // inside).  This is not the event's duration - see
                            // duration_us/duration_samples below
    uint32_t duration_us;      // the event's DURATION: the width the bounds are judged
    uint32_t duration_samples; // with and the number reported as the channel's width.
                               // The excursion above, corrected for the pulse's own
                               // envelope (EventDetector::envelopeDuration), because a
                               // faint pulse is cut off high up its flanks and its
                               // excursion under-reads its length by up to 40%.  Never
                               // shorter than the excursion it came from
    uint32_t samples;       // number of samples that were inside the event
    uint32_t rise_samples;  // samples from the event start to the maximum, i.e. the
                            // length of the leading edge (0 = the first sample IS the
                            // top: a step)

    int32_t  baseline;      // frozen baseline at the moment the event started
    int32_t  noise;         // frozen noise level (standard deviation) at event start
    int32_t  threshold;     // detection threshold that was used (baseline + k_on*noise)

    int32_t  max_value;     // peak amplitude above the baseline (per-channel counts).
                            // This is the *supported* apex: the signal was filtered on
                            // the way in (see ImpulseFilter), so a single-sample noise
                            // impulse can never be the value reported here.
    int32_t  max_abs_value; // absolute value of the peak (baseline + max_value)
    uint64_t area;          // integral of (signal - baseline) over the event

    uint8_t  valid;         // 1 = genuine peak, 0 = rejected as a false peak
    uint8_t  reason;        // PeakRejectReason
};

// ===========================================================================
// Per-event bundle: one event window + one record per channel
// ===========================================================================
// The M4 runs one EventDetector per AD7606 channel, so a single event produces
// one set of metrics for EVERY active channel.  EventAggregator (M4) fills an
// EventRecord below and metrics_sink.h ships it to the M7 as one raw frame.
//
// Two mode-dependent details (see SORTER_METRIC_MODE in config.h):
//   * LINKED      - all channels report the primary channel's width, and their
//                   height/area were measured inside the primary's window;
//   * PER_CHANNEL - every channel reports its own start/end/width, so the widths
//                   differ from channel to channel.
// ===========================================================================

static const uint8_t SORTER_MAX_CHANNELS = 16;  // channels in one event bundle

// How the per-channel records of an event were assembled.
enum MetricMode : uint8_t {
    METRIC_MODE_PER_CHANNEL = 0,  // each channel timed itself
    METRIC_MODE_LINKED      = 1,  // one primary channel timed the whole event
};

// Per-channel bits of ChannelMetrics::flags.
enum ChannelFlag : uint8_t {
    CH_FLAG_PRESENT   = 0x01, // channel rose above its own on-threshold in the window
    CH_FLAG_TRUNCATED = 0x02, // the window ended while this channel was still well
                              // inside its excursion: it had not dropped back below
                              // its own off-threshold for end_debounce consecutive
                              // samples, and at the window's last sample it was still
                              // half its hysteresis band above that threshold
                              // (LINKED), or the detector's max_width guard closed it
                              // (PER_CHANNEL).  Its peak was cut off, not measured.
                              // A channel whose tail merely trails the primary's by a
                              // few counts (two modules, one waveform) is NOT flagged:
                              // see accumulate() in event_aggregator.h.
    CH_FLAG_RAILED    = 0x04, // this channel's input was saturated (see REASON_RAILED);
                              // its detector was re-baselined and held off briefly
};

// Wire layout: packed, so both cores (and the host) agree byte for byte and a
// full 16-channel bundle still fits in one RPC raw frame.
#pragma pack(push, 1)

// One channel's contribution to an event.  `valid` is implied by the reason
// (valid == (reason == REASON_OK)), so one byte carries both.
struct ChannelMetrics {
    uint8_t  channel;       // AD7606 channel index (0..15)
    uint8_t  flags;         // ChannelFlag bits
    uint8_t  reason;        // PeakRejectReason for this channel
    int32_t  baseline;      // this channel's frozen baseline for the event
    int32_t  max_value;     // height above `baseline`
    uint32_t width_samples; // width in samples (LINKED: the primary's width)
    uint32_t width_us;      // width in microseconds
    uint64_t area;          // integral of the positive excursion above `baseline`
};  // 27 bytes

// One closed event: the window, plus how many channel records follow.
struct EventBundle {
    uint32_t index;         // sequential event number (0,1,2,...)
    uint32_t start_us;      // window start (the primary's opening edge: where it left its
                            // noise band - see PeakMetrics::start_us above)
    uint32_t end_us;        // window end
    uint32_t width_samples; // window width in samples
    uint32_t width_us;      // window width in microseconds
    uint8_t  primary;       // channel that timed the event (0xFF = none)
    uint8_t  mode;          // MetricMode
    uint8_t  n_channels;    // valid entries in EventRecord::ch
    uint8_t  valid;         // event verdict (the reference/primary channel's)
    uint8_t  reason;        // PeakRejectReason of the reference/primary channel
};  // 25 bytes

// Staging form shared by the M4 producer and the M7 receiver: a fixed array plus
// a count, so the raw frame payload has an identical layout on both cores.
struct EventRecord {
    EventBundle    bundle;
    ChannelMetrics ch[SORTER_MAX_CHANNELS];
};

#pragma pack(pop)

static_assert(sizeof(ChannelMetrics) == 27, "ChannelMetrics wire layout changed");
static_assert(sizeof(EventBundle) == 25, "EventBundle wire layout changed");
static_assert(sizeof(EventRecord) == 25 + 27 * SORTER_MAX_CHANNELS,
              "EventRecord wire layout changed");

// `valid` and `reason` are the same information, so derive one from the other
// instead of storing (and shipping) both.
inline bool metricsValid(uint8_t reason) { return reason == REASON_OK; }
