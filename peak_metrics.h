// peak_metrics.h
//
// Data types describing a single detected "peak" event.
//
// This header is deliberately free of any Arduino/hardware dependency so it can
// be shared verbatim by:
//   * the M4 firmware (producer of the metrics), and
//   * the M7 firmware (consumer of the metrics),
// as well as by the host-side unit test (test/test_event_detector.cpp).
//
// All fields use fixed-width types so the in-memory layout is identical on both
// cores of the STM32H747 and on the host.

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
    REASON_SHORT_FOR_HEIGHT = 5, // width too short *for its height* (spike/glitch)
};

struct PeakMetrics {
    uint32_t index;         // sequential event number (0,1,2,...)
    uint32_t start_us;      // timestamp when the signal first crossed the on-threshold
    uint32_t peak_us;       // timestamp of the maximum
    uint32_t width_us;      // total event duration in microseconds (start -> return)
    uint32_t width_samples; // total event duration in samples
    uint32_t samples;       // number of samples that were inside the event

    int32_t  baseline;      // frozen baseline at the moment the event started
    int32_t  noise;         // frozen noise level (standard deviation) at event start
    int32_t  threshold;     // detection threshold that was used (baseline + k_on*noise)

    int32_t  max_value;     // peak amplitude above the baseline (combined-signal units)
    int32_t  max_abs_value; // absolute value of the peak (baseline + max_value)
    uint64_t area;          // integral of (signal - baseline) over the event

    uint8_t  valid;         // 1 = genuine peak, 0 = rejected as a false peak
    uint8_t  reason;        // PeakRejectReason
};
