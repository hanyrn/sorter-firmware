// metrics_sink.h
//
// M4 -> M7 delivery of closed-event metrics over the RPC raw endpoint.
//
// Why the raw endpoint (and not RPC.call): the RPC function dispatcher supports
// only a limited number of arguments, whereas a peak record has many fields.
// Sending the whole struct as one raw frame is simple, cheap and lossless.
//
// Both cores compile this same header, so PeakFrame has an identical layout on
// the M4 and the M7; magic + version let the receiver reject anything it does
// not understand.

#pragma once

#include <Arduino.h>
#include <RPC.h>
#include <string.h>
#include "peak_metrics.h"

namespace sorter {

static const uint16_t PEAK_FRAME_MAGIC   = 0x4B50u; // 'PK'
static const uint8_t  PEAK_FRAME_VERSION = 1u;

struct PeakFrame {
    uint16_t    magic;
    uint8_t     version;
    uint8_t     pad;
    PeakMetrics metrics;
};

// M4 side: transmit one closed event to the M7.
inline void sendPeak(const PeakMetrics& m) {
    PeakFrame frame;
    frame.magic   = PEAK_FRAME_MAGIC;
    frame.version = PEAK_FRAME_VERSION;
    frame.pad     = 0;
    frame.metrics = m;
    RPC.write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame), true);
}

// M7 side: validate and copy an incoming raw frame.
inline bool parsePeak(const uint8_t* buf, size_t len, PeakMetrics& out) {
    if (len != sizeof(PeakFrame)) {
        return false;
    }
    PeakFrame frame;
    memcpy(&frame, buf, sizeof(frame));
    if (frame.magic != PEAK_FRAME_MAGIC || frame.version != PEAK_FRAME_VERSION) {
        return false;
    }
    out = frame.metrics;
    return true;
}

} // namespace sorter
