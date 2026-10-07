// metrics_sink.h
//
// M4 -> M7 delivery of closed-event metrics over the RPC raw endpoint.
//
// One frame carries a whole event: the window plus one record per channel that is
// in SORTER_ACTIVE_CHANNEL_MASK.  The RPC.call dispatcher supports only a limited
// number of arguments, whereas an event has many fields per channel, so the record
// is sent as one raw frame - simple, cheap and lossless.
//
// Both cores compile this same header, so the frame has an identical layout on the
// M4 and the M7; magic + version let the receiver reject anything it does not
// understand, and the channel count inside the bundle fixes the payload length:
//
//   uint16 magic | uint8 version | uint8 pad
//   EventBundle   bundle                    (25 bytes)
//   ChannelMetrics ch[bundle.n_channels]    (27 bytes each)
//
// A 9-channel event is 272 bytes and the worst case (all 16 channels) is 461
// bytes - both inside the 512-byte RPMsg payload buffer (RPMSG_BUFFER_SIZE).

#pragma once

#include <Arduino.h>
#include <RPC.h>
#include <string.h>
#include <stddef.h>
#include "peak_metrics.h"

namespace sorter {

static const uint16_t BUNDLE_FRAME_MAGIC   = 0x4B42u; // 'BK' (bundle)
static const uint8_t  BUNDLE_FRAME_VERSION = 1u;

#pragma pack(push, 1)
struct BundleFrame {
    uint16_t       magic;
    uint8_t        version;
    uint8_t        pad;
    EventBundle    bundle;
    ChannelMetrics ch[SORTER_MAX_CHANNELS];
};
#pragma pack(pop)

// Payload length of a bundle with `n_channels` channel records.
inline size_t bundleBytes(uint8_t n_channels) {
    return offsetof(BundleFrame, ch) + (size_t)n_channels * sizeof(ChannelMetrics);
}

// M4 side: transmit one closed event (window + per-channel records) to the M7.
// Only the M4 acquisition thread transmits, so one static frame is enough.
inline bool sendBundle(const EventBundle& bundle, const ChannelMetrics* channels) {
    static BundleFrame frame;

    if (bundle.n_channels > (uint8_t)SORTER_MAX_CHANNELS) { return false; }

    frame.magic   = BUNDLE_FRAME_MAGIC;
    frame.version = BUNDLE_FRAME_VERSION;
    frame.pad     = 0;
    frame.bundle  = bundle;
    memcpy(frame.ch, channels, (size_t)bundle.n_channels * sizeof(ChannelMetrics));

    RPC.write(reinterpret_cast<const uint8_t*>(&frame), bundleBytes(bundle.n_channels), true);
    return true;
}

// M7 side: validate and copy an incoming raw frame into `out` (which must have
// room for SORTER_MAX_CHANNELS records).
inline bool parseBundle(const uint8_t* buf, size_t len, EventRecord& out) {
    if (len < offsetof(BundleFrame, ch)) { return false; }

    BundleFrame frame;
    memcpy(&frame, buf, offsetof(BundleFrame, ch));
    if (frame.magic != BUNDLE_FRAME_MAGIC || frame.version != BUNDLE_FRAME_VERSION) {
        return false;
    }
    if (frame.bundle.n_channels > (uint8_t)SORTER_MAX_CHANNELS) { return false; }
    if (len != bundleBytes(frame.bundle.n_channels)) { return false; }

    out.bundle = frame.bundle;
    memcpy(out.ch, buf + offsetof(BundleFrame, ch),
           (size_t)frame.bundle.n_channels * sizeof(ChannelMetrics));
    return true;
}

} // namespace sorter

