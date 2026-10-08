// stream_sink.h
//
// M4 -> M7 delivery of raw waveform samples, over the same RPC raw endpoint as the
// event bundles (metrics_sink.h).  This is what the live plot window draws: the
// generator's own output (the "expected" trace - in the bench loopback that is
// literally the voltage the AD7606 digitises) together with what the detector saw
// on the first SORTER_STREAM_CHANNELS active channels.
//
// The two frame kinds are told apart by their magic, so the receiver can simply
// dispatch on it:
//
//   uint16 magic | uint8 version | uint8 n_channels | uint16 decim
//   uint16 count | uint32 first_index | uint8 channels[8]
//   StreamPoint point[count]
//
// with StreamPoint = { int16 gen; int16 adc[SORTER_STREAM_CHANNELS]; }.  The
// channel byte array is always the full 8 entries (only the first n_channels are
// meaningful) so that the point array stays at a fixed offset on both cores - six
// wasted bytes per frame is cheaper than a variable layout.
//
// At the default 64 points / 2 channels a frame is 404 bytes, inside the
// 512-byte RPMsg payload buffer (RPMSG_BUFFER_SIZE) - the static_assert below
// keeps it that way when SORTER_STREAM_* is retuned.
//
// `adc` values are post input-polarity (config.h), i.e. exactly the samples the
// detector works on, and `gen` is the DAC code written for that sample.

#pragma once

#include <Arduino.h>
#include <RPC.h>
#include <string.h>
#include <stddef.h>
#include "config.h"

namespace sorter {

static const uint16_t STREAM_FRAME_MAGIC   = 0x5453u; // 'ST' (sample stream)
static const uint8_t  STREAM_FRAME_VERSION = 1u;

// Hard bounds for the frame layout.  Macros on purpose: the #if guards below run in
// the preprocessor, where a `static const` would silently read as 0.
#define STREAM_MAX_CHANNELS 8u    // channel indices fit in one byte
#define STREAM_MAX_POINTS   128u  // hard bound for the frame's point array

#if SORTER_STREAM_CHANNELS > STREAM_MAX_CHANNELS
#error "SORTER_STREAM_CHANNELS must be <= STREAM_MAX_CHANNELS (8)"
#endif
#if SORTER_STREAM_POINTS > STREAM_MAX_POINTS
#error "SORTER_STREAM_POINTS must be <= STREAM_MAX_POINTS (128)"
#endif
#if SORTER_STREAM_DECIM > 65535u
#error "SORTER_STREAM_DECIM must fit in uint16 (the decim field is 16-bit)"
#endif

struct StreamHeader {
    uint16_t magic;
    uint8_t  version;
    uint8_t  n_channels;
    uint16_t decim;        // samples per point (1 = every sample)
    uint16_t count;        // points in this frame (<= SORTER_STREAM_POINTS)
    uint32_t first_index;  // sample index of point[0]
    uint8_t  channels[STREAM_MAX_CHANNELS];  // 0xFF = unused entry
};

struct StreamPoint {
    int16_t gen;                          // generated DAC code (drives the ADC)
    int16_t adc[SORTER_STREAM_CHANNELS];  // measured value per channel
};

#pragma pack(push, 1)
struct StreamFrame {
    StreamHeader header;
    StreamPoint  point[SORTER_STREAM_POINTS];
};
#pragma pack(pop)

static_assert(sizeof(StreamHeader) + SORTER_STREAM_POINTS * sizeof(StreamPoint) <= 512u,
              "a stream frame must fit the 512-byte RPMSG payload buffer");

// Payload length of a frame carrying `count` points.
inline size_t streamBytes(uint16_t count) {
    return offsetof(StreamFrame, point) + (size_t)count * sizeof(StreamPoint);
}

// The k-th channel of SORTER_ACTIVE_CHANNEL_MASK, i.e. exactly the channels the
// streamed points carry.  Both cores compute the same list, and the M7 prints it,
// so the host tools never have to guess which column is which channel.
inline uint8_t streamChannelIndex(uint8_t k) {
    uint8_t seen = 0;
    for (uint8_t ch = 0; ch < (uint8_t)SORTER_MAX_CHANNELS; ch++) {
        if (SORTER_ACTIVE_CHANNEL_MASK & (1u << ch)) {
            if (seen == k) { return ch; }
            seen++;
        }
    }
    return 0xFFu;
}

// M4 side: offer one acquisition sample to the stream.  Every
// SORTER_STREAM_DECIM-th sample is kept; when SORTER_STREAM_POINTS points have
// accumulated the frame goes to the M7.  Only the acquisition thread calls this,
// so the accumulating frame can be static (same trick as sendBundle()).
inline void streamSample(uint32_t index, int32_t gen, const int32_t* adc) {
    static StreamFrame frame;
    static uint16_t    count = 0;

    if (count == 0u) {
        frame.header.magic       = STREAM_FRAME_MAGIC;
        frame.header.version     = STREAM_FRAME_VERSION;
        frame.header.n_channels  = (uint8_t)SORTER_STREAM_CHANNELS;
        frame.header.decim       = (uint16_t)SORTER_STREAM_DECIM;
        frame.header.count       = 0;
        frame.header.first_index = index;
        for (uint8_t k = 0; k < STREAM_MAX_CHANNELS; k++) {
            frame.header.channels[k] = (k < (uint8_t)SORTER_STREAM_CHANNELS)
                                        ? streamChannelIndex(k) : 0xFFu;
        }
    }

    StreamPoint& p = frame.point[count];
    p.gen = (int16_t)gen;
    for (uint8_t k = 0; k < (uint8_t)SORTER_STREAM_CHANNELS; k++) {
        p.adc[k] = (int16_t)adc[k];
    }

    if (++count >= (uint16_t)SORTER_STREAM_POINTS) {
        frame.header.count = count;
        RPC.write(reinterpret_cast<const uint8_t*>(&frame), streamBytes(count), true);
        count = 0;
    }
}

// M7 side: validate an incoming raw frame and copy header + points into `out`
// (which is always full size; only out.header.count points are meaningful).
inline bool parseStreamFrame(const uint8_t* buf, size_t len, StreamFrame& out) {
    if (len < offsetof(StreamFrame, point)) { return false; }

    StreamHeader header;
    memcpy(&header, buf, sizeof(header));
    if (header.magic != STREAM_FRAME_MAGIC || header.version != STREAM_FRAME_VERSION) {
        return false;
    }
    if (header.n_channels > (uint8_t)STREAM_MAX_CHANNELS) { return false; }
    if (header.count == 0u || header.count > (uint16_t)SORTER_STREAM_POINTS) { return false; }
    if (len != streamBytes(header.count)) { return false; }

    out.header = header;
    memcpy(out.point, buf + offsetof(StreamFrame, point),
           (size_t)header.count * sizeof(StreamPoint));
    return true;
}

} // namespace sorter
