// adc7606_parallel.h
//
// Cortex-M4 driver for two AD7606 modules wired as TWO 16-bit parallel buses
// with shared control lines (CONVST/RESET/RD/CS tied together, see config.h).
//
// readWord() latches one module's DB0..DB15 straight from the GPIO input
// registers through AD7606_DB_LINES[], so the 32 data lines may sit on any free
// GIGA pins - they do not have to be contiguous and no three-state timing is
// involved. Only the control lines are asserted/released with the port
// bit-set/reset register (BSRR), which is a single store.

#pragma once

#include <Arduino.h>
#include "config.h"

class Adc7606Parallel {
public:
    // Configure all data/control pins and put them in their idle state.
    // Call once before reset()/readAll(). Must run on the M4 core.
    void begin();

    // Pulse the RESET line (polarity per AD7606_RESET_ACTIVE_HIGH) so both
    // modules start from a known state. Call once after begin().
    void reset();

    // Start a simultaneous conversion on both modules (shared CONVST), then read
    // all 16 channels during one shared CS burst:
    //   channels[0..7]  -> module 0 (on its own DB0..DB15)
    //   channels[8..15] -> module 1 (on its own DB0..DB15)
    // Channel n of both modules is latched in the SAME RD cycle, so the 16
    // values are one snapshot of the same instant. Values are signed 16-bit
    // two's-complement, as produced by the AD7606.
    void readAll(int32_t channels[AD7606_NUM_CHANNELS]);

    // Convenience: convert + read + combine into a single sample the channels
    // selected by SORTER_ACTIVE_CHANNEL_MASK (mean or sum, per
    // SORTER_COMBINE_MEAN).
    int32_t readCombined();

private:
    void convstPulse();
    void waitReady();
};
