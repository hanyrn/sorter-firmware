// adc7606_parallel.h
//
// Cortex-M4 driver for two AD7606 modules on a shared 16-bit parallel bus.
//
// The bus is read with direct register accesses (see readBus()) so the whole
// 16-bit word is latched with two loads from GPIOJ/GPIOK, keeping the per-sample
// overhead low. Only the control lines are asserted/released with the port
// bit-set/reset register (BSRR), which is also a single store.

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

    // Start a simultaneous conversion on both modules (shared CONVST) and read
    // all 16 channels:
    //   channels[0..7]  -> module 0 (CS0)
    //   channels[8..15] -> module 1 (CS1)
    // Values are signed 16-bit two's-complement, as produced by the AD7606.
    void readAll(int32_t channels[AD7606_NUM_CHANNELS]);

    // Convenience: convert + read + combine all channels into a single sample
    // (mean or sum, per SORTER_COMBINE_MEAN).
    int32_t readCombined();

private:
    static inline uint16_t readBus();
    void convstPulse();
    void waitReady();
    void readModule(uint8_t moduleIndex, int32_t* out, uint8_t base);
};
