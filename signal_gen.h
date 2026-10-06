// signal_gen.h
//
// Cortex-M4 synthetic signal generator for bench-testing the event detector.
//
// It drives the GIGA's two 12-bit DAC outputs (A12 = DAC1_OUT1, A13 =
// DAC1_OUT2) with a repeating, fully known waveform so that signal can be looped
// straight back into the AD7606 analog inputs.  That makes the complete chain
// (parallel bus -> combine -> event detector -> RPC -> M7 printout) testable on
// real hardware without any external signal source.
//
// tick() is called once per acquisition iteration, i.e. the generator is
// *sample-locked* to the ADC: one DAC update per conversion, with no timer and
// no interrupt.  Because each DAC code is held for a whole sample period, a
// conversion digitises the previous sample's settled value - a deterministic
// one-sample delay that does not affect any peak metric.
//
// The waveform is a train of raised-cosine pulses (broad peaks) plus
// rectangular spikes (deliberate false peaks) on a DC baseline with dither.
// Amplitudes are expressed in AD7606 counts and converted to DAC codes with
// SIGNALGEN_ADC_PER_DAC (see config.h).

#pragma once

#include <stdint.h>
#include "config.h"

namespace sorter {

// One injected pulse. Amplitudes are AD7606 counts above the baseline.
struct PulseSpec {
    uint32_t start;      // first sample of the pulse inside the pattern cycle
    uint32_t width;      // duration in samples
    int32_t  amplitude;  // peak height above the baseline (AD7606 counts)
    uint8_t  rect;       // 0 = raised-cosine (peak), 1 = rectangle (glitch)
};

class SignalGen {
public:
    // Configure both DAC outputs and put the baseline on them. Call once, on the
    // M4, before the acquisition loop starts.
    void begin();

    // Advance one sample and update both DAC outputs. Call once per ADC
    // conversion, immediately *before* the conversion is started.
    void tick();

    void     enable(bool on) { enabled_ = on; }
    bool     enabled()  const { return enabled_; }
    uint32_t position() const { return pos_; }     // sample index within the cycle
    uint32_t cycles()   const { return cycles_; }  // completed pattern cycles

private:
    void writeSample(uint32_t index);

    uint32_t pos_     = 0;
    uint32_t cycles_  = 0;
    uint32_t rng_     = SIGNALGEN_SEED;
    float    dither_  = 0.0f;   // normalised (-1..1) dither state
    bool     enabled_ = true;
    bool     ready_   = false;
};

} // namespace sorter
