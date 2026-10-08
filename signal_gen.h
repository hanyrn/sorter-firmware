// signal_gen.h
//
// Cortex-M4 bench-test signal source: drives the GIGA's two 12-bit DAC outputs
// (A12 = DAC1_OUT1, A13 = DAC1_OUT2) so the generated waveform can be looped
// straight back into the AD7606 analog inputs.  That turns the complete chain
// (parallel bus -> combine -> event detector -> RPC -> M7 printout) into a
// hardware-in-the-loop test without any external signal source.
//
// The waveform itself - a train of randomly parameterised pulses on a DC baseline
// with dither - lives in signal_model.h (Arduino-free, unit-tested, mirrored in
// Python).  This class is only the DAC side of it: it owns the model and writes
// each generated code to A12/A13 with analogWrite().
//
// tick() is called once per acquisition iteration, i.e. the generator is
// *sample-locked* to the ADC: one DAC update per conversion, with no timer and no
// interrupt.  Because each DAC code is held for a whole sample period, a
// conversion digitises the previous sample's settled value - a deterministic
// one-sample delay that does not affect any peak metric.
//
// Pulse amplitudes are expressed in AD7606 counts and converted to DAC codes with
// SIGNALGEN_ADC_PER_DAC (see config.h).

#pragma once

#include <stdint.h>
#include "config.h"
#include "signal_model.h"

namespace sorter {

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

    // The DAC code both outputs hold, i.e. the sample just generated.  This is
    // also what the sample stream reports as the "expected" trace.
    int32_t  code()     const { return model_.code(); }

    uint32_t position() const { return model_.position(); }  // sample index in the cycle
    uint32_t cycles()   const { return model_.cycles(); }    // completed pattern cycles

    // The pulse train currently in use (redrawn at every cycle start), so a host
    // tool can print/plot exactly what is being injected.
    const PulseSpec* pulses()     const { return model_.pulses(); }
    uint32_t         pulseCount() const { return model_.pulseCount(); }

private:
    void writeOutputs(int32_t code);

    SignalModel model_;
    bool        ready_   = false;
    bool        enabled_ = true;
};

} // namespace sorter
