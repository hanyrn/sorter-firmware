// signal_gen.cpp
//
// Implementation of the bench-test synthetic signal generator (Cortex-M4).
//
// The DAC outputs are driven through the public Arduino API (analogWrite /
// analogWriteResolution), i.e. exactly the path wiring_analog.cpp takes for the
// GIGA's DAC pins, so no register access is needed here. If an application ever
// needs a much higher update rate, the two analogWrite() calls in writeSample()
// can be replaced by single stores to DAC1->DHR12R1 / DAC1->DHR12R2 - by then
// the channels are already enabled with their output buffers on.

#include "signal_gen.h"

#include <math.h>

namespace {

const float kPi = 3.14159265358979f;

// Raised-cosine pulse shape, sampled over kLutSize entries (0 .. 1).
const uint32_t kLutSize = 256u;
float          g_shape[kLutSize];

// ---------------------------------------------------------------------------
// Bench pattern - edit freely; it repeats every SIGNALGEN_CYCLE_SAMPLES.
//
// The pulse timings mirror the scenario in test/model_check.py: three broad
// peaks (which must come back as VALID) plus three deliberate false peaks that
// the detector bounds should reject. Pulses sit well inside the cycle and far
// apart, so no event straddles a cycle boundary.
//
//   start   width  amplitude  shape   expected detector verdict
//   ---------------------------------------------------------------
//    2000    1100       800   cosine  VALID
//    6000     900      1200   cosine  VALID
//   11000    1400       500   cosine  VALID
//   15000       4       900   rect    rejected: SHORT_FOR_HEIGHT
//   17000       2      4000   rect    rejected: WIDTH_TOO_NARROW
//   18500      20        60   rect    rejected: MAX_TOO_SMALL
//
// "amplitude" is in AD7606 counts above the baseline; it is divided by
// SIGNALGEN_ADC_PER_DAC to get DAC codes.
// ---------------------------------------------------------------------------
const sorter::PulseSpec kPattern[] = {
    {  2000,  1100,   800, 0 },   // P1 broad peak
    {  6000,   900,  1200, 0 },   // P2 broad peak
    { 11000,  1400,   500, 0 },   // P3 broad peak
    { 15000,     4,   900, 1 },   // G1 spike, too short for its height
    { 17000,     2,  4000, 1 },   // G2 spike, ends before the width bound
    { 18500,    20,    60, 1 },   // G3 small bump, below the amplitude bound
};
const uint32_t kPatternCount = sizeof(kPattern) / sizeof(kPattern[0]);

// Pulse amplitudes in DAC codes (converted once here, not per sample).
float g_pulseDac[kPatternCount];

} // namespace

void sorter::SignalGen::begin() {
    analogWriteResolution(12);   // analogWrite() now takes 0 .. 4095

    for (uint32_t i = 0; i < kLutSize; i++) {
        const float u = (float)i / (float)kLutSize;          // 0 .. 1
        g_shape[i] = 0.5f * (1.0f - cosf(2.0f * kPi * u));
    }
    for (uint32_t i = 0; i < kPatternCount; i++) {
        g_pulseDac[i] = (float)kPattern[i].amplitude / SIGNALGEN_ADC_PER_DAC;
    }

    writeSample(0);              // put the baseline on both outputs immediately
    ready_ = true;
}

void sorter::SignalGen::writeSample(uint32_t index) {
    float v = (float)SIGNALGEN_BASELINE_CODE + dither_ * (float)SIGNALGEN_NOISE_CODE;

    for (uint32_t i = 0; i < kPatternCount; i++) {
        const PulseSpec& p = kPattern[i];
        if (index < p.start || index >= (p.start + p.width)) {
            continue;
        }
        if (p.rect != 0u) {
            v += g_pulseDac[i];
        } else {
            const uint32_t u = ((index - p.start) * kLutSize) / p.width;
            v += g_pulseDac[i] * g_shape[u];
        }
    }

    // Stay inside the DAC output buffer's linear window.
    if (v < (float)SIGNALGEN_MIN_CODE) { v = (float)SIGNALGEN_MIN_CODE; }
    if (v > (float)SIGNALGEN_MAX_CODE) { v = (float)SIGNALGEN_MAX_CODE; }

    const int code = (int)lroundf(v);
    analogWrite(DAC_0, code);    // A12 = DAC1_OUT1 -> AD7606 module 0
    analogWrite(DAC_1, code);    // A13 = DAC1_OUT2 -> AD7606 module 1
}

void sorter::SignalGen::tick() {
    if (!ready_) {
        return;
    }

    if (enabled_) {
        // Dither so the detector sees a realistic, non-zero noise floor.
        // SIGNALGEN_NOISE_ALPHA = 1.0 gives white noise; lower values low-pass it.
        rng_ ^= rng_ << 13;
        rng_ ^= rng_ >> 17;
        rng_ ^= rng_ << 5;
        const float white = (float)(int32_t)rng_ * (1.0f / 2147483648.0f);  // -1 .. 1
        dither_ += SIGNALGEN_NOISE_ALPHA * (white - dither_);

        writeSample(pos_);
    }

    if (++pos_ >= SIGNALGEN_CYCLE_SAMPLES) {
        pos_ = 0;
        cycles_++;
    }
}
