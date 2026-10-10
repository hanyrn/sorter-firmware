// signal_gen.cpp
//
// DAC side of the bench-test signal generator (Cortex-M4).  See signal_gen.h for
// what it is for and signal_model.h for the waveform itself.
//
// The outputs are driven through the public Arduino API (analogWrite /
// analogWriteResolution), i.e. exactly the path wiring_analog.cpp takes for the
// GIGA's DAC pins, so no register access is needed here. If an application ever
// needs a much higher update rate, the two analogWrite() calls in writeOutputs()
// can be replaced by single stores to DAC1->DHR12R1 / DAC1->DHR12R2 - by then
// the channels are already enabled with their output buffers on.

#include "signal_gen.h"

namespace {

// Board-side scalars for the waveform model, taken from config.h.  The pulse
// statistics (heights, widths, gaps, glitch rate) are passed the same way: they
// are documented in config.h and defaulted in SignalModel::Config.
sorter::SignalModel::Config makeConfig() {
    sorter::SignalModel::Config c;

    c.cycle_samples = SIGNALGEN_CYCLE_SAMPLES;
    c.max_pulses    = SIGNALGEN_MAX_PULSES;
    c.cycle_margin  = SIGNALGEN_CYCLE_MARGIN;

    c.gap_min = SIGNALGEN_GAP_MIN;
    c.gap_max = SIGNALGEN_GAP_MAX;

    c.merge_pct          = SIGNALGEN_MERGE_PCT;
    c.merged_span_min    = SIGNALGEN_MERGED_SPAN_MIN;
    c.merged_span_max    = SIGNALGEN_MERGED_SPAN_MAX;
    c.merge_min_overlap  = SIGNALGEN_MERGE_MIN_OVERLAP;

    c.width_mean = SIGNALGEN_WIDTH_MEAN;
    c.width_sd   = SIGNALGEN_WIDTH_SD;
    c.width_min  = SIGNALGEN_WIDTH_MIN;
    c.width_max  = SIGNALGEN_WIDTH_MAX;
    c.width_span_pct = SIGNALGEN_WIDTH_SPAN_PCT;

    c.height_mean     = SIGNALGEN_HEIGHT_MEAN;
    c.height_sd       = SIGNALGEN_HEIGHT_SD;
    c.height_span_pct = SIGNALGEN_HEIGHT_SPAN_PCT;
    c.height_min      = SIGNALGEN_HEIGHT_MIN;
    c.height_max      = SIGNALGEN_HEIGHT_MAX;

    c.frontier_pct        = SIGNALGEN_FRONTIER_PCT;
    c.frontier_width_min  = SIGNALGEN_FRONTIER_WIDTH_MIN;
    c.frontier_width_max  = SIGNALGEN_FRONTIER_WIDTH_MAX;
    c.frontier_trigger    = SIGNALGEN_FRONTIER_TRIGGER;
    c.frontier_rise       = SIGNALGEN_FRONTIER_RISE;
    c.frontier_margin_min = SIGNALGEN_FRONTIER_MARGIN_MIN;
    c.frontier_margin_max = SIGNALGEN_FRONTIER_MARGIN_MAX;

    c.ladder        = SIGNALGEN_LADDER;
    c.ladder_width  = SIGNALGEN_LADDER_WIDTH;
    c.ladder_min    = SIGNALGEN_LADDER_MIN;
    c.ladder_step   = SIGNALGEN_LADDER_STEP;
    c.ladder_count  = SIGNALGEN_LADDER_COUNT;

    c.glitch_pct         = SIGNALGEN_GLITCH_PCT;
    c.glitch_width_max   = SIGNALGEN_GLITCH_WIDTH_MAX;
    c.glitch_height_mean = SIGNALGEN_GLITCH_HEIGHT_MEAN;
    c.glitch_height_sd   = SIGNALGEN_GLITCH_HEIGHT_SD;
    c.glitch_height_min  = SIGNALGEN_GLITCH_HEIGHT_MIN;
    c.glitch_height_max  = SIGNALGEN_GLITCH_HEIGHT_MAX;

    c.baseline_code = SIGNALGEN_BASELINE_CODE;
    c.noise_code    = SIGNALGEN_NOISE_CODE;
    c.noise_alpha   = SIGNALGEN_NOISE_ALPHA;
    c.baseline_wave_code    = SIGNALGEN_BASELINE_WAVE_CODE;
    c.baseline_wave_samples = SIGNALGEN_BASELINE_WAVE_SAMPLES;
    c.spike_pct     = SIGNALGEN_SPIKE_PCT;
    c.spike_max     = SIGNALGEN_SPIKE_MAX;
    c.min_code      = SIGNALGEN_MIN_CODE;
    c.max_code      = SIGNALGEN_MAX_CODE;
    c.adc_per_dac   = SIGNALGEN_ADC_PER_DAC;
    c.seed          = SIGNALGEN_SEED;
    return c;
}

} // namespace

void sorter::SignalGen::begin() {
    analogWriteResolution(12);   // analogWrite() now takes 0 .. 4095

    model_.configure(makeConfig());
    model_.begin();
    writeOutputs(model_.code()); // put the baseline on both outputs immediately
    ready_ = true;
}

void sorter::SignalGen::writeOutputs(int32_t code) {
    const int value = (int)code;
    analogWrite(DAC_0, value);   // A12 = DAC1_OUT1 -> AD7606 module 0
    analogWrite(DAC_1, value);   // A13 = DAC1_OUT2 -> AD7606 module 1
}

void sorter::SignalGen::tick() {
    if (!ready_) {
        return;
    }

    // The pattern clock keeps running even while the generator is muted (the DACs
    // simply hold their last code), which is how this behaved before the
    // waveform model existed.
    const int32_t code = model_.tick();
    if (enabled_) {
        writeOutputs(code);
    }
}
