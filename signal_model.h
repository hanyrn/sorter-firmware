// signal_model.h
//
// Arduino-free, deterministic model of the bench-test synthetic waveform: a train
// of randomly parameterised pulses on a DC baseline with dither and impulse noise.
//
// Why this is a header of its own (and not part of signal_gen.cpp): exactly like
// event_detector.h / peak_metrics.h it has *no* Arduino dependency, so the very
// same code
//   * runs on the M4 to feed the two DACs (signal_gen.cpp does the analogWrite),
//   * is unit-tested on the host (test/test_signal_model.cpp), and
//   * is mirrored in Python (test/signal_model.py) for the offline plot
//     (test/plot_signals.py) and the live datastream viewer (test/live_plot.py).
// Only the *waveform maths* lives here; every board-side scalar is passed in
// through Config, which signal_gen.cpp fills from config.h.
//
// ---------------------------------------------------------------------------
// Randomisation
// ---------------------------------------------------------------------------
// The pulse train is not a fixed table: a new train is drawn at the start of every
// pattern cycle, so over a few cycles the detector meets a wide variety of peak
// shapes instead of the same six pulses for ever.  The train is deliberately a
// *hard* one - neighbours overlap, peaks span the whole dynamic range, and the
// baseline carries spikes that look like peaks:
//
//   gap between pulses      uniform in [gap_min, gap_max] samples
//   merged pairs            merge_pct % of the pulses are placed *inside* the
//                           previous one instead of after it, so that the two add up
//                           into one two-humped event spanning merged_span_min..max
//                           nominal widths: clearly wider than any single pulse, so
//                           the event-width distribution grows a second mode (and,
//                           because a merged pulse never merges again, pairs are
//                           never chained into longer events)
//   peak width (cosine)     Gaussian(width_mean, width_sd)   clipped to bounds,
//                           except for width_span_pct % of the peaks, which are
//                           drawn log-uniform over [width_min, width_mean].  That
//                           slice is the *narrow* end of the range - down to
//                           width_min, which is 3.0 ms at 10 kSPS - because the
//                           shortest event the machine can be asked to measure is a
//                           real requirement, not a tail case the Gaussian is too
//                           thin to ever produce.  It never reaches past
//                           width_mean, so a single pulse still cannot be as wide
//                           as a merged pair and the event-width distribution stays
//                           bimodal (see test_signal_model.cpp, check "double
//                           peaked") - that is also why width_min is the *only*
//                           thing this slice widens.
//   peak height             Gaussian(height_mean, height_sd) clipped to bounds,
//                           except for height_span_pct % of the peaks, which are
//                           drawn log-uniform over the whole [height_min,
//                           height_max] span - from an apex that barely clears
//                           k_on*sigma up to one that drives the DAC into its clamp
//                           (a saturated, flat-topped peak)
//   small-peak slice        frontier_pct % of the peaks take the small-peak style -
//                           and with it the apex from frontierApex(), the smallest
//                           one the detector's rise test can accept at the width the
//                           pulse was drawn with, times frontier_margin_min..max -
//                           when that width lands in [frontier_width_min,
//                           frontier_width_max].  Height and width are drawn
//                           independently above, so a small apex is rare and usually
//                           lands where being small does not matter; the corner the
//                           bounds decide (a small apex on a pulse wide enough for
//                           DET_MIN_RISE_SAMPLES to be the bound that decides) would
//                           otherwise never be visited.  See config.h's
//                           SIGNALGEN_FRONTIER_* block for the measured floor and for
//                           why the band stops where it does.
//   floor ladder            with `ladder` = 1 none of the above is drawn at all: the
//                           pattern becomes a *staircase* of ladder_count isolated
//                           pulses of one width, apexes stepping by ladder_step from
//                           ladder_min - the measured floor, presented instead of
//                           buried in a shoulder (see Config::ladder and config.h's
//                           SIGNALGEN_LADDER_* block).  Off by default: the random
//                           train above is the bench, the ladder is the probe.
//   glitch width (rect)     uniform in [1, glitch_width_max] samples
//   glitch height           Gaussian(glitch_height_mean, glitch_height_sd)
//   glitch probability      glitch_pct % of the pulses
//   dither                  white noise (or low-passed with noise_alpha), +/- noise_code
//   baseline wander         the DC level the pulses sit on is itself moving:
//                           baseline_wave_code DAC codes peak, over
//                           baseline_wave_samples samples (30 s at 10 kSPS).  A
//                           real front end drifts (thermal, supply, a slow
//                           background), and what has to cope with it is the
//                           detector's baseline tracker: a steady DC offset is
//                           tracked and cancels out of height/width/area, while a
//                           drift the tracker cannot follow would look like one
//                           enormous event.  The wave runs off a *global* sample
//                           counter, never off the position inside the pattern
//                           cycle, so it is continuous across the cycle boundary
//                           and across a whole capture
//   impulses                spike_pct % of the samples carry a single-sample impulse
//                           of uniform height in +/- spike_max - much bigger than
//                           the dither, i.e. the false peaks the width, max-value
//                           and slope bounds have to reject
//
// Gaussian because that is what a real signal looks like: most peaks are close to
// the nominal size, a few are much larger or smaller.  The log-uniform slice on top
// guarantees the *extremes* are represented in every cycle, which a Gaussian alone
// cannot do (the tails are too thin): some peaks barely clear the noise band, others
// saturate the source.  The rectangular glitches and the single-sample impulses are
// kept so the false-peak bounds (DET_MIN_MAX_VALUE, DET_MIN_WIDTH_SAMPLES,
// DET_MAX_SLOPE) keep being exercised - a random train must still show the detector
// rejecting what it should reject.
//
// The *draw order is fixed and documented* so the Python mirror can reproduce the
// train exactly: buildTrain() draws, per pulse, gap/overlap distance, kind, overlap
// decision, u1, u2, glitch-width, height, height-style, width-gate and width-style
// (ten uniforms, always, whatever the branch), and Box-Muller turns the (u1, u2) pair
// into two independent standard normals (width, nominal height).  The height-style
// draw is the one the small-peak slice rides on: it picks between the frontier,
// log-uniform and Gaussian height (and, for the frontier, re-uses the height and
// width-style uniforms as its margin and its log-uniform width).  tick() draws per
// sample: one white() sample for the dither and one uniform for the impulse gate,
// plus a second white() for the impulse's height when it fires, so no draw has to be
// cached between samples.  The baseline wander draws nothing at all: it is a pure
// function of the global sample index.
//
// Float note: the firmware computes in float32, the Python mirror in float64, so a
// drawn height/width can differ by one count in the last digit.  That does not
// matter for a plot, and every *decision* (how many pulses, how many glitches,
// where they land, which samples carry an impulse) uses the same integers on both
// sides.

#pragma once

#include <stdint.h>
#include <math.h>

namespace sorter {

static const float kPi = 3.14159265358979f;

// One pulse of a cycle's train.  `amplitude` is the peak height above the
// baseline in AD7606 counts (the same unit the detector reports in), so it can be
// compared with the printed max= / the Config bounds directly.
struct PulseSpec {
    uint32_t start;      // first sample of the pulse inside the pattern cycle
    uint32_t width;      // duration in samples
    int32_t  amplitude;  // peak height above the baseline (AD7606 counts)
    uint8_t  rect;       // 0 = raised-cosine (peak), 1 = rectangle (glitch)
};

inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// The smallest apex a raised cosine of `width` samples can have and still show
// `rise_samples` samples of leading edge above a trigger of `trigger` counts - i.e.
// the apex the detector's rise test (DET_MIN_RISE_SAMPLES) can *just* accept.
//
// A raised cosine of width w and apex a crosses a trigger t at
// w*acos(2t/a - 1)/pi samples above it, so the edge it shows - crossing to apex - is
//
//     rise(a) = (w/2) * (1 - acos(1 - 2t/a) / pi)      samples,
//
// which grows with a from 0 (the apex is the crossing) up to w/2 (the crossing sits
// at the foot of the flank).  Asking for rise >= rise_samples therefore has a
// solution only when w > 2*rise_samples, and then it is
//
//     a >= t / cos^2(rise_samples * pi / w).
//
// Returns 0 when no apex can satisfy it: the pulse has no edge that lasts, however
// tall it is drawn (the 3-5 ms needles in config.h's table).  This is the *ideal*
// envelope - the detector measures its edge on a filtered, slope-bounded path, so
// the bench is the judge of the real one.
inline int32_t frontierApex(int32_t trigger, uint32_t rise_samples, uint32_t width) {
    if (trigger <= 0 || rise_samples == 0u) { return 0; }
    if (width <= 2u * rise_samples) { return 0; }   // no edge can last that long
    const float c = cosf((float)rise_samples * kPi / (float)width);
    if (c <= 0.0f) { return 0; }
    return (int32_t)ceilf((float)trigger / (c * c));
}

// The slow baseline wander (see "baseline wander" above): a pure function of the
// *global* sample index `total`, one full sine over `period` samples with a peak of
// +/- `amplitude` DAC codes.  It is deliberately one shared function rather than two
// copies of the same maths, because the host harness rebuilds the noise-free trace
// this model renders and has to land on the same integer code: the drift has to sit
// inside the reconstruction, not on top of it.
//
// The modulo is taken on the *integer* index, never on a running phase, so the
// argument stays exact however long the run is - a float phase would visibly lose
// resolution after tens of minutes.  With a 300 000-sample period the code moves at
// most (2*pi/period) * amplitude ~= 0.003 codes per sample, i.e. three orders of
// magnitude below DET_MAX_SLOPE: it is a level the baseline tracker follows, not a
// signal any bound is meant to catch.
inline float baselineWaveCode(int32_t amplitude, uint32_t period, uint32_t total) {
    if (amplitude == 0 || period == 0u) { return 0.0f; }
    const float u = (float)(total % period) / (float)period;   // 0 .. 1
    return (float)amplitude * sinf(2.0f * kPi * u);
}

class SignalModel {
public:
    struct Config {
        // ---- pattern cycle -------------------------------------------------
        uint32_t cycle_samples = 20000u;  // 2.0 s at 10 kSPS
        uint32_t max_pulses    = 48u;     // never more pulses than this per cycle
                                          // (a densely packed cycle holds ~20)
        uint32_t cycle_margin  = 600u;    // keep pulses clear of the cycle boundary,
                                          // so no event straddles it
        // ---- time between events (uniform) ---------------------------------
        uint32_t gap_min = 30u;           // samples of quiet after a pulse - 0.3 ms,
                                          // barely more than the end debounce, so
                                          // the next excursion is already rising
                                          // while the detector closes the previous
                                          // window
        uint32_t gap_max = 900u;          // (was 3200) the cycle is packed with events
                                          // from end to end instead of holding one
                                          // or two quiet seconds
        // ---- deliberately merged pairs (a two-humped event) ----------------
        uint32_t merge_pct      = 80u;    // percent of pulses that try to fuse with
                                          // the previous one (pairs only, no chains)
        float    merged_span_min = 1.50f; // an event of two pulses spans this..that
        float    merged_span_max = 2.00f; // times width_mean (uniform in between),
                                          // so merged widths form their own tight mode
        float    merge_min_overlap = 0.12f;  // ... and the two pulses must overlap by at
                                          // least this fraction of the first one's
                                          // width, or they stay two separate events
        // ---- peaks (raised cosine) -----------------------------------------
        float    width_mean = 1200.0f;    // samples (120 ms at 10 kSPS)
        float    width_sd   = 260.0f;
        uint32_t width_min  = 30u;        // 3.0 ms: the narrowest event that still
                                          // has to be measured - its leading edge
                                          // is only ~15 samples long
        uint32_t width_max  = 3000u;
        uint32_t width_span_pct = 40u;    // percent of the peaks drawn log-uniform
                                          // over [width_min, width_mean] instead -
                                          // the needles, in every cycle (see the
                                          // notes at the top of this file)
        float    height_mean = 900.0f;    // AD7606 counts above the baseline (nominal)
        float    height_sd   = 300.0f;
        uint32_t height_span_pct = 30u;   // percent of peaks drawn log-uniform over the
                                          // *whole* [height_min, height_max] span
                                          // instead - that is what guarantees the
                                          // extremes appear in every cycle
        int32_t  height_min  = 160;       // counts: about 1.3 * k_on * sigma, i.e. the
                                          // smallest apex the detector can still see
        int32_t  height_max  = 20000;     // counts: beyond what the DAC can output, so
                                          // the top of the span saturates the source
        // ---- the small-peak ("frontier") slice ------------------------------
        // What the height draw above cannot guarantee: a *small* apex on a pulse
        // wide enough for the rise test to be the thing that decides.  See the
        // header notes and config.h's SIGNALGEN_FRONTIER_* block.
        uint32_t frontier_pct = 50u;         // percent of peaks that take the frontier
                                             // style when their width lands in the
                                             // band (about a quarter of the draws do)
        uint32_t frontier_width_min = 200u;  // the band the slice applies in (20 ms
        uint32_t frontier_width_max = 800u;  // .. 80 ms): outside it DET_MAX_SLOPE
                                             // owns the narrow corner, and at the
                                             // wide end the run's own reference sets
                                             // the floor (it is k_on * sigma, plus
                                             // DET_MIN_MAX_VALUE on the reported max)
                                             // - config.h has it, measured
        int32_t  frontier_trigger = 100;     // counts: the on-threshold the apex is
                                             // judged against (k_on * sigma: +52..59
                                             // measured on the filtered stream)
        uint32_t frontier_rise    = 20u;     // samples of leading edge it must show
                                             // (DET_MIN_RISE_SAMPLES)
        float    frontier_margin_min = 2.5f; // apex = frontierApex(width) x this..that
        float    frontier_margin_max = 5.0f; // (uniform)
        // ---- the floor ladder (SIGNALGEN_LADDER_*) --------------------------
        // The one thing the random train above cannot present: a *small* apex with the
        // trace at the baseline on both sides.  Its gaps are 3..90 ms against 120 ms
        // pulses and 80 % of its pulses fuse with a neighbour, so a 300-count apex is
        // normally a shoulder on a 900-count slope, and on a plot whose scale is set by
        // the train's own range (the DAC clamp) it is a couple of pixels - which is
        // exactly what makes "how close to the baseline can an apex be and still be
        // detected" unanswerable on the plot.
        //
        // This mode answers it: the pattern becomes COUNT raised-cosine pulses of one
        // width, each centred in its own slot of the cycle (so every rung is isolated
        // by a whole slot of flat baseline), apexes stepping by `ladder_step` from
        // `ladder_min` up.  Nothing else is drawn - no random pulses, no glitches and
        // no impulses (see tick()) - so dither and the slow wander are all the detector
        // has to separate a rung from, and the plot's own scale settles near the ladder
        // instead of on the tallest false peak.  config.h's SIGNALGEN_LADDER_* block
        // has the numbers, the reasoning and how to run it.
        uint32_t ladder       = 0u;      // 0 = the random train above, 1 = the ladder
        uint32_t ladder_width = 600u;    // samples (60 ms): the width every rung has
        int32_t  ladder_min   = 20;      // counts: the bottom rung's apex - well inside
                                         // the +/-42-count dither band, so the staircase
                                         // starts where no apex can be told from noise
        int32_t  ladder_step  = 40;      // counts between neighbouring rungs
        uint32_t ladder_count = 12u;     // rungs: min .. min + step * (count - 1):
                                         // 20, 60 ... 460 counts at the values above
        // ---- rectangular glitches (deliberate false peaks) -----------------
        uint32_t glitch_pct = 40u;        // percent of pulses that are glitches
        uint32_t glitch_width_max = 6u;   // glitch width is uniform in 1..this
        float    glitch_height_mean = 1800.0f;
        float    glitch_height_sd   = 1100.0f;
        int32_t  glitch_height_min  = 120;
        int32_t  glitch_height_max  = 20000;
        // ---- waveform / scaling -------------------------------------------
        int32_t  baseline_code = 2048;    // DC level in DAC codes (~1.65 V)
        int32_t  noise_code    = 16;      // dither amplitude in DAC codes (~13 mV)
        float    noise_alpha   = 1.0f;    // dither low-pass weight (1.0 = white)
        // ---- slow baseline wander ------------------------------------------
        int32_t  baseline_wave_code    = 150;     // the DC level itself drifts,
                                                  // +/- this many DAC codes
                                                  // (~+/-396 AD counts at +/-10 V,
                                                  // i.e. many sigma of the dither)
        uint32_t baseline_wave_samples = 300000u; // one full sine period is this many
                                                  // samples - 30 s at 10 kSPS, so the
                                                  // drift is ~300x slower than the
                                                  // detector's own baseline tracker
                                                  // and must NOT be reported as an
                                                  // event
        float    spike_pct     = 0.05f;   // percent of samples carrying an impulse
        int32_t  spike_max     = 400;     // impulse height, uniform in +/- this (DAC
                                          // codes: ~+/-1056 AD counts at +/-10 V, i.e.
                                          // as tall as a nominal peak)
        int32_t  min_code      = 250;     // DAC output buffer linear window
        int32_t  max_code      = 3850;    // ... and the clip point of a saturated peak
        float    adc_per_dac   = 2.64f;   // AD7606 counts per DAC count (see config.h)
        uint32_t seed          = 0x2F6E2B1u;
    };

    SignalModel() {}
    explicit SignalModel(const Config& c) : cfg_(c) {}

    void          configure(const Config& c) { cfg_ = c; }
    const Config& config()  const { return cfg_; }

    // Seed the PRNG, draw cycle 0's train and compute the idle output value.
    void begin();

    // Advance one sample and return the DAC code for it.  Call once per ADC
    // conversion, immediately *before* the conversion is started.
    int32_t tick();

    int32_t  code()     const { return code_; }    // what the DAC holds now
    uint32_t position() const { return pos_; }     // sample index within the cycle
    uint32_t cycles()   const { return cycles_; }  // completed pattern cycles
    uint32_t samples()  const { return total_; }   // samples generated since begin():
                                                   // the index the baseline wander
                                                   // walks on
    uint32_t impulses() const { return impulses_; }// impulse-noise samples injected

    // The train currently in use (it is redrawn at every cycle start).
    const PulseSpec* pulses()     const { return pulses_; }
    uint32_t         pulseCount() const { return n_pulses_; }

private:
    static const uint32_t kLutSize   = 256u;  // raised-cosine LUT entries
    static const uint32_t kMaxPulses = 64u;   // array bound (Config::max_pulses <= this)

    uint32_t nextU32();                        // xorshift32
    float    uniform01();                      // [0, 1)
    float    white();                          // [-1, 1)
    void     buildTrain();                     // draw one cycle's pulse train
    void     buildLadder();                    // ... or the floor ladder (Config::ladder)
    int32_t  sampleCode(uint32_t index) const;
    static const float* shapeLut();

    Config    cfg_;
    uint32_t  rng_         = 0x2F6E2B1u;
    uint32_t  pos_         = 0;
    uint32_t  cycles_      = 0;
    uint32_t  total_       = 0;      // samples since begin(): the wander's clock, so
                                     // the drift does not restart with the pattern
    uint32_t  train_cycle_ = 0;      // cycle the current train belongs to
    uint32_t  n_pulses_    = 0;
    PulseSpec pulses_[kMaxPulses];
    float     dither_      = 0.0f;
    float     spike_       = 0.0f;   // impulse added to the sample being built
    uint32_t  impulses_    = 0;      // impulse count since begin()
    int32_t   code_        = 0;
};

// ===========================================================================
// Implementation (header-only: the M4, the host test and the Python mirror all
// share this one copy of the waveform maths)
// ===========================================================================

inline uint32_t SignalModel::nextU32() {
    // xorshift32 - the generator the bench pattern has always used.
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return rng_;
}

inline float SignalModel::uniform01() {
    // Top 24 bits only: exactly representable in a float, so the value is
    // bit-identical to the Python mirror (same integer / 2^24).
    return (float)(nextU32() >> 8) * (1.0f / 16777216.0f);   // [0, 1)
}

inline float SignalModel::white() {
    return (float)(int32_t)nextU32() * (1.0f / 2147483648.0f);   // [-1, 1)
}

inline const float* SignalModel::shapeLut() {
    static float lut[kLutSize];
    static bool  ready = false;
    if (!ready) {
        for (uint32_t i = 0; i < kLutSize; i++) {
            const float u = (float)i / (float)kLutSize;      // 0 .. 1
            lut[i] = 0.5f * (1.0f - cosf(2.0f * kPi * u));
        }
        ready = true;
    }
    return lut;
}

inline void SignalModel::begin() {
    rng_         = cfg_.seed;
    pos_         = 0;
    cycles_      = 0;
    total_       = 0;
    train_cycle_ = 0;
    dither_      = 0.0f;
    spike_       = 0.0f;
    impulses_    = 0;

    if (cfg_.max_pulses > kMaxPulses) { cfg_.max_pulses = kMaxPulses; }
    buildTrain();
    code_ = sampleCode(pos_);        // baseline (undithered) on both outputs at once
}

inline int32_t SignalModel::tick() {
    // A new cycle draws a new train, so the detector keeps meeting a fresh mix of
    // heights / widths / gaps instead of one fixed scenario.  Drawn once, at the
    // first sample of the cycle.
    if (train_cycle_ != cycles_) {
        buildTrain();
        train_cycle_ = cycles_;
    }

    // Dither so the detector sees a realistic, non-zero noise floor.
    // noise_alpha = 1.0 gives white noise; lower values low-pass it.
    dither_ += cfg_.noise_alpha * (white() - dither_);

    // Impulse noise: a single-sample spike, uniform in +/- spike_max, hundreds of
    // AD7606 counts tall - the "noise spikes" that look like peaks.  It is added on
    // top of whatever the pulses produce (and inside the DAC clamp), so it also
    // lands on real peaks and shows up in their reported max.
    //
    // The floor ladder draws none of them, and does not even spend the draws: a
    // +/-1056-count spike is taller than every rung of the ladder, so on the live plot
    // it would set the vertical scale on its own and hide the apexes the ladder exists
    // to show.  The false-peak bounds are the random train's business
    // (SIGNALGEN_SPIKE_PCT / SIGNALGEN_GLITCH_PCT); the ladder's business is the floor.
    spike_ = 0.0f;
    if (cfg_.ladder == 0u && (uniform01() * 100.0f) < cfg_.spike_pct) {
        spike_ = white() * (float)cfg_.spike_max;
        impulses_++;
    }

    code_ = sampleCode(pos_);

    total_++;                       // the wander's clock: never reset by the cycle,
                                    // so the drift runs on across the boundary
    if (++pos_ >= cfg_.cycle_samples) {
        pos_ = 0;
        cycles_++;
    }
    return code_;
}

inline int32_t SignalModel::sampleCode(uint32_t index) const {
    float v = (float)cfg_.baseline_code + dither_ * (float)cfg_.noise_code + spike_ +
              baselineWaveCode(cfg_.baseline_wave_code, cfg_.baseline_wave_samples,
                               total_);
    const float* shape = shapeLut();

    for (uint32_t i = 0; i < n_pulses_; i++) {
        const PulseSpec& p = pulses_[i];
        if (index < p.start || index >= (p.start + p.width)) {
            continue;
        }
        const float a = (float)p.amplitude / cfg_.adc_per_dac;   // counts -> DAC codes
        if (p.rect != 0u) {
            v += a;
        } else {
            uint32_t u = ((index - p.start) * kLutSize) / p.width;
            if (u >= kLutSize) { u = kLutSize - 1u; }
            v += a * shape[u];
        }
    }

    // Stay inside the DAC output buffer's linear window.
    if (v < (float)cfg_.min_code) { v = (float)cfg_.min_code; }
    if (v > (float)cfg_.max_code) { v = (float)cfg_.max_code; }
    return (int32_t)lroundf(v);
}

// The floor ladder (Config::ladder): COUNT rungs of one width, each centred in its own
// slot of the cycle, apexes stepping from `ladder_min` by `ladder_step`.  Deliberately
// deterministic - no PRNG draw at all - so the same staircase comes back every cycle
// and a rung can be compared with the same rung of the cycle before it.  What makes
// each one a *peak near the baseline* instead of a shoulder is the slot: a raised
// cosine is exactly zero outside its own width, so there are `slot - width` samples of
// nothing but dither and wander between two rungs - enough for the detector's
// end-debounce to close a window well before the next rung rises.
inline void SignalModel::buildLadder() {
    const uint32_t count = (cfg_.ladder_count == 0u) ? 1u : cfg_.ladder_count;
    const uint32_t slot  = cfg_.cycle_samples / count;
    if (cfg_.ladder_width == 0u || slot <= cfg_.ladder_width) {
        return;   // the rungs would touch (or not fit): nothing to draw
    }
    for (uint32_t k = 0; k < count && n_pulses_ < cfg_.max_pulses; k++) {
        const uint32_t start = slot * k + (slot - cfg_.ladder_width) / 2u;
        if (start + cfg_.ladder_width > cfg_.cycle_samples) {
            break;   // keep the whole rung inside the cycle; the slot arithmetic above
                     // already holds the last one clear of the boundary, so a rung
                     // never straddles a cycle the way a random pulse could
        }
        pulses_[n_pulses_].start     = start;
        pulses_[n_pulses_].width     = cfg_.ladder_width;
        pulses_[n_pulses_].amplitude = cfg_.ladder_min + (int32_t)k * cfg_.ladder_step;
        pulses_[n_pulses_].rect      = 0u;   // a raised cosine: a peak, never a glitch
        n_pulses_++;
    }
}

inline void SignalModel::buildTrain() {
    n_pulses_ = 0;

    // The floor ladder is not a train: it is the measurement the train above cannot
    // make (see Config::ladder).  One branch, and nothing of the random draw below
    // runs - so the ladder cannot move any of the train's statistics.
    if (cfg_.ladder != 0u) {
        buildLadder();
        return;
    }

    uint32_t cursor      = 0;        // end of the last pulse, in samples
    uint32_t prev_start  = 0;        // start of the last pulse
    uint32_t prev_w      = 0;        // width of the last pulse
    bool     prev_merged = false;    // was the last pulse merged into its own?

    while (n_pulses_ < cfg_.max_pulses) {
        // Fixed draw order - the Python mirror replays exactly this sequence.
        const float u_dist = uniform01();   // gap, or the merged pair's span
        const float u_kind = uniform01();   // glitch or peak
        const float u_ov   = uniform01();   // fuse with the previous pulse?
        const float u1     = uniform01();
        const float u2     = uniform01();
        const float u_w    = uniform01();   // glitch width
        const float u_hgt  = uniform01();   // log-uniform height, or the frontier margin
        const float u_span = uniform01();   // height style: frontier / log-uniform /
                                            // Gaussian (see below)
        const float u_wgt  = uniform01();   // is this peak's width log-uniform?
        const float u_wid  = uniform01();   // ... the log-uniform width itself

        const bool glitch = (u_kind * 100.0f) < (float)cfg_.glitch_pct;

        // Box-Muller: one uniform pair -> two independent standard normals.  Both
        // are used (one for the width, one for the nominal height), so no normal has
        // to be cached between pulses and the mirror stays exact.
        const float r  = sqrtf(-2.0f * logf(u1 > 0.0f ? u1 : 1e-7f));
        const float an = 2.0f * kPi * u2;
        const float z0 = r * cosf(an);
        const float z1 = r * sinf(an);

        uint32_t width;
        int32_t  amplitude;
        if (glitch) {
            width     = 1u + (uint32_t)(u_w * (float)cfg_.glitch_width_max);  // 1..max
            amplitude = clampI32((int32_t)lroundf(cfg_.glitch_height_mean +
                                                  cfg_.glitch_height_sd * z0),
                                 cfg_.glitch_height_min, cfg_.glitch_height_max);
        } else {
            const int32_t w = (int32_t)lroundf(cfg_.width_mean + cfg_.width_sd * z0);
            width     = (uint32_t)clampI32(w, (int32_t)cfg_.width_min,
                                              (int32_t)cfg_.width_max);
            // The u_span draw picks this pulse's *height style*: the small-peak
            // frontier slice, the log-uniform slice, or the nominal Gaussian.  The
            // slice only applies inside the width band where the documented envelope
            // is the whole story - outside it the slope guard owns the narrow corner
            // and a faint wide pulse can close its own window early (config.h's
            // SIGNALGEN_FRONTIER_* block).  A pulse outside the band keeps the
            // ordinary draws, which is what the needles and the wide faint ones are
            // for.  The width itself is drawn as always, whatever the style.
            const float style    = u_span * 100.0f;
            const bool  frontier = (style < (float)cfg_.frontier_pct) &&
                                   (width >= cfg_.frontier_width_min) &&
                                   (width <= cfg_.frontier_width_max);
            if (frontier) {
                // A small peak drawn from the envelope the detector's own bounds allow
                // at this width - the corner the independent height/width draws above
                // never reach (see frontierApex and config.h's measured sweep).
                const float floor_apex = (float)frontierApex(cfg_.frontier_trigger,
                                                             cfg_.frontier_rise, width);
                const float margin = cfg_.frontier_margin_min +
                                     u_hgt * (cfg_.frontier_margin_max -
                                              cfg_.frontier_margin_min);
                amplitude = clampI32((int32_t)lroundf(floor_apex * margin),
                                     cfg_.height_min, cfg_.height_max);
            } else {
                if ((u_wgt * 100.0f) < (float)cfg_.width_span_pct) {
                    // Log-uniform over [width_min, width_mean]: every octave equally
                    // likely, so the narrow end of the range is present in *every*
                    // cycle - a 3 ms needle next to a nominal-width peak.  It stops at
                    // width_mean on purpose (see the notes at the top of this file): a
                    // single pulse must never be as wide as a merged pair, or the
                    // event-width distribution stops having two modes.
                    const float lo = (float)cfg_.width_min;
                    const float hi = (float)cfg_.width_mean;
                    width =
                        (uint32_t)clampI32((int32_t)lroundf(lo * powf(hi / lo, u_wid)),
                                           (int32_t)cfg_.width_min,
                                           (int32_t)cfg_.width_max);
                }
                if (style >= (float)cfg_.frontier_pct &&
                    style < (float)(cfg_.frontier_pct + cfg_.height_span_pct)) {
                    // Log-uniform over the whole span: every octave equally likely, so
                    // a cycle always contains both peaks that barely clear the noise
                    // band and peaks that clip the DAC (saturated, flat-topped events).
                    const float lo = (float)cfg_.height_min;
                    const float hi = (float)cfg_.height_max;
                    amplitude = clampI32((int32_t)lroundf(lo * powf(hi / lo, u_hgt)),
                                         cfg_.height_min, cfg_.height_max);
                } else {
                    amplitude = clampI32((int32_t)lroundf(cfg_.height_mean +
                                                          cfg_.height_sd * z1),
                                         cfg_.height_min, cfg_.height_max);
                }
            }
        }

        // Where does this pulse start?  Normally after the previous pulse plus a
        // random gap, but merge_pct % of them are placed *inside* the previous one,
        // far enough in that the two shapes add up into a single two-humped event
        // spanning merged_span_min..max nominal widths - clearly wider than any
        // single pulse, which is what makes the width distribution double peaked.
        // Merges come in pairs (a pulse that is already part of a pair never starts
        // another one) and are only used when the two pulses really do overlap
        // enough to fuse; otherwise the pulse is placed separated, like any other.
        uint32_t start  = 0;
        bool     merged = false;
        if (n_pulses_ > 0u && !prev_merged &&
            ((u_ov * 100.0f) < (float)cfg_.merge_pct)) {
            const float    span_f = cfg_.merged_span_min +
                                    u_dist * (cfg_.merged_span_max - cfg_.merged_span_min);
            const uint32_t span   = (uint32_t)lroundf(span_f * cfg_.width_mean);
            const uint32_t need   = (uint32_t)(cfg_.merge_min_overlap * (float)prev_w);
            // span > width keeps the order (the pair never starts before the first
            // pulse); span + need <= prev_w + width is the overlap requirement.
            if (span > width && (span + need) <= (prev_w + width)) {
                start  = prev_start + span - width;
                merged = true;
            }
        }
        if (!merged) {
            const uint32_t gap = cfg_.gap_min +
                                 (uint32_t)(u_dist * (float)(cfg_.gap_max - cfg_.gap_min + 1u));
            start = cursor + gap;
        }

        // Keep the whole pulse - plus a margin - inside the cycle, so no event
        // straddles a cycle boundary.
        if (start + width + cfg_.cycle_margin > cfg_.cycle_samples) {
            break;
        }

        pulses_[n_pulses_].start     = start;
        pulses_[n_pulses_].width     = width;
        pulses_[n_pulses_].amplitude = amplitude;
        pulses_[n_pulses_].rect      = glitch ? 1u : 0u;
        n_pulses_++;

        cursor      = (start + width > cursor) ? (start + width) : cursor;
        prev_start  = start;
        prev_w      = width;
        prev_merged = merged;
    }
}

} // namespace sorter
