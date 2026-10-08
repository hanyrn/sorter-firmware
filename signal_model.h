// signal_model.h
//
// Arduino-free, deterministic model of the bench-test synthetic waveform: a train
// of randomly parameterised pulses on a DC baseline with dither.
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
// shapes instead of the same six pulses for ever:
//
//   gap between pulses      uniform in [gap_min, gap_max] samples
//   peak width (cosine)     Gaussian(width_mean, width_sd)   clipped to bounds
//   peak height             Gaussian(height_mean, height_sd) clipped to bounds
//   glitch width (rect)     uniform in [1, glitch_width_max] samples
//   glitch height           Gaussian(glitch_height_mean, glitch_height_sd)
//   glitch probability      glitch_pct % of the pulses
//
// Gaussian because that is what a real signal looks like: most peaks are close to
// the nominal size, a few are much larger or smaller.  The rectangular glitches are
// kept so the false-peak bounds (DET_MIN_MAX_VALUE, DET_MIN_WIDTH_SAMPLES,
// DET_MAX_SLOPE) keep being exercised - a random train must still show the detector
// rejecting what it should reject.
//
// The *draw order is fixed and documented* so the Python mirror can reproduce the
// train exactly: buildTrain() draws, per pulse, gap, kind, u1, u2, glitch-width (five
// uniforms, always, whatever the branch), and Box-Muller turns the (u1, u2) pair
// into two independent standard normals, so both members of the pair are used and
// no normal has to be cached between pulses.
//
// Float note: the firmware computes in float32, the Python mirror in float64, so a
// drawn height/width can differ by one count in the last digit.  That does not
// matter for a plot, and every *decision* (how many pulses, how many glitches,
// where they land) uses the same integers on both sides.

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

class SignalModel {
public:
    struct Config {
        // ---- pattern cycle -------------------------------------------------
        uint32_t cycle_samples = 20000u;  // 2.0 s at 10 kSPS
        uint32_t max_pulses    = 16u;     // never more pulses than this per cycle
        uint32_t cycle_margin  = 600u;    // keep pulses clear of the cycle boundary,
                                          // so no event straddles it
        // ---- time between events (uniform) ---------------------------------
        uint32_t gap_min = 700u;          // samples of silence after a pulse
        uint32_t gap_max = 3200u;
        // ---- broad peaks (raised cosine), Gaussian -------------------------
        float    width_mean = 1200.0f;    // samples
        float    width_sd   = 260.0f;
        uint32_t width_min  = 300u;
        uint32_t width_max  = 3000u;
        float    height_mean = 900.0f;    // AD7606 counts above the baseline
        float    height_sd   = 300.0f;
        int32_t  height_min  = 150;
        int32_t  height_max  = 3500;
        // ---- rectangular glitches (deliberate false peaks) -----------------
        uint32_t glitch_pct = 25u;        // percent of pulses that are glitches
        uint32_t glitch_width_max = 6u;   // glitch width is uniform in 1..this
        float    glitch_height_mean = 1200.0f;
        float    glitch_height_sd   = 600.0f;
        int32_t  glitch_height_min  = 200;
        int32_t  glitch_height_max  = 3500;
        // ---- waveform / scaling -------------------------------------------
        int32_t  baseline_code = 2048;    // DC level in DAC codes (~1.65 V)
        int32_t  noise_code    = 6;       // dither amplitude in DAC codes (~4.8 mV)
        float    noise_alpha   = 1.0f;    // dither low-pass weight (1.0 = white)
        int32_t  min_code      = 250;     // DAC output buffer linear window
        int32_t  max_code      = 3850;
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

    // The train currently in use (it is redrawn at every cycle start).
    const PulseSpec* pulses()     const { return pulses_; }
    uint32_t         pulseCount() const { return n_pulses_; }

private:
    static const uint32_t kLutSize   = 256u;  // raised-cosine LUT entries
    static const uint32_t kMaxPulses = 32u;   // array bound (Config::max_pulses <= this)

    uint32_t nextU32();                        // xorshift32
    float    uniform01();                      // [0, 1)
    float    white();                          // [-1, 1)
    void     buildTrain();                     // draw one cycle's pulse train
    int32_t  sampleCode(uint32_t index) const;
    static const float* shapeLut();

    Config    cfg_;
    uint32_t  rng_         = 0x2F6E2B1u;
    uint32_t  pos_         = 0;
    uint32_t  cycles_      = 0;
    uint32_t  train_cycle_ = 0;      // cycle the current train belongs to
    uint32_t  n_pulses_    = 0;
    PulseSpec pulses_[kMaxPulses];
    float     dither_      = 0.0f;
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
    train_cycle_ = 0;
    dither_      = 0.0f;

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

    code_ = sampleCode(pos_);

    if (++pos_ >= cfg_.cycle_samples) {
        pos_ = 0;
        cycles_++;
    }
    return code_;
}

inline int32_t SignalModel::sampleCode(uint32_t index) const {
    float v = (float)cfg_.baseline_code + dither_ * (float)cfg_.noise_code;
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

inline void SignalModel::buildTrain() {
    n_pulses_ = 0;
    uint32_t cursor = 0;             // end of the last pulse, in samples

    while (n_pulses_ < cfg_.max_pulses) {
        // Fixed draw order - the Python mirror replays exactly this sequence.
        const float u_gap  = uniform01();
        const float u_kind = uniform01();
        const float u1     = uniform01();
        const float u2     = uniform01();
        const float u_w    = uniform01();

        const uint32_t gap    = cfg_.gap_min +
                                (uint32_t)(u_gap * (float)(cfg_.gap_max - cfg_.gap_min + 1u));
        const uint32_t start  = cursor + gap;
        const bool     glitch = (u_kind * 100.0f) < (float)cfg_.glitch_pct;

        // Box-Muller: one uniform pair -> two independent standard normals.  Both
        // are used (one for the width, one for the height), so no normal has to be
        // cached between pulses and the mirror stays exact.
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
            amplitude = clampI32((int32_t)lroundf(cfg_.height_mean +
                                                  cfg_.height_sd * z1),
                                 cfg_.height_min, cfg_.height_max);
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

        cursor = start + width;
    }
}

} // namespace sorter
