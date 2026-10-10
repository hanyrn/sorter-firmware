// test/test_event_detector.cpp
//
// Host-side unit test for EventDetector (no Arduino required).
// Build & run it with test/run_tests.ps1 (needs g++ or clang++).
//
// Scenarios:
//   1. baseline 2000 +/- 5, three broad peaks (amplitude 800, sigma 200 samples),
//      a 1-sample glitch and a 4-sample glitch.  The three broad peaks are reported
//      as VALID, and neither glitch becomes an event at all: the 1-sample one the
//      impulse filter takes out, and the 4-sample one fails the RISE TEST (2 ms) and
//      is suppressed - not even a rejected record is produced for it.
//   2. the same peak with single-sample impulses (taller than the peak) landing on
//      and around it: the reported maximum must stay the peak's own height.  This
//      is the regression test for "a noise spike became the event's maximum".
//   3. a step-like excursion.  A 1 ms step (10 samples) is suppressed by the rise
//      test; a fast-attack excursion that lasts long enough to be an event (a 1 ms
//      leading edge onto a slow decay) is reported and rejected as
//      SHORT_FOR_HEIGHT, because its apex is reached 1 ms after the crossing.
//   4. the rise test's dip tolerance: a real rise may dip below its off-threshold
//      for up to dip_tolerance samples and still be an event, while a longer
//      dropout kills the candidate.
//   5. the quiet period: an event inside it is not reported, and the reference it
//      establishes is the noise rather than the peak that lands in it.
//   6. ImpulseFilter on its own: a 1-sample spike (and a 2-sample one with 5 taps)
//      never reaches the output, while a ramp passes through (delayed).


#include "../event_detector.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

namespace {

uint32_t g_rng = 0x12345678u;
uint32_t rndU32() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng; }
double   uniform() { return (double)(rndU32() & 0x00FFFFFFu) / (double)0x01000000u; }
double   gauss() {   // Box-Muller
    const double u1 = uniform() + 1e-12;
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
}

// The shipped configuration (see config.h): the behaviour below is judged against
// the bounds the firmware actually runs with, at the rate it runs at (10 kSPS =>
// one sample is 100 us, so 20 samples = 2 ms and 50 samples = 5 ms).
EventDetector::Config testConfig() {
    EventDetector::Config c;
    c.baseline_alpha    = 0.001f;
    c.noise_alpha       = 0.001f;
    c.k_on              = 5.0f;
    c.k_off             = 2.0f;
    c.k_gate            = 3.0f;
    c.sigma_floor       = 1;
    c.quiet_samples     = 3000;    // DET_QUIET_SAMPLES     (0.3 s)
    c.quiet_seed_points = 128;     // DET_QUIET_SEED_POINTS
    c.gate_max           = 160;    // DET_GATE_MAX
    c.end_debounce      = 2;       // DET_END_DEBOUNCE      (0.2 ms)
    c.min_rise_samples  = 20;      // DET_MIN_RISE_SAMPLES  (2 ms)
    c.dip_tolerance     = 3;       // DET_DIP_TOLERANCE     (0.3 ms)
    c.impulse_filter_taps = 7;     // DET_IMPULSE_FILTER_TAPS
    c.min_max_value     = 80;      // DET_MIN_MAX_VALUE
    c.min_width_samples = 35;      // DET_MIN_WIDTH_SAMPLES (3.5 ms)
    c.max_width_samples = 20000;   // DET_MAX_WIDTH_SAMPLES (2 s, fault guard)
    c.min_area          = 0;       // DET_MIN_AREA
    c.max_slope         = 100.0f;  // DET_MAX_SLOPE
    return c;
}

// A waveform: broad cosine-ish peaks, rectangular glitches and single-sample
// impulses, all on a noisy baseline (the bench model in miniature).
struct Pulse  { int c, w; double amp; };
struct Glitch { int at, len; double amp; };

struct Wave {
    const Pulse*  peaks;     int n_peaks;
    const Glitch* glitches;  int n_glitches;
    const int*    impulses;  int n_impulses;
    double        impulse_amp;
};

int32_t signalAt(const Wave& w, int i) {
    const double baseline = 2000.0, noise = 5.0;
    double x = baseline + gauss() * noise;
    for (int p = 0; p < w.n_peaks; p++) {
        const double d = (double)(i - w.peaks[p].c);
        x += w.peaks[p].amp * std::exp(-(d * d) / (2.0 * w.peaks[p].w * w.peaks[p].w));
    }
    for (int g = 0; g < w.n_glitches; g++) {
        if (i >= w.glitches[g].at && i < w.glitches[g].at + w.glitches[g].len) {
            x += w.glitches[g].amp;
        }
    }
    for (int k = 0; k < w.n_impulses; k++) {
        if (i == w.impulses[k]) { x += w.impulse_amp; }
    }
    return (int32_t)std::lround(x);
}

} // namespace

static int g_failures = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: ");                                              \
            std::printf(__VA_ARGS__);                                           \
            std::printf("\n");                                                  \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

// ---------------------------------------------------------------------------
// 1. peaks + a 1-sample and a 4-sample glitch
// ---------------------------------------------------------------------------
static void testPeaksAndGlitches() {
    const Pulse  peaks[3]    = { {6000, 200, 800.0}, {13000, 200, 800.0}, {20000, 200, 800.0} };
    const Glitch glitches[2] = { {9000, 1, 900.0}, {16000, 4, 900.0} };
    const Wave   wave        = { peaks, 3, glitches, 2, nullptr, 0, 0.0 };

    EventDetector det(testConfig());
    PeakMetrics   m;
    int closes = 0, valid = 0, invalid = 0, suppressed = 0;

    for (int i = 0; i < 26000; i++) {
        const DetectorResult r = det.update(signalAt(wave, i), (uint32_t)i * 100u,
                                           (uint32_t)i, m);
        if (r == DET_DISCARDED) { suppressed++; continue; }
        if (r != DET_CLOSED)    { continue; }
        closes++;
        std::printf("event #%u  %s  reason=%u  max=%d  width=%u smp  rise=%u smp  "
                    "area=%llu\n",
                    (unsigned)m.index, m.valid ? "VALID" : "rejected", (unsigned)m.reason,
                    (int)m.max_value, (unsigned)m.width_samples, (unsigned)m.rise_samples,
                    (unsigned long long)m.area);
        if (m.valid) {
            valid++;
            CHECK(m.max_value >= 650 && m.max_value <= 950,
                  "peak amplitude out of range: %d", (int)m.max_value);
            CHECK(m.width_samples >= 600 && m.width_samples <= 1700,
                  "peak width out of range: %u", (unsigned)m.width_samples);
            // The width the bounds were judged with is the event's *duration*: the
            // excursion above, widened for the pulse's own envelope - never narrowed,
            // and never by more than the 2x cap.  (These pulses are Gaussians of
            // half-width 200 samples, so the factor is small but not exactly 1.)
            CHECK(m.duration_samples >= m.width_samples &&
                  m.duration_samples <= 2u * m.width_samples,
                  "duration %u outside [%u, %u]", (unsigned)m.duration_samples,
                  (unsigned)m.width_samples, 2u * (unsigned)m.width_samples);
            CHECK(m.duration_samples >= 600 && m.duration_samples <= 1700,
                  "peak duration out of range: %u", (unsigned)m.duration_samples);
            CHECK(m.width_samples > 0 &&
                  m.duration_us == (uint32_t)((uint64_t)m.width_us * m.duration_samples /
                                              m.width_samples),
                  "duration %uus does not match %usmp of %uus", (unsigned)m.duration_us,
                  (unsigned)m.duration_samples, (unsigned)m.width_us);
            CHECK(m.rise_samples >= 300,
                  "a real peak reported a %u-sample leading edge",
                  (unsigned)m.rise_samples);
        } else {
            invalid++;
            CHECK(m.reason == REASON_SHORT_FOR_HEIGHT ||
                  m.reason == REASON_WIDTH_TOO_NARROW,
                  "unexpected rejection reason %u", (unsigned)m.reason);
        }
    }

    std::printf("\nclosed=%d valid=%d invalid=%d suppressed=%d  baseline=%.1f sigma=%.2f\n",
                closes, valid, invalid, suppressed, det.baseline(), det.sigma());
    // The three peaks and nothing else.  Neither glitch is an event: the 1-sample one
    // never reaches the detector (the impulse filter removes it) and the 4-sample one
    // (0.4 ms) fails the rise test and is suppressed - no record is produced for it.
    CHECK(valid == 3, "expected 3 valid peaks, got %d", valid);
    CHECK(invalid == 0, "expected no rejected records, got %d", invalid);
    CHECK(closes == 3, "expected 3 closed events, got %d", closes);
    CHECK(suppressed >= 1,
          "the 4-sample glitch should have been suppressed (suppressed=%d)", suppressed);
    CHECK(std::fabs(det.baseline() - 2000.0) <= 25.0,
          "baseline did not track to ~2000 (got %.1f)", det.baseline());
}

// ---------------------------------------------------------------------------
// 2. impulses on and around a peak must not become its maximum
// ---------------------------------------------------------------------------
static void testImpulseDoesNotBecomeMax() {
    const Pulse peaks[1] = { {6000, 200, 800.0} };
    const int   imp[3]   = { 5000, 6000, 6500 };   // before, on and after the apex
    const Wave  wave     = { peaks, 1, nullptr, 0, imp, 3, 2000.0 };

    EventDetector det(testConfig());
    PeakMetrics   m;
    int valid = 0, closes = 0, suppressed = 0, max_seen = 0;

    for (int i = 0; i < 12000; i++) {
        const DetectorResult r = det.update(signalAt(wave, i), (uint32_t)i * 100u,
                                           (uint32_t)i, m);
        if (r == DET_DISCARDED) { suppressed++; continue; }
        if (r != DET_CLOSED)    { continue; }
        closes++;
        std::printf("impulse: event #%u %s reason=%u max=%d width=%u rise=%u\n",
                    (unsigned)m.index, m.valid ? "VALID" : "rejected",
                    (unsigned)m.reason, (int)m.max_value, (unsigned)m.width_samples,
                    (unsigned)m.rise_samples);
        if (m.valid) { valid++; if (m.max_value > max_seen) { max_seen = m.max_value; } }
    }

    CHECK(closes == 1, "impulse: expected 1 event (the peak), got %d", closes);
    CHECK(valid == 1, "impulse: expected the peak to stay VALID");
    // The peak is ~800 counts and the impulses are 2000: without the input filter, and
    // without the slope-bounded measurement path, the reported maximum would be ~2800
    // (baseline + peak + impulse).
    CHECK(max_seen >= 700 && max_seen <= 950,
          "impulse: reported max %d - the impulse leaked into the maximum", max_seen);
    // An impulse the median filter could not take out is suppressed, never reported as
    // an event of its own.
    std::printf("impulse: suppressed candidates=%d\n", suppressed);
}

// ---------------------------------------------------------------------------
// 3. a step is not a peak - and what happens to it depends on how long it is
// ---------------------------------------------------------------------------
// A 1 ms step (10 samples): the rise test throws it out silently.
static void testShortStepSuppressed() {
    const Glitch glitches[1] = { {6000, 10, 500.0} };
    const Wave   wave        = { nullptr, 0, glitches, 1, nullptr, 0, 0.0 };

    EventDetector det(testConfig());
    PeakMetrics   m;
    int closes = 0, suppressed = 0;

    for (int i = 0; i < 12000; i++) {
        const DetectorResult r = det.update(signalAt(wave, i), (uint32_t)i * 100u,
                                           (uint32_t)i, m);
        if (r == DET_DISCARDED) { suppressed++; }
        if (r == DET_CLOSED)    { closes++; }
    }

    CHECK(closes == 0, "short step: expected no event at all, got %d", closes);
    CHECK(suppressed >= 1, "short step: the candidate should have been suppressed");
}

// A fast-attack excursion: 500 counts that arrive in 5 samples (1 ms) and then decay
// slowly, so the excursion lasts ~300 samples and *is* an event - but its apex is there
// 5 samples after the crossing, which is not a rise, so it must be rejected by the
// shape guard.  (The 1 ms step above never gets this far: the rise test suppresses it.)
static void testFastAttackRejected() {
    EventDetector det(testConfig());
    PeakMetrics   m;
    int closes = 0, valid = 0, reason = REASON_OK;

    for (int i = 0; i < 12000; i++) {
        double x = 2000.0 + gauss() * 5.0;
        if (i >= 6000) {
            const double t = (double)(i - 6000);
            x += (t < 5.0) ? (100.0 * t) : (500.0 * std::exp(-(t - 5.0) / 80.0));
        }
        const DetectorResult r = det.update((int32_t)std::lround(x), (uint32_t)i * 100u,
                                           (uint32_t)i, m);
        if (r != DET_CLOSED) { continue; }
        closes++;
        valid += m.valid ? 1 : 0;
        reason = m.reason;
        std::printf("fast attack: event #%u %s reason=%u max=%d width=%u rise=%u\n",
                    (unsigned)m.index, m.valid ? "VALID" : "rejected",
                    (unsigned)m.reason, (int)m.max_value, (unsigned)m.width_samples,
                    (unsigned)m.rise_samples);
    }

    CHECK(closes == 1, "fast attack: expected 1 event, got %d", closes);
    CHECK(valid == 0, "fast attack: a 1 ms leading edge was accepted as a peak");
    CHECK(reason == REASON_SHORT_FOR_HEIGHT,
          "fast attack: expected SHORT_FOR_HEIGHT, got reason %u", (unsigned)reason);
}

// ---------------------------------------------------------------------------
// 4. the rise test's dip tolerance
// ---------------------------------------------------------------------------
// A trapezoid on the noisy baseline: rise over `rise_n` samples, hold, fall.  The
// notch cuts a gap into the leading edge (the signal sits at the baseline for
// `notch_len` samples while the excursion is still young).
#define TRAP_T0    6000
#define TRAP_RISE  40
#define TRAP_HOLD  160
#define TRAP_FALL  40
#define TRAP_AMP   320.0
static int32_t trapezoid(int i, int notch_at, int notch_len) {
    double x = 2000.0 + gauss() * 5.0;
    if (notch_len > 0 && i >= notch_at && i < notch_at + notch_len) {
        return (int32_t)std::lround(x);                     // the notch: back to baseline
    }
    const int t1 = TRAP_T0 + TRAP_RISE;
    const int t2 = t1 + TRAP_HOLD;
    const int t3 = t2 + TRAP_FALL;
    if      (i >= TRAP_T0 && i < t1) { x += TRAP_AMP * (double)(i - TRAP_T0) / TRAP_RISE; }
    else if (i >= t1      && i < t2) { x += TRAP_AMP; }
    else if (i >= t2      && i < t3) { x += TRAP_AMP * (double)(t3 - i) / TRAP_FALL; }
    return (int32_t)std::lround(x);
}

static void testDipTolerance() {
    // The excursion crosses the on-threshold at ~6003, so a notch at 6010 is inside
    // the rise test, where the dip rule lives.  The impulse filter is off: what is
    // under test is the dip rule, not the median (a 3-sample notch would not survive
    // a 7-tap median at all, which would hide the rule behind the filter).
    EventDetector::Config cfg = testConfig();
    cfg.impulse_filter_taps = 1;

    const int  notches[2] = { 3, 6 };          // <= dip_tolerance, then more
    const bool want_sup[2] = { false, true };

    for (int k = 0; k < 2; k++) {
        EventDetector det(cfg);
        PeakMetrics   m;
        int closes = 0, valid = 0, suppressed = 0;

        for (int i = 0; i < 12000; i++) {
            const DetectorResult r = det.update(trapezoid(i, 6010, notches[k]),
                                                (uint32_t)i * 100u, (uint32_t)i, m);
            if (r == DET_DISCARDED) { suppressed++; }
            if (r == DET_CLOSED)    { closes++; valid += m.valid ? 1 : 0; }
        }

        std::printf("dip=%d: closed=%d valid=%d suppressed=%d\n",
                    notches[k], closes, valid, suppressed);
        CHECK(closes == 1, "dip=%d: expected 1 event, got %d", notches[k], closes);
        CHECK(valid == 1, "dip=%d: the trapezoid should be a VALID peak", notches[k]);
        CHECK((suppressed >= 1) == want_sup[k],
              "dip=%d: %s a suppressed candidate (got %d)", notches[k],
              want_sup[k] ? "expected" : "did not expect", suppressed);
    }
}

// ---------------------------------------------------------------------------
// 5. the quiet period establishes the reference, and reports nothing
// ---------------------------------------------------------------------------
static void testQuietPeriodAndReference() {
    // One trapezoid right inside the quiet period (it must not be reported and must
    // not become the reference), one afterwards (it must be reported normally).
    EventDetector det(testConfig());
    PeakMetrics   m;
    int closes = 0, valid = 0;

    for (int i = 0; i < 12000; i++) {
        const int32_t x = (i >= 1000 && i < 1240) ? (int32_t)std::lround(2000.0 + 320.0 + gauss() * 5.0)
                                                  : trapezoid(i, 0, 0);
        const DetectorResult r = det.update(x, (uint32_t)i * 100u, (uint32_t)i, m);
        if (r != DET_CLOSED) { continue; }
        closes++;
        valid += m.valid ? 1 : 0;
        std::printf("quiet: event #%u %s start=%u width=%u max=%d rise=%u\n",
                    (unsigned)m.index, m.valid ? "VALID" : "rejected",
                    (unsigned)m.start_us, (unsigned)m.width_samples,
                    (int)m.max_value, (unsigned)m.rise_samples);
    }

    // Only the trapezoid after the quiet period is an event: an event inside the quiet
    // period cannot be reported (detection is not armed yet).
    CHECK(closes == 1, "quiet: expected 1 event (the one after the quiet period), got %d",
          closes);
    CHECK(valid == 1, "quiet: the trapezoid after the quiet period should be VALID");
    // ... and the peak inside it did not become the reference.
    CHECK(std::fabs(det.baseline() - 2000.0) <= 25.0,
          "quiet: baseline is %.1f, expected ~2000 (a peak inside the quiet period "
          "leaked into the reference)", det.baseline());
}

// ---------------------------------------------------------------------------
// 6. the input filter itself
// ---------------------------------------------------------------------------
static void testImpulseFilter() {
    // 1-sample spike, 3 taps: the output never picks it up.
    {
        ImpulseFilter f;
        f.configure(3);
        const int32_t in[5] = { 0, 0, 1000, 0, 0 };
        int32_t worst = 0;
        for (int i = 0; i < 5; i++) { if (f.step(in[i]) > worst) { worst = f.value(); } }
        CHECK(worst == 0, "filter: a 1-sample spike survived 3 taps (peak %d)", (int)worst);
    }
    // 2-sample spike, 5 taps: ditto.
    {
        ImpulseFilter f;
        f.configure(5);
        const int32_t in[6] = { 0, 0, 1000, 1000, 0, 0 };
        int32_t worst = 0;
        for (int i = 0; i < 6; i++) { f.step(in[i]); if (f.value() > worst) { worst = f.value(); } }
        CHECK(worst == 0, "filter: a 2-sample spike survived 5 taps (peak %d)", (int)worst);
    }
    // A ramp comes through (one sample late), so a real peak is not distorted.
    {
        ImpulseFilter f;
        f.configure(3);
        int32_t out[5];
        for (int i = 0; i < 5; i++) { out[i] = f.step(i * 10); }
        CHECK(out[3] == 20 && out[4] == 30,
              "filter: ramp came through as %d/%d, expected 20/30", (int)out[3], (int)out[4]);
    }
    // taps = 1 is a passthrough.
    {
        ImpulseFilter f;
        f.configure(1);
        CHECK(f.step(1234) == 1234, "filter: taps=1 did not pass the sample through");
    }
}

// ---------------------------------------------------------------------------
// 7. the duration the width bound is judged with (envelopeDuration)
// ---------------------------------------------------------------------------
// A raised cosine of duration W whose apex A is read against a trigger T spends
// (W / pi) * acos(2T/A - 1) above T, so the span an event shows is not its length: a
// faint pulse is cut off high up its flanks.  The detector inverts that relation, which
// is what makes DET_MIN_WIDTH_SAMPLES ("the shortest event worth reporting") a statement
// about the event's length instead of a statement about how much of it cleared the
// trigger (see the witness in config.h's DET_MIN_WIDTH_SAMPLES note).
static void testEnvelopeDuration() {
    // A = 4T: acos(-1/2) = pi/3, so the factor is exactly 3/2.
    CHECK(EventDetector::envelopeDuration(400, 100, 40) == 60,
          "A = 4T: expected 60, got %u",
          (unsigned)EventDetector::envelopeDuration(400, 100, 40));
    // A = T: the apex *is* the trigger, so there is no flank left to read the length
    // from - the relation is degenerate and the cap answers instead.
    CHECK(EventDetector::envelopeDuration(100, 100, 40) == 80,
          "A = T: expected the 2x cap (80), got %u",
          (unsigned)EventDetector::envelopeDuration(100, 100, 40));
    // No trigger level / no height: nothing to invert, the span comes back untouched.
    CHECK(EventDetector::envelopeDuration(800, 0, 40) == 40,
          "no trigger level: expected the span back, got %u",
          (unsigned)EventDetector::envelopeDuration(800, 0, 40));
    CHECK(EventDetector::envelopeDuration(0, 100, 40) == 40,
          "no height: expected the span back, got %u",
          (unsigned)EventDetector::envelopeDuration(0, 100, 40));
    // A >> T: the factor approaches 1 but never quite reaches it, because a raised
    // cosine's flanks only touch the baseline at its very ends.
    {
        const uint32_t wide = EventDetector::envelopeDuration(100000, 100, 40);
        CHECK(wide >= 40 && wide <= 44, "A >> T: expected ~40, got %u", (unsigned)wide);
    }
    // ... and the two properties that have to hold whatever the numbers are: the
    // result is never shorter than the span it was given, and never more than twice it.
    int below = 0, above = 0;
    for (int32_t a = 1; a <= 5000; a += 37) {
        const uint32_t d = EventDetector::envelopeDuration(a, 100, 33);
        if (d < 33) { below++; }
        if (d > 66) { above++; }
    }
    CHECK(below == 0 && above == 0,
          "the factor left [1, 2]: %d below the span, %d above twice it", below, above);
    std::printf("envelope duration: 1.5x at A = 4T, capped at 2x, span returned when "
                "there is nothing to invert\n");
}

int main() {
    std::printf("== peaks and glitches ==\n");
    testPeaksAndGlitches();
    std::printf("\n== impulses around a peak ==\n");
    testImpulseDoesNotBecomeMax();
    std::printf("\n== a 1 ms step is suppressed ==\n");
    testShortStepSuppressed();
    std::printf("\n== a 6 ms step is rejected ==\n");
    testFastAttackRejected();
    std::printf("\n== the rise test tolerates a dip ==\n");
    testDipTolerance();
    std::printf("\n== the quiet period ==\n");
    testQuietPeriodAndReference();
    std::printf("\n== impulse filter ==\n");
    testImpulseFilter();
    std::printf("\n== the duration the width bound judges ==\n");
    testEnvelopeDuration();

    if (g_failures) {
        std::printf("\nTEST FAILED (%d)\n", g_failures);
        return 1;
    }
    std::printf("\nTEST PASSED\n");
    return 0;
}
