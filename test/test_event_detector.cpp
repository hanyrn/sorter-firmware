// test/test_event_detector.cpp
//
// Host-side unit test for EventDetector (no Arduino required).
// Build & run it with test/run_tests.ps1 (needs g++ or clang++).
//
// Scenario (mirrors test/model_check.py):
//   baseline 2000 +/- 5, three broad peaks (amplitude 800, sigma 200 samples),
//   a 1-sample glitch and a 4-sample glitch. The three broad peaks must be
//   reported as VALID; the two glitches must be rejected.

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

EventDetector::Config testConfig() {
    EventDetector::Config c;
    c.baseline_alpha    = 0.001f;
    c.noise_alpha       = 0.001f;
    c.k_on              = 5.0f;
    c.k_off             = 2.0f;
    c.k_gate            = 3.0f;
    c.sigma_floor       = 1;
    c.warmup_samples    = 3000;
    c.end_debounce      = 2;
    c.min_max_value     = 80;
    c.min_width_samples = 3;
    c.max_width_samples = 200000;
    c.min_area          = 0;
    c.max_slope         = 100.0f;
    return c;
}

int32_t signalAt(int i) {
    const double baseline = 2000.0, noise = 5.0;
    struct Peak { int c, w; double amp; };
    const Peak peaks[3] = { {6000, 200, 800.0}, {13000, 200, 800.0}, {20000, 200, 800.0} };
    double x = baseline + gauss() * noise;
    for (int p = 0; p < 3; p++) {
        const double d = (double)(i - peaks[p].c);
        x += peaks[p].amp * std::exp(-(d * d) / (2.0 * peaks[p].w * peaks[p].w));
    }
    if (i == 9000)                       x += 900.0;   // 1-sample glitch
    if (i >= 16000 && i < 16004)         x += 900.0;   // 4-sample glitch
    return (int32_t)std::lround(x);
}

} // namespace

int main() {
    EventDetector det(testConfig());
    PeakMetrics m;
    int closes = 0, valid = 0, invalid = 0, failures = 0;

    for (int i = 0; i < 26000; i++) {
        if (det.update(signalAt(i), (uint32_t)i * 10u, (uint32_t)i, m)) {
            closes++;
            printf("event #%u  %s  reason=%u  max=%d  width=%u smp (%u us)  area=%llu\n",
                   (unsigned)m.index, m.valid ? "VALID" : "rejected", (unsigned)m.reason,
                   (int)m.max_value, (unsigned)m.width_samples, (unsigned)m.width_us,
                   (unsigned long long)m.area);
            if (m.valid) {
                valid++;
                if (m.max_value < 650 || m.max_value > 950) {
                    printf("  ! peak amplitude out of range\n"); failures++;
                }
                if (m.width_samples < 600 || m.width_samples > 1700) {
                    printf("  ! peak width out of range\n"); failures++;
                }
            } else {
                invalid++;
                if (m.reason != REASON_WIDTH_TOO_NARROW && m.reason != REASON_SHORT_FOR_HEIGHT) {
                    printf("  ! unexpected rejection reason %u\n", (unsigned)m.reason);
                    failures++;
                }
            }
        }
    }

    printf("\nclosed=%d valid=%d invalid=%d  baseline=%.1f sigma=%.2f\n",
           closes, valid, invalid, det.baseline(), det.sigma());
    if (valid != 3)   { printf("FAIL: expected 3 valid peaks\n");      failures++; }
    if (invalid != 2) { printf("FAIL: expected 2 rejected glitches\n"); failures++; }
    if (std::fabs(det.baseline() - 2000.0) > 25.0) {
        printf("FAIL: baseline did not track to ~2000\n"); failures++;
    }

    if (failures) { printf("\nTEST FAILED (%d)\n", failures); return 1; }
    printf("\nTEST PASSED\n");
    return 0;
}
