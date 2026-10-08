// test/test_signal_model.cpp
//
// Host-side unit test for SignalModel (signal_model.h) - the randomized bench
// waveform the M4 drives out of its two DACs.  No Arduino needed: build & run it
// with test/run_tests.ps1 (needs g++ or clang++).
//
// Its Python twin is test/signal_model.py (`py -3 test/signal_model.py`), which
// checks the same properties on the float64 mirror.  Keeping both green is what
// makes that mirror trustworthy for the offline plot (plot_signals.py) and for the
// live datastream viewer (live_plot.py).
//
// What is checked:
//   1. Config defaults still describe the config.h waveform (they are copied into
//      Config by signal_gen.cpp makeConfig() on the board)
//   2. determinism        - same seed -> same train, same samples
//   3. cycle bookkeeping  - the position/cycle counters wrap, a new train is drawn
//                           at every cycle start
//   4. per-pulse rules    - gaps, widths, heights, glitch widths, cycle margin
//   5. distributions      - uniform gaps, Gaussian peaks, ~glitch_pct% glitches
//   6. the output itself  - every DAC code inside [min_code, max_code] and a peak
//                           apex that really reaches baseline + its drawn amplitude
//
// SignalModel::Config's defaults mirror the SIGNALGEN_* block of config.h (which
// the host build cannot include - it pulls in Arduino.h), so checkDefaults()
// re-reads them here: retuning the waveform means touching config.h,
// signal_model.h and test/signal_model.py's DEFAULTS together.

#include "../signal_model.h"
#include <algorithm>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <vector>

using sorter::PulseSpec;
using sorter::SignalModel;

namespace {

int g_failures = 0;

void check(const char* name, bool ok, const char* fmt = nullptr, ...) {
    printf("  %-4s %s", ok ? "ok" : "FAIL", name);
    if (fmt != nullptr) {
        char detail[200];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(detail, sizeof(detail), fmt, ap);
        va_end(ap);
        printf("   (%s)", detail);
    }
    printf("\n");
    if (!ok) { g_failures++; }
}

struct Stats {
    int    n    = 0;
    double mean = 0.0;
    double sd   = 0.0;
};

Stats stats(const std::vector<double>& v) {
    Stats s;
    s.n = (int)v.size();
    if (s.n == 0) { return s; }

    double sum = 0.0;
    for (size_t i = 0; i < v.size(); i++) { sum += v[i]; }
    s.mean = sum / (double)s.n;

    double var = 0.0;
    for (size_t i = 0; i < v.size(); i++) { var += (v[i] - s.mean) * (v[i] - s.mean); }
    s.sd = std::sqrt(var / (double)s.n);
    return s;
}

bool isRect(const PulseSpec& p) { return p.rect != 0u; }

// Everything the invariants and the statistics need, accumulated over many cycles.
struct TrainStats {
    std::vector<double> gaps, peak_w, peak_h, glitch_w, glitch_h;
    int bad_gap = 0, bad_span = 0, bad_width = 0, bad_height = 0;
    int max_pulses = 0;

    void add(const SignalModel& m) {
        const SignalModel::Config& c = m.config();
        const PulseSpec*           p = m.pulses();
        const uint32_t             n = m.pulseCount();

        if ((int)n > max_pulses) { max_pulses = (int)n; }

        uint32_t end = 0;                 // end of the previous pulse
        for (uint32_t i = 0; i < n; i++) {
            const uint32_t gap = p[i].start - end;
            if (gap < c.gap_min || gap > c.gap_max) { bad_gap++; }
            if (p[i].start + p[i].width + c.cycle_margin > c.cycle_samples) { bad_span++; }
            gaps.push_back((double)gap);

            if (isRect(p[i])) {
                glitch_w.push_back((double)p[i].width);
                glitch_h.push_back((double)p[i].amplitude);
                if (p[i].width < 1u || p[i].width > c.glitch_width_max) { bad_width++; }
                if (p[i].amplitude < c.glitch_height_min ||
                    p[i].amplitude > c.glitch_height_max) { bad_height++; }
            } else {
                peak_w.push_back((double)p[i].width);
                peak_h.push_back((double)p[i].amplitude);
                if (p[i].width < c.width_min || p[i].width > c.width_max) { bad_width++; }
                if (p[i].amplitude < c.height_min ||
                    p[i].amplitude > c.height_max) { bad_height++; }
            }
            end = p[i].start + p[i].width;
        }
    }
};

bool sameTrain(const std::vector<PulseSpec>& a, const std::vector<PulseSpec>& b) {
    if (a.size() != b.size()) { return false; }
    for (size_t i = 0; i < a.size(); i++) {
        if (a[i].start != b[i].start || a[i].width != b[i].width ||
            a[i].amplitude != b[i].amplitude || a[i].rect != b[i].rect) {
            return false;
        }
    }
    return true;
}

// The defaults must stay the documented waveform (config.h's SIGNALGEN_* block).
bool defaultsMatchConfig() {
    const SignalModel::Config c;
    return c.cycle_samples == 20000u && c.max_pulses == 16u && c.cycle_margin == 600u &&
           c.gap_min == 700u && c.gap_max == 3200u &&
           c.width_mean == 1200.0f && c.width_sd == 260.0f &&
           c.width_min == 300u && c.width_max == 3000u &&
           c.height_mean == 900.0f && c.height_sd == 300.0f &&
           c.height_min == 150 && c.height_max == 3500 &&
           c.glitch_pct == 25u && c.glitch_width_max == 6u &&
           c.glitch_height_mean == 1200.0f && c.glitch_height_sd == 600.0f &&
           c.glitch_height_min == 200 && c.glitch_height_max == 3500 &&
           c.baseline_code == 2048 && c.noise_code == 6 && c.noise_alpha == 1.0f &&
           c.min_code == 250 && c.max_code == 3850 && c.seed == 0x2F6E2B1u &&
           std::fabs(c.adc_per_dac - 2.64f) < 1e-6f;
}

void checkMean(const char* name, double got, double want, double tol,
               const char* unit, const char* extra = "") {
    check(name, std::fabs(got - want) <= tol, "%.0f %s, want %.0f +/- %.0f%s",
          got, unit, want, tol, extra);
}

} // namespace

int main() {
    const SignalModel::Config cfg;              // defaults == config.h
    const uint32_t           cycle = cfg.cycle_samples;
    const int                kCycles = 150;     // ~3 M samples, still well under 1 s

    printf("SignalModel host test - randomized bench waveform (signal_model.h)\n\n");

    // ---- 1. the defaults still describe the config.h waveform ---------------
    check("Config defaults mirror config.h's SIGNALGEN_* block", defaultsMatchConfig());

    // ---- 2. determinism -----------------------------------------------------
    SignalModel a(cfg), b(cfg);
    a.begin();
    b.begin();
    a.tick();
    b.tick();

    bool train_match = (a.pulseCount() == b.pulseCount());
    for (uint32_t i = 0; train_match && i < a.pulseCount(); i++) {
        train_match = (a.pulses()[i].start == b.pulses()[i].start) &&
                      (a.pulses()[i].width == b.pulses()[i].width) &&
                      (a.pulses()[i].amplitude == b.pulses()[i].amplitude) &&
                      (a.pulses()[i].rect == b.pulses()[i].rect);
    }
    check("same seed -> same pulse train", train_match,
          "%u pulses in cycle 0", (unsigned)a.pulseCount());

    bool codes_match = true;
    for (int i = 0; i < 4000 && codes_match; i++) { codes_match = (a.tick() == b.tick()); }
    check("same seed -> identical samples", codes_match);

    SignalModel::Config other = cfg;
    other.seed = 0xC0FFEEu;
    SignalModel c(other), d(cfg);
    c.begin();
    d.begin();
    bool seed_differs = false;
    for (int i = 0; i < 4000 && !seed_differs; i++) { seed_differs = (c.tick() != d.tick()); }
    check("different seed -> a different waveform", seed_differs);

    // ---- 3. many cycles through the firmware's own tick() path --------------
    SignalModel m(cfg);
    m.begin();

    TrainStats                  st;
    std::vector<int32_t>        code0;                     // cycle 0's samples
    std::vector<std::vector<PulseSpec> > trains;           // the first three trains
    code0.reserve(cycle);

    int32_t lo = 1000000, hi = -1000000;
    for (int cy = 0; cy < kCycles; cy++) {
        for (uint32_t i = 0; i < cycle; i++) {
            const int32_t code = m.tick();
            if (code < lo) { lo = code; }
            if (code > hi) { hi = code; }
            if (cy == 0) { code0.push_back(code); }
        }
        st.add(m);
        if (cy < 3) {
            std::vector<PulseSpec> t;
            for (uint32_t i = 0; i < m.pulseCount(); i++) { t.push_back(m.pulses()[i]); }
            trains.push_back(t);
        }
    }

    // ---- 4. cycle bookkeeping ----------------------------------------------
    check("a new random train is drawn every cycle",
          !sameTrain(trains[0], trains[1]) && !sameTrain(trains[1], trains[2]) &&
          !sameTrain(trains[0], trains[2]),
          "%u/%u/%u pulses in cycles 0/1/2", (unsigned)trains[0].size(),
          (unsigned)trains[1].size(), (unsigned)trains[2].size());
    check("tick() wraps at cycle_samples and counts whole cycles",
          m.cycles() == (uint32_t)kCycles && m.position() == 0u,
          "cycles=%u position=%u", (unsigned)m.cycles(), (unsigned)m.position());
    check("1..max_pulses pulses per cycle",
          st.max_pulses > 0 && st.max_pulses <= (int)cfg.max_pulses,
          "busiest cycle: %d of %u allowed", st.max_pulses, (unsigned)cfg.max_pulses);

    // ---- 5. the DAC codes the M4 would drive out ----------------------------
    check("every DAC code inside [min_code, max_code]",
          lo >= cfg.min_code && hi <= cfg.max_code,
          "saw %d..%d, allowed %d..%d", (int)lo, (int)hi,
          (int)cfg.min_code, (int)cfg.max_code);
    check("the waveform is far from flat", hi - lo > 100, "span=%d codes", (int)(hi - lo));

    const PulseSpec* tallest = nullptr;
    for (size_t i = 0; i < trains[0].size(); i++) {
        const PulseSpec& p = trains[0][i];
        if (!isRect(p) && (tallest == nullptr || p.amplitude > tallest->amplitude)) {
            tallest = &p;
        }
    }

    if (tallest == nullptr) {
        check("cycle 0 drew a peak to check the apex on", false, "only glitches were drawn");
    } else {
        int32_t apex = 0;
        for (uint32_t i = tallest->start; i < tallest->start + tallest->width; i++) {
            if (code0[i] > apex) { apex = code0[i]; }
        }
        const double expect = (double)cfg.baseline_code +
                              (double)tallest->amplitude / (double)cfg.adc_per_dac;
        check("the tallest peak's apex reaches baseline + its amplitude",
              std::fabs((double)apex - expect) <= (double)cfg.noise_code + 2.0,
              "apex=%d, expected %.0f (amp %d codes over %.3f adc-per-dac)",
              (int)apex, expect, (int)tallest->amplitude, (double)cfg.adc_per_dac);
    }

    // ---- 6. the per-pulse rules hold for every pulse of every cycle ---------
    check("every gap inside [gap_min, gap_max]",
          st.bad_gap == 0, "%d of %u gaps outside %u..%u", st.bad_gap,
          (unsigned)st.gaps.size(), (unsigned)cfg.gap_min, (unsigned)cfg.gap_max);
    check("every pulse stays clear of the cycle boundary", st.bad_span == 0,
          "%d of %u pulses too close to the end", st.bad_span, (unsigned)st.gaps.size());
    check("every width inside its drawn range", st.bad_width == 0,
          "%d of %u widths", st.bad_width,
          (unsigned)(st.peak_w.size() + st.glitch_w.size()));
    check("every height inside its drawn range", st.bad_height == 0,
          "%d of %u heights", st.bad_height,
          (unsigned)(st.peak_h.size() + st.glitch_h.size()));

    // ---- 7. the requested distributions ------------------------------------
    const Stats gap = stats(st.gaps), pw = stats(st.peak_w), ph = stats(st.peak_h);
    const Stats gw = stats(st.glitch_w), gh = stats(st.glitch_h);
    char        buf[80];

    // A uniform draw over [gap_min, gap_max] has sd = (max - min + 1) / sqrt(12).
    const double gap_sd_uniform = ((double)cfg.gap_max - (double)cfg.gap_min + 1.0) /
                                  std::sqrt(12.0);
    snprintf(buf, sizeof(buf), " (sd %.0f, uniform sd %.0f)", gap.sd, gap_sd_uniform);
    checkMean("gap mean is uniform(gap_min..gap_max)", gap.mean,
              0.5 * ((double)cfg.gap_min + (double)cfg.gap_max),
              0.15 * gap_sd_uniform, "smp", buf);

    checkMean("peak height mean ~ height_mean (Gaussian)", ph.mean, (double)cfg.height_mean,
              50.0, "codes");
    checkMean("peak height sd   ~ height_sd   (Gaussian)", ph.sd, (double)cfg.height_sd,
              50.0, "codes");
    checkMean("peak width  mean ~ width_mean  (Gaussian)", pw.mean, (double)cfg.width_mean,
              50.0, "smp");
    checkMean("peak width  sd   ~ width_sd    (Gaussian)", pw.sd, (double)cfg.width_sd,
              50.0, "smp");

    // A glitch is a rectangle 1..glitch_width_max samples wide, raised by a
    // Gaussian height: the shapes the peak detector has to reject.
    checkMean("glitch width mean ~ 1..glitch_width_max", gw.mean,
              (1.0 + (double)cfg.glitch_width_max) / 2.0, 0.75, "smp");
    checkMean("glitch height mean ~ glitch_height_mean", gh.mean,
              (double)cfg.glitch_height_mean, 120.0, "codes");

    const double total = (double)(st.glitch_w.size() + st.peak_w.size());
    const double share = (total > 0.0) ? 100.0 * (double)st.glitch_w.size() / total : 0.0;
    snprintf(buf, sizeof(buf), " (%u glitches / %.0f pulses)",
             (unsigned)st.glitch_w.size(), total);
    checkMean("glitch share ~ glitch_pct of the pulses", share, (double)cfg.glitch_pct,
              8.0, "%", buf);

    // ---- summary ------------------------------------------------------------
    printf("\ngaps %.0f..%.0f (mean %.0f)   peaks %u (h %.0f +/- %.0f, w %.0f +/- %.0f)"
           "   glitches %u\n",
           *std::min_element(st.gaps.begin(), st.gaps.end()),
           *std::max_element(st.gaps.begin(), st.gaps.end()), gap.mean,
           (unsigned)st.peak_w.size(), ph.mean, ph.sd, pw.mean, pw.sd,
           (unsigned)st.glitch_w.size());

    if (g_failures != 0) {
        printf("\nTEST FAILED (%d)\n", g_failures);
        return 1;
    }
    printf("\nTEST PASSED\n");
    return 0;
}

