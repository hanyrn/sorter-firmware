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
//   5. the merged pairs   - how many there are, the span of a pair and its overlap
//   6. distributions      - uniform gaps, Gaussian peaks, ~glitch_pct% glitches
//   7. event widths       - the double-peaked width distribution the merged pairs
//                           exist to produce (the histogram is printed)
//   8. the output itself  - every DAC code inside [min_code, max_code], the rail
//                           really reached (saturated peaks), impulse noise present,
//                           every peak reaching its drawn amplitude, and an apex
//                           that lands on baseline + amplitude
//
// The defaults mirror the SIGNALGEN_* block of config.h (which the host build cannot
// include - it pulls in Arduino.h), so defaultsMatchConfig() re-reads them here:
// retuning the waveform means touching config.h, signal_model.h and
// test/signal_model.py's DEFAULTS together.

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
        char detail[240];
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

double median(std::vector<double> v) {
    if (v.empty()) { return 0.0; }
    std::sort(v.begin(), v.end());
    return v[v.size() / 2u];
}

bool isRect(const PulseSpec& p) { return p.rect != 0u; }

// Bucket the values into [lo, hi), returning the fullest bucket's count.
int bucketed(const std::vector<double>& v, double lo, double hi, int buckets,
             std::vector<int>& counts) {
    counts.assign((size_t)buckets, 0);
    for (size_t i = 0; i < v.size(); i++) {
        int b = (int)((v[i] - lo) * (double)buckets / (hi - lo));
        if (b < 0) { b = 0; }
        if (b >= buckets) { b = buckets - 1; }
        counts[(size_t)b]++;
    }

    int top = 0;
    for (int b = 0; b < buckets; b++) {
        if (counts[(size_t)b] > top) { top = counts[(size_t)b]; }
    }
    return top;
}

// The two modes and the valley a "double-peaked" distribution has to show: the
// fullest bucket, the point where the histogram stops falling on either side of it
// (the valley), and the fullest bucket beyond that valley.
void histogramModes(const std::vector<double>& v, double lo, double hi, int buckets,
                    int& mode1, int& mode2, int& valley) {
    std::vector<int> counts;
    bucketed(v, lo, hi, buckets, counts);

    int m1 = 0;
    for (int b = 1; b < buckets; b++) {
        if (counts[(size_t)b] > counts[(size_t)m1]) { m1 = b; }
    }

    int right_dip = m1;
    for (int b = m1; b + 1 < buckets && counts[(size_t)(b + 1)] < counts[(size_t)b];
         b++) {
        right_dip = b + 1;
    }
    int right_mode = -1;
    for (int b = right_dip + 1; b < buckets; b++) {
        if (right_mode < 0 || counts[(size_t)b] > counts[(size_t)right_mode]) {
            right_mode = b;
        }
    }

    int left_dip = m1;
    for (int b = m1; b > 0 && counts[(size_t)(b - 1)] < counts[(size_t)b]; b--) {
        left_dip = b - 1;
    }
    int left_mode = -1;
    for (int b = left_dip - 1; b >= 0; b--) {
        if (left_mode < 0 || counts[(size_t)b] > counts[(size_t)left_mode]) {
            left_mode = b;
        }
    }

    const int right_peak = (right_mode < 0) ? 0 : counts[(size_t)right_mode];
    const int left_peak  = (left_mode < 0) ? 0 : counts[(size_t)left_mode];

    mode1 = counts[(size_t)m1];
    if (left_peak > right_peak) {
        mode2  = left_peak;
        valley = counts[(size_t)left_dip];
    } else {
        mode2  = right_peak;
        valley = counts[(size_t)right_dip];
    }
}

void printHistogram(const char* label, const std::vector<double>& v, double lo,
                    double hi, int buckets) {
    std::vector<int> counts;
    const int        top = bucketed(v, lo, hi, buckets, counts);

    printf("       %s\n", label);
    for (int b = 0; b < buckets; b++) {
        printf("       %6.0f ", lo + (hi - lo) * (double)b / (double)buckets);
        for (int k = 0; k < 44 * counts[(size_t)b] / (top != 0 ? top : 1); k++) {
            printf("#");
        }
        printf(" %d\n", counts[(size_t)b]);
    }
}

// Everything the invariants and the statistics need, accumulated over many cycles.
//
// An "event" here is what a detector times: pulses that overlap each other add up in
// the waveform, so one event covers all of them.  A lone glitch (a 1..6-sample
// rectangle) is an event too, but not a *width*, so glitch events are counted
// separately and kept out of the width statistics.
struct TrainStats {
    std::vector<double> gaps, peak_w, peak_h, glitch_w, glitch_h;
    std::vector<double> merged_span;   // first pulse start -> second pulse end
    std::vector<double> overlap;       // overlap / first width (peaks only)
    std::vector<double> peak_span;     // event spans, glitch events excluded
    std::vector<double> single_span;   // ... of the events covering one pulse
    std::vector<double> pair_span;     // ... of the events covering two pulses
    int bad_gap = 0, bad_span = 0, bad_width = 0, bad_height = 0;
    int merged = 0, weak_overlap = 0, glitch_events = 0, long_events = 0;
    int max_pulses = 0;

    void add(const SignalModel& m) {
        const SignalModel::Config& c = m.config();
        const PulseSpec*           p = m.pulses();
        const uint32_t             n = m.pulseCount();

        if ((int)n > max_pulses) { max_pulses = (int)n; }

        ev_pulses = 0;

        uint32_t end        = 0;      // end of the furthest pulse so far
        uint32_t prev_start = 0;      // start of the immediately previous pulse
        uint32_t prev_w     = 0;

        for (uint32_t i = 0; i < n; i++) {
            const uint32_t start = p[i].start;
            const uint32_t stop  = start + p[i].width;

            if (i > 0u && start < prev_start + prev_w) {
                // Placed inside the previous pulse: a merged pair.
                merged++;
                merged_span.push_back((double)(stop - prev_start));
                if (!isRect(p[i]) && prev_w >= 100u) {
                    const double ov = (double)(prev_start + prev_w - start) /
                                      (double)prev_w;
                    overlap.push_back(ov);
                    if (ov < 0.9 * (double)c.merge_min_overlap) { weak_overlap++; }
                }
            } else {
                const uint32_t gap = start - end;
                if (gap < c.gap_min || gap > c.gap_max) { bad_gap++; }
                gaps.push_back((double)gap);
            }

            if (stop + c.cycle_margin > c.cycle_samples) { bad_span++; }

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

            // Group the pulses into the events a detector would time.
            if (ev_pulses == 0 || start >= ev_end) {
                if (ev_pulses != 0) { closeEvent(); }
                ev_start  = start;
                ev_end    = stop;
                ev_pulses = 1;
                ev_glitch = isRect(p[i]);
            } else {
                if (stop > ev_end) { ev_end = stop; }
                ev_pulses++;
                ev_glitch = ev_glitch || isRect(p[i]);
            }

            end        = (stop > end) ? stop : end;
            prev_start = start;
            prev_w     = p[i].width;
        }
        if (ev_pulses != 0) { closeEvent(); }
    }

private:
    // The event currently being grouped (every add() closes the last one).
    uint32_t ev_start  = 0;
    uint32_t ev_end    = 0;
    int      ev_pulses = 0;
    bool     ev_glitch = false;

    void closeEvent() {
        const double span = (double)(ev_end - ev_start);
        if (ev_glitch) {
            glitch_events++;
        } else {
            peak_span.push_back(span);
            if (ev_pulses == 1)      { single_span.push_back(span); }
            else if (ev_pulses == 2) { pair_span.push_back(span); }
            else                     { long_events++; }
        }
        ev_pulses = 0;
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
    return c.cycle_samples == 20000u && c.max_pulses == 48u && c.cycle_margin == 600u &&
           c.gap_min == 30u && c.gap_max == 900u &&
           c.merge_pct == 80u && std::fabs(c.merged_span_min - 1.50f) < 1e-6f &&
           std::fabs(c.merged_span_max - 2.00f) < 1e-6f &&
           std::fabs(c.merge_min_overlap - 0.12f) < 1e-6f &&
           c.width_mean == 1200.0f && c.width_sd == 260.0f &&
           c.width_min == 30u && c.width_max == 3000u && c.width_span_pct == 40u &&
           c.height_mean == 1800.0f && c.height_sd == 600.0f &&
           c.height_span_pct == 30u && c.height_min == 320 && c.height_max == 40000 &&
           c.frontier_pct == 50u && c.frontier_width_min == 200u &&
           c.frontier_width_max == 800u && c.frontier_trigger == 200 &&
           c.frontier_rise == 20u &&
           std::fabs(c.frontier_margin_min - 2.5f) < 1e-6f &&
           std::fabs(c.frontier_margin_max - 5.0f) < 1e-6f &&
           c.ladder == 0u && c.ladder_width == 600u && c.ladder_min == 40 &&
           c.ladder_step == 80 && c.ladder_count == 12u &&
           c.glitch_pct == 40u && c.glitch_width_max == 6u &&
           c.glitch_height_mean == 3600.0f && c.glitch_height_sd == 2200.0f &&
           c.glitch_height_min == 240 && c.glitch_height_max == 40000 &&
           c.baseline_code == 2048 && c.noise_code == 16 && c.noise_alpha == 1.0f &&
           c.baseline_wave_code == 150 && c.baseline_wave_samples == 300000u &&
           std::fabs(c.spike_pct - 0.05f) < 1e-6f && c.spike_max == 400 &&
           c.min_code == 250 && c.max_code == 3850 && c.seed == 0x2F6E2B1u &&
           std::fabs(c.adc_per_dac - 5.28f) < 1e-6f;
}

void checkMean(const char* name, double got, double want, double tol,
               const char* unit, const char* extra = "") {
    check(name, std::fabs(got - want) <= tol, "%.0f %s, want %.0f +/- %.0f%s",
          got, unit, want, tol, extra);
}

// Pulses whose rendered apex does not reach baseline + their drawn amplitude.
// Neighbours and impulses can only *add* to a sample, so every pulse has to reach at
// least its own amplitude - unless that amplitude is past the DAC rail, in which case
// the rail is as far as the source can go.  The baseline wander is why this is an
// exact statement rather than an approximate one: the level a pulse sits on is the
// wander *at its apex sample*, not the nominal DC code.
int peaksShortOfAmplitude(const SignalModel::Config& c,
                          const std::vector<int32_t>& codes,
                          const std::vector<std::vector<PulseSpec> >& trains) {
    int short_pulses = 0;
    for (size_t cy = 0; cy < trains.size(); cy++) {
        const size_t base = cy * (size_t)c.cycle_samples;
        for (size_t i = 0; i < trains[cy].size(); i++) {
            const PulseSpec& p = trains[cy][i];

            size_t  at   = base + p.start;
            int32_t apex = codes[at];
            for (uint32_t k = 1; k < p.width; k++) {
                const int32_t v = codes[base + p.start + k];
                if (v > apex) { apex = v; at = base + p.start + k; }
            }

            const float want = (float)c.baseline_code +
                               sorter::baselineWaveCode(c.baseline_wave_code,
                                                        c.baseline_wave_samples,
                                                        (uint32_t)at) +
                               (float)p.amplitude / c.adc_per_dac;
            const float reach = (want < (float)c.max_code) ? want : (float)c.max_code;
            if ((float)apex < reach - (float)c.noise_code - 2.0f) { short_pulses++; }
        }
    }
    return short_pulses;
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

    TrainStats                           st;
    std::vector<int32_t>                 codes;   // every sample of every cycle
    std::vector<std::vector<PulseSpec> > trains;  // the train each cycle used
    codes.reserve((size_t)cycle * (size_t)kCycles);
    trains.reserve((size_t)kCycles);

    int32_t lo = 1000000, hi = -1000000;
    for (int cy = 0; cy < kCycles; cy++) {
        for (uint32_t i = 0; i < cycle; i++) {
            const int32_t code = m.tick();
            if (code < lo) { lo = code; }
            if (code > hi) { hi = code; }
            codes.push_back(code);
        }
        st.add(m);

        std::vector<PulseSpec> t;
        for (uint32_t i = 0; i < m.pulseCount(); i++) { t.push_back(m.pulses()[i]); }
        trains.push_back(t);
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

    // ---- 4b. the slow baseline wander ---------------------------------------
    // The DC level the pulses sit on drifts, and it has to drift in a way no peak
    // bound can mistake for a signal:
    //   * one sine of +/-baseline_wave_code codes over baseline_wave_samples;
    //   * continuous across the pattern-cycle boundary (its clock is the *global*
    //     sample index, never the position inside the cycle);
    //   * far slower than anything the bounds are written for: 150 codes over
    //     300 000 samples is at most 0.004 codes per sample, four orders of
    //     magnitude below DET_MAX_SLOPE;
    //   * and present in the rendered codes exactly as the shared function says -
    //     which is what test_bench_detect.cpp's ground truth leans on.
    const int32_t  wave_amp = cfg.baseline_wave_code;
    const uint32_t wave_per = cfg.baseline_wave_samples;
    const float    quarter  = sorter::baselineWaveCode(wave_amp, wave_per, wave_per / 4u);
    const float    three_q  = sorter::baselineWaveCode(wave_amp, wave_per,
                                                       3u * wave_per / 4u);
    check("the wander is a sine of +/-150 codes over 300 000 samples (30 s)",
          std::fabs(sorter::baselineWaveCode(wave_amp, wave_per, 0u)) < 1e-4f &&
          std::fabs(sorter::baselineWaveCode(wave_amp, wave_per, wave_per)) < 1e-4f &&
          std::fabs(quarter - (float)wave_amp) < 1e-3f &&
          std::fabs(three_q + (float)wave_amp) < 1e-3f,
          "0 / %+.0f / 0 / %+.0f codes at 0, 1/4, 1/2, 3/4 of the period",
          (double)quarter, (double)three_q);
    check("the wander is continuous across the pattern-cycle boundary",
          std::fabs(sorter::baselineWaveCode(wave_amp, wave_per, cycle) -
                    sorter::baselineWaveCode(wave_amp, wave_per, cycle - 1u)) < 0.01f,
          "%.4f -> %.4f codes at sample %u",
          (double)sorter::baselineWaveCode(wave_amp, wave_per, cycle - 1u),
          (double)sorter::baselineWaveCode(wave_amp, wave_per, cycle),
          (unsigned)cycle);

    float fastest = 0.0f;
    for (uint32_t i = 0; i < wave_per; i += 997u) {
        const float d = std::fabs(sorter::baselineWaveCode(wave_amp, wave_per, i + 1u) -
                                  sorter::baselineWaveCode(wave_amp, wave_per, i));
        if (d > fastest) { fastest = d; }
    }
    check("the wander moves far slower than any peak bound", fastest < 0.01f,
          "fastest step %.4f codes/sample (DET_MAX_SLOPE is 100)", (double)fastest);

    {
        // The impulses have to be off to see the wander exactly: a +/-400-count
        // spike lands on an idle sample as happily as on a peak, and it is the one
        // term that is not the DC level.  With it out of the way an idle sample IS
        // the DC level + the wander + the dither.
        SignalModel::Config qc = cfg;
        qc.spike_pct = 0.0f;
        SignalModel qm(qc);
        qm.begin();

        std::vector<int32_t>                 q_codes;
        std::vector<std::vector<PulseSpec> > q_trains;
        const int kQuietCycles = 8;      // 16 s: past the 7.5 s peak and the zero
                                         // crossing at 15 s
        for (int cy = 0; cy < kQuietCycles; cy++) {
            for (uint32_t i = 0; i < cycle; i++) { q_codes.push_back(qm.tick()); }
            std::vector<PulseSpec> t;
            for (uint32_t i = 0; i < qm.pulseCount(); i++) { t.push_back(qm.pulses()[i]); }
            q_trains.push_back(t);
        }

        int     idle_n = 0;
        int32_t worst  = 0;
        int32_t lifted = -1000000;
        int32_t dipped =  1000000;
        for (size_t i = 0; i < q_codes.size(); i += 37u) {
            const size_t   cy  = i / (size_t)cycle;
            const uint32_t loc = (uint32_t)(i % (size_t)cycle);
            bool           covered = false;
            for (size_t k = 0; k < q_trains[cy].size(); k++) {
                const PulseSpec& p = q_trains[cy][k];
                if (loc >= p.start && loc < p.start + p.width) { covered = true; }
            }
            if (covered) { continue; }

            const int32_t want = (int32_t)lroundf(
                (float)cfg.baseline_code +
                sorter::baselineWaveCode(wave_amp, wave_per, (uint32_t)i));
            const int32_t dev = q_codes[i] - want;
            const int32_t mag = (dev < 0) ? -dev : dev;
            if (mag > worst) { worst = mag; }
            // Measured against the *nominal* DC code, not against `want`: what has
            // to swing up and down is the level itself.
            const int32_t level = q_codes[i] - cfg.baseline_code;
            if (level > lifted) { lifted = level; }
            if (level < dipped) { dipped = level; }
            idle_n++;
        }

        check("an idle sample is exactly baseline + wander + dither",
              idle_n > 0 && worst <= cfg.noise_code + 1,
              "worst |idle - (baseline + wander)| = %d codes over %d idle samples",
              (int)worst, idle_n);
        check("the idle level really does wander up and down",
              (double)lifted > 0.9 * (double)wave_amp && dipped < 0,
              "+%d / %d codes over %d idle samples", (int)lifted, (int)dipped, idle_n);
    }

    // ---- 5. the per-pulse rules hold for every pulse of every cycle ---------
    const double pulses = (double)(st.peak_w.size() + st.glitch_w.size());
    check("separated pulses keep gaps inside [gap_min, gap_max]", st.bad_gap == 0,
          "%d of %u gaps outside %u..%u", st.bad_gap, (unsigned)st.gaps.size(),
          (unsigned)cfg.gap_min, (unsigned)cfg.gap_max);
    check("every pulse stays clear of the cycle boundary", st.bad_span == 0,
          "%d of %.0f pulses too close to the end", st.bad_span, pulses);
    check("every width inside its drawn range", st.bad_width == 0,
          "%d of %.0f widths", st.bad_width, pulses);
    check("every height inside its drawn range", st.bad_height == 0,
          "%d of %.0f heights", st.bad_height, pulses);

    // ---- 6. the merged pairs -----------------------------------------------
    // A merged pulse is *placed* inside the previous one so the pair spans a fixed
    // fraction of width_mean.  How many of the merge_pct draws survive depends on the
    // widths that happen to be drawn (a pair of very narrow pulses has no room), so
    // the share is only bounded - but every rule of the mechanism is checked.
    const double merged_pct = 100.0 * (double)st.merged / pulses;
    check("a fair share of the pulses merge with the previous one",
          merged_pct > 5.0 && merged_pct < (double)cfg.merge_pct,
          "%.0f%% merged (%d of %.0f pulses, at most the drawn %u%%)",
          merged_pct, st.merged, pulses, (unsigned)cfg.merge_pct);

    const double span_lo = (double)cfg.merged_span_min * (double)cfg.width_mean;
    const double span_hi = (double)cfg.merged_span_max * (double)cfg.width_mean;
    const double span_seen_lo = *std::min_element(st.merged_span.begin(),
                                                  st.merged_span.end());
    const double span_seen_hi = *std::max_element(st.merged_span.begin(),
                                                  st.merged_span.end());
    check("a merged pair spans merged_span_min..max x width_mean",
          span_seen_lo >= span_lo - 1.0 && span_seen_hi <= span_hi + 1.0,
          "%.0f..%.0f samples over %u pairs (%.1f..%.1f x width_mean)",
          span_seen_lo, span_seen_hi, (unsigned)st.merged_span.size(),
          (double)cfg.merged_span_min, (double)cfg.merged_span_max);

    const double ov_least = st.overlap.empty()
                                ? 0.0
                                : *std::min_element(st.overlap.begin(),
                                                    st.overlap.end());
    check("its pulses overlap by at least merge_min_overlap of the first one",
          st.weak_overlap == 0 && !st.overlap.empty(),
          "least %.2f of %u peak pairs (%d overlapping too little)",
          ov_least, (unsigned)st.overlap.size(), st.weak_overlap);
    check("a merged event covers exactly two pulses", st.long_events == 0,
          "%d events covered three or more pulses (%d glitch events)",
          st.long_events, st.glitch_events);

    // ---- 7. the requested distributions ------------------------------------
    const Stats gap = stats(st.gaps), pw = stats(st.peak_w), ph = stats(st.peak_h);
    const Stats gw = stats(st.glitch_w), gh = stats(st.glitch_h);
    char        buf[110];

    // A uniform draw over [gap_min, gap_max] has sd = (max - min + 1) / sqrt(12).
    const double gap_sd_uniform = ((double)cfg.gap_max - (double)cfg.gap_min + 1.0) /
                                  std::sqrt(12.0);
    snprintf(buf, sizeof(buf), " (sd %.0f, uniform sd %.0f)", gap.sd, gap_sd_uniform);
    checkMean("gap mean is uniform(gap_min..gap_max)", gap.mean,
              0.5 * ((double)cfg.gap_min + (double)cfg.gap_max),
              0.15 * gap_sd_uniform, "smp", buf);

    // Peak width is a two-part draw, exactly like the peak height: width_span_pct %
    // of the peaks are log-uniform over [width_min, width_mean] (the 3 ms needles),
    // the rest is the nominal Gaussian.  So the Gaussian's own moments are measured
    // on a draw with the slice switched off, and the mixed draw is checked for what
    // the slice exists to guarantee - that the narrow end of the range really is
    // represented, in the share a log-uniform draw implies.  The small-peak
    // (frontier) slice is switched off in the same run, or it would be measured
    // inside the Gaussian's moments too.
    {
        SignalModel::Config g = cfg;
        g.width_span_pct = 0u;
        g.frontier_pct   = 0u;
        SignalModel gm(g);
        gm.begin();
        std::vector<double> gauss_w, nominal_h;
        for (int cy = 0; cy < kCycles; cy++) {
            for (uint32_t i = 0; i < cycle; i++) { gm.tick(); }
            for (uint32_t i = 0; i < gm.pulseCount(); i++) {
                if (!isRect(gm.pulses()[i])) {
                    gauss_w.push_back((double)gm.pulses()[i].width);
                    nominal_h.push_back((double)gm.pulses()[i].amplitude);
                }
            }
        }
        const Stats gws = stats(gauss_w);
        checkMean("peak width mean ~ width_mean (the Gaussian alone)", gws.mean,
                  (double)cfg.width_mean, 50.0, "smp");
        checkMean("peak width sd   ~ width_sd   (the Gaussian alone)", gws.sd,
                  (double)cfg.width_sd, 50.0, "smp");
        // ... and the height median, on the same run: the frontier slice pulls the
        // *mixed* median down on purpose (it is the small end of the train), so the
        // nominal figure is measured with it out of the way.
        checkMean("peak height median ~ height_mean (the nominal draw alone)",
                  median(nominal_h), (double)cfg.height_mean, 240.0, "codes");
    }

    const double narrowest = *std::min_element(st.peak_w.begin(), st.peak_w.end());
    const double needle_cut = (double)cfg.width_mean / 4.0;
    int          needle_n   = 0;
    for (size_t i = 0; i < st.peak_w.size(); i++) {
        if (st.peak_w[i] < needle_cut) { needle_n++; }
    }
    const double needle_pct = 100.0 * (double)needle_n / (double)st.peak_w.size();
    check("the narrow end of the width span is really drawn (needles)",
          narrowest <= 1.25 * (double)cfg.width_min,
          "narrowest of %u peaks: %.0f samples (width_min %u)",
          (unsigned)st.peak_w.size(), narrowest, (unsigned)cfg.width_min);
    check("needles ~ width_span_pct of the peaks below width_mean/4",
          needle_pct > 12.0 && needle_pct < 38.0,
          "%.0f%% below %.0f samples (%d of %u peaks)", needle_pct, needle_cut,
          needle_n, (unsigned)st.peak_w.size());

    // Peak height is a two-part draw: height_span_pct % of the peaks are log-uniform
    // over the whole span (the extremes), the rest is the nominal Gaussian - so it is
    // the *median* that tracks height_mean.  That median is measured above, on a draw
    // with the small-peak slice switched off; what the mixed draw has to keep is the
    // shape of the mix: a *minority* of the peaks is small.
    {
        int small = 0;
        for (size_t i = 0; i < st.peak_h.size(); i++) {
            if (st.peak_h[i] < 1000.0) { small++; }
        }
        const double small_pct = 100.0 * (double)small / (double)st.peak_h.size();
        check("the small-peak slice is a minority of the peaks",
              small_pct > 5.0 && small_pct < 40.0,
              "%.0f%% of %u peaks are under 1000 counts (the slice is %u%%)",
              small_pct, (unsigned)st.peak_h.size(), (unsigned)cfg.frontier_pct);
    }

    // A glitch is a rectangle 1..glitch_width_max samples wide, raised by a Gaussian
    // height: the shapes the peak detector has to reject.
    checkMean("glitch width mean ~ 1..glitch_width_max", gw.mean,
              (1.0 + (double)cfg.glitch_width_max) / 2.0, 0.75, "smp");
    checkMean("glitch height mean ~ glitch_height_mean", gh.mean,
              (double)cfg.glitch_height_mean, 360.0, "codes");

    const double share = (pulses > 0.0) ? 100.0 * (double)st.glitch_w.size() / pulses
                                        : 0.0;
    snprintf(buf, sizeof(buf), " (%.0f glitches / %.0f pulses)",
             (double)st.glitch_w.size(), pulses);
    checkMean("glitch share ~ glitch_pct of the pulses", share,
              (double)cfg.glitch_pct, 8.0, "%", buf);

    // ---- 8. the event widths: two modes, and a dip between them ------------
    const double pair_pct = st.peak_span.empty()
                                ? 0.0
                                : 100.0 * (double)st.pair_span.size() /
                                      (double)st.peak_span.size();
    check("merged pairs are a fair share of the peak events",
          !st.pair_span.empty() && pair_pct > 10.0,
          "%u of %u peak events cover 2 pulses (%d glitch events ignored)",
          (unsigned)st.pair_span.size(), (unsigned)st.peak_span.size(),
          st.glitch_events);
    check("merged events are much wider (the second mode)",
          median(st.pair_span) > median(st.single_span) + 0.4 * (double)cfg.width_mean,
          "median %.0f vs %.0f samples (%u vs %u events)", median(st.pair_span),
          median(st.single_span), (unsigned)st.pair_span.size(),
          (unsigned)st.single_span.size());

    int mode1 = 0, mode2 = 0, valley = 0;
    histogramModes(st.peak_span, 0.0, 3000.0, 20, mode1, mode2, valley);
    check("the width distribution is double peaked (a dip between the modes)",
          valley < 0.5 * (double)std::min(mode1, mode2),
          "modes %d and %d events wide, dip %d (150-sample buckets)", mode1, mode2,
          valley);
    printHistogram("peak widths (glitch events excluded) - bimodal on purpose:",
                   st.peak_span, 0.0, 3000.0, 20);

    // ---- 9. the DAC codes the M4 would drive out ---------------------------
    check("every DAC code inside [min_code, max_code]",
          lo >= cfg.min_code && hi <= cfg.max_code,
          "saw %d..%d, allowed %d..%d", (int)lo, (int)hi,
          (int)cfg.min_code, (int)cfg.max_code);
    check("the waveform is far from flat", hi - lo > 100, "span=%d codes",
          (int)(hi - lo));

    const double samples     = (double)codes.size();
    const double want_spikes = (double)cfg.spike_pct * samples / 100.0;
    check("impulse noise hits ~spike_pct of the samples",
          std::fabs((double)m.impulses() - want_spikes) < 4.0 * std::sqrt(want_spikes),
          "%u impulses in %.0f samples (expected %.0f)", (unsigned)m.impulses(),
          samples, want_spikes);
    check("impulses pull the trace below the baseline too",
          lo < cfg.baseline_code - cfg.noise_code,
          "lowest code %d, baseline %d -/+%d", (int)lo, (int)cfg.baseline_code,
          (int)cfg.noise_code);

    const double clip = ((double)cfg.max_code - (double)cfg.baseline_code) *
                        (double)cfg.adc_per_dac;
    check("the DAC rail is reached (peaks saturate the source)", hi == cfg.max_code,
          "highest code %d = rail; the rail is %.0f counts above the baseline",
          (int)hi, clip);

    const int short_pulses = peaksShortOfAmplitude(cfg, codes, trains);
    check("every drawn peak reaches its amplitude (or the rail)", short_pulses == 0,
          "%d of %.0f pulses short", short_pulses, pulses);

    int low_h = 0, clipped_h = 0;
    for (size_t i = 0; i < st.peak_h.size(); i++) {
        if (st.peak_h[i] < 2.0 * (double)cfg.height_min) { low_h++; }
        if (st.peak_h[i] > clip) { clipped_h++; }
    }
    const double low_share  = 100.0 * (double)low_h / (double)st.peak_h.size();
    const double clip_share = 100.0 * (double)clipped_h / (double)st.peak_h.size();
    check("peaks span from the noise floor to past the rail",
          median(st.peak_h) < clip && low_share > 3.0 && clip_share > 3.0,
          "%.0f%% below %d counts (2x floor), %.0f%% past the rail",
          low_share, 2 * (int)cfg.height_min, clip_share);

    // ---- 11. the small-peak slice -------------------------------------------
    // Height and width are drawn independently, so on its own the train almost never
    // puts a small apex on a pulse wide enough for the rise test to be the bound that
    // decides - which is the corner that slice exists to present.  It has to be there
    // (at least one such peak per cycle on average, this signal is the detector's
    // small-peak test case) and every one of them has to sit *at or above* the apex
    // the rise test can accept at its own width: a "small peak" drawn below that
    // would be a test case for the wrong thing.
    {
        int frontier_n = 0, cycles_with = 0, below = 0;
        for (size_t cy = 0; cy < trains.size(); cy++) {
            int seen = 0;
            for (size_t i = 0; i < trains[cy].size(); i++) {
                const PulseSpec& p = trains[cy][i];
                if (isRect(p) || p.width < cfg.frontier_width_min ||
                    p.width > cfg.frontier_width_max) {
                    continue;
                }
                const int32_t floor_apex = sorter::frontierApex(cfg.frontier_trigger,
                                                                cfg.frontier_rise,
                                                                p.width);
                if (floor_apex <= 0) { continue; }
                const double lo = (double)floor_apex * (double)cfg.frontier_margin_min;
                const double hi = (double)floor_apex * (double)cfg.frontier_margin_max;
                if ((double)p.amplitude < lo - 1.0 ||
                    (double)p.amplitude > hi + 1.0) {
                    continue;   // an ordinary draw that happens to land in the band
                }
                seen++;
                if ((double)p.amplitude < (double)floor_apex) { below++; }
            }
            frontier_n += seen;
            if (seen > 0) { cycles_with++; }
        }
        check("the train presents small peaks on the detector's frontier",
              2 * frontier_n >= (int)trains.size(),
              "%d frontier peaks over %u cycles (%d of them hold one)",
              frontier_n, (unsigned)trains.size(), cycles_with);
        check("no frontier peak is drawn under the apex the rise test needs",
              below == 0, "%d of %d below it", below, frontier_n);
    }

    // ---- 10. an isolated pulse reaches baseline + its drawn amplitude -------
    // With the random extras switched off (no merged pairs, no impulses, no
    // glitches, no log-uniform heights, no needles, no baseline wander) the tallest
    // peak must land exactly on baseline + amplitude / adc_per_dac - that is what
    // makes a printed max= comparable with the drawn amplitude at all.
    SignalModel::Config plain = cfg;
    plain.merge_pct         = 0u;
    plain.spike_pct         = 0.0f;
    plain.glitch_pct        = 0u;
    plain.height_span_pct   = 0u;
    plain.width_span_pct    = 0u;
    plain.frontier_pct      = 0u;
    plain.baseline_wave_code = 0;
    SignalModel pm(plain);
    pm.begin();

    const PulseSpec* tallest = nullptr;
    for (uint32_t i = 0; i < pm.pulseCount(); i++) {
        const PulseSpec& p = pm.pulses()[i];
        if (tallest == nullptr || p.amplitude > tallest->amplitude) { tallest = &p; }
    }

    int32_t apex = 0;
    for (uint32_t i = 0; i < cycle; i++) {
        const int32_t code = pm.tick();
        if (tallest != nullptr && i >= tallest->start &&
            i < tallest->start + tallest->width && code > apex) {
            apex = code;
        }
    }

    if (tallest == nullptr) {
        check("an isolated peak was drawn to check the apex on", false, "no pulses");
    } else {
        const double expect = (double)plain.baseline_code +
                              (double)tallest->amplitude / (double)plain.adc_per_dac;
        check("an isolated peak's apex ~ baseline + amplitude",
              std::fabs((double)apex - expect) <= (double)plain.noise_code + 2.0,
              "apex=%d, expected %.0f (amp %d counts)", (int)apex, expect,
              (int)tallest->amplitude);
    }

    // ---- 12. the floor ladder -----------------------------------------------
    // `ladder` = 1 replaces the whole train with a deterministic staircase: one rung
    // per slot, ascending apexes, no impulses, no glitches, no random pulses.  The
    // property the mirror check above cannot state is whether the *gap* between two
    // rungs really is the baseline - so measure it: between the rungs the code has to
    // sit on the wandering DC level within the dither.  That is what makes a rung a
    // peak near the baseline instead of a shoulder on its neighbour, and it is why the
    // live plot's own scale can show the small ones at all (config.h's
    // SIGNALGEN_LADDER_* block).
    {
        SignalModel::Config lad = cfg;
        lad.ladder = 1u;
        SignalModel lm(lad);
        lm.begin();
        check("the ladder draws ladder_count rungs",
              lm.pulseCount() == lad.ladder_count, "%u rungs of %u asked for",
              (unsigned)lm.pulseCount(), (unsigned)lad.ladder_count);

        bool  lshape = true, lfit = true;
        const uint32_t slot = lad.cycle_samples / lad.ladder_count;
        for (uint32_t i = 0; i < lm.pulseCount(); i++) {
            const PulseSpec& p = lm.pulses()[i];
            if (p.rect != 0u || p.width != lad.ladder_width ||
                p.amplitude != lad.ladder_min + (int32_t)i * lad.ladder_step) {
                lshape = false;
            }
            if (p.start != slot * i + (slot - lad.ladder_width) / 2u ||
                p.start + p.width > lad.cycle_samples) {
                lfit = false;
            }
        }
        check("every rung is a raised cosine of the ladder's width, stepped by "
              "ladder_step", lshape && lm.pulseCount() > 0,
              "%u rungs of %u smp", (unsigned)lm.pulseCount(),
              (unsigned)lad.ladder_width);
        check("the rungs are centred in their slots and inside the cycle", lfit,
              "slot %u smp, width %u smp", (unsigned)slot,
              (unsigned)lad.ladder_width);

        int32_t worst_gap = 0;
        for (uint32_t i = 0; i < (uint32_t)cycle; i++) {
            const int32_t code = lm.tick();
            if (i + 1u >= cycle) { continue; }   // no next sample to look at
            bool in_gap = true;
            for (uint32_t k = 0; k < lm.pulseCount(); k++) {
                const PulseSpec& p = lm.pulses()[k];
                if (i >= p.start && i < p.start + p.width) { in_gap = false; }
            }
            if (!in_gap) { continue; }
            // The level the sample should be on: the wandering DC level, i.e. the model
            // without dither and without impulses (both of which the ladder keeps).
            const double want = (double)lad.baseline_code +
                                sorter::baselineWaveCode(lad.baseline_wave_code,
                                                         lad.baseline_wave_samples, i);
            const int32_t off = (int32_t)std::lround(std::fabs((double)code - want));
            if (off > worst_gap) { worst_gap = off; }
        }
        check("between two rungs the trace is back at the baseline",
              worst_gap <= lad.noise_code + 1,
              "worst |code - the wandering level| in a gap %d codes (dither +/-%d)",
              (int)worst_gap, (int)lad.noise_code);
        check("the ladder draws no impulse noise", lm.impulses() == 0u,
              "%u impulses", (unsigned)lm.impulses());
    }

    // ---- summary ------------------------------------------------------------
    printf("\ngaps %.0f..%.0f (mean %.0f)   peaks %u (h %.0f median / %.0f mean,"
           " w %.0f +/- %.0f)   pairs %u   glitches %u\n",
           *std::min_element(st.gaps.begin(), st.gaps.end()),
           *std::max_element(st.gaps.begin(), st.gaps.end()), gap.mean,
           (unsigned)st.peak_w.size(), median(st.peak_h), ph.mean, pw.mean, pw.sd,
           (unsigned)st.merged, (unsigned)st.glitch_w.size());

    if (g_failures != 0) {
        printf("\nTEST FAILED (%d)\n", g_failures);
        return 1;
    }
    printf("\nTEST PASSED\n");
    return 0;
}
