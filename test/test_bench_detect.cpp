// test/test_bench_detect.cpp
//
// End-to-end audit of the bench loopback (SORTER_SIGNAL_GEN = 1), host side:
//
//     SignalModel (the M4's DAC waveform, signal_model.h)
//         -> AD7606 counts
//         -> EventAggregator / EventDetector (the real headers, LINKED on ch8)
//         -> one bundle per event, compared with the model's own ground truth.
//
// Nothing here is a mirror: the waveform comes from the same SignalModel the M4
// runs, the detection comes from the same EventAggregator the M4 runs, and the
// tuning is copied from config.h, so every number printed below is a number the
// board prints over serial in bench mode.
//
// Ground truth
// ------------
//   * pulses() : the nominal train of each pattern cycle (start / width /
//                amplitude / rect), snapshotted once per cycle.  Grouped by
//                overlap exactly like signal_model.py::event_spans(), i.e. two
//                pulses the model drew on top of each other are ONE span - that
//                is the event the bench log is supposed to report (README: "two
//                peaks that overlap -> ONE VALID event with width ~1.5-2.0
//                nominal widths").
//   * render   : the noise-free rendered waveform, rebuilt from the pulses with
//                the same 256-entry raised-cosine LUT signal_model.h uses (and
//                verified against a noise-free twin model below), so a span can
//                be judged on what the trace really does: does it dip below the
//                detector's own off-threshold between two humps, or does it never
//                leave the noise band at all?
//
// The three symptoms this measures (the ones the bench log shows)
// --------------------------------------------------------------
//   MISSED  a span the detector never reported.
//   SPLIT   one span covered by two events: the wide event the model injected
//           came back cut in two, i.e. "the event is truncated".  The audit says
//           whether the trace really fell back into the noise band between the
//           two (justified - the detector had to close) or never did
//           (unjustified - the close was triggered by the dither, a bug).
//   FUSED   one event covering two spans: two close events reported as one,
//           "only one of them being identified".  Again split into justified
//           (the trace never returned to the noise band between them) and
//           unjustified.
//   GHOST   a VALID event over no raised-cosine pulse at all - a spike or a
//           rectangular glitch accepted as an event.  `swallowed` counts the
//           glitches that did not become their own event but were measured
//           INSIDE a real peak's window (that is how a spike ends up as the
//           reported max= of an otherwise healthy event).
//   WIDTH   the reported width against the span the model injected, and against
//           the span the *trace* really spends above the on-threshold.  CUT
//           counts the events that came back shorter than half their span.
//   TRUNC   [truncated] flags, split by channel: with ch0 and ch8 fed the same
//           waveform an ideal pair never truncates, so a non-zero count is the
//           window closing before the trace did.
//
// Build & run: test/run_tests.ps1 (g++/clang++), or by hand:
//     g++ -std=c++14 -O2 -Wall -Wextra -I .. test_bench_detect.cpp -o test_bench_detect
//     ./test_bench_detect --cycles 6 --detail
//
// Options:
//   --cycles N   pattern cycles to simulate (default 4; 1 cycle = 20 000 samples)
//   --seed HEX   waveform seed; default = SIGNALGEN_SEED in config.h
//   --detail     list the mismatching spans with their valley evidence
//   --peaks      print the full per-span inventory (every peak, not just the bad
//                ones) with the detector's answer on it - the table to walk through
//                when a particular peak is in dispute
//   --start MODE hold (default) = the detector's contract, the quiet period sees noise
//                only; live = the board's reality, the model's pulses start at sample 0
//                and therefore run through the quiet period too
//   --hw         feed the two modules slightly different signals (see HwPath)
//   --report     print the report but exit 0 (the checks stay visible)
//   --sweep      run the candidate tuning set and print one line each
//   --ladder     run the floor ladder instead of the random train (SIGNALGEN_LADDER_*
//                in config.h): isolated pulses of one width whose apexes step up from
//                below the detector's floor, printed rung by rung.  This is the run to
//                *see* how close to the baseline a peak can be and still be detected -
//                `--ladder --stream` draws it for test/live_plot.py, and its scoring is
//                its own (the rungs below the floor are the measurement, so the bench
//                contract "every injected peak is reported" does not apply)
//   --ladder-width N  width of every ladder rung in samples (default 600 = 60 ms).
//   --ladder-min N    apex of the bottom rung in counts (default 20)
//   --ladder-step N   counts between rungs (default 40)
//   --ladder-count N  rungs drawn (default 12: 20 .. 460 counts)
//                All four only move this run - SIGNALGEN_LADDER_* in config.h is what
//                ships.  The floor is a staircase in width as well, so a fine min/step
//                at one width, then `--ladder-width 200 / 1200`, walks the table.
//   --stream     print the board's own serial text instead of the report: the M7's
//                `STREAM on:` header, one `S,` line per point and printBundle()'s
//                event lines, board-exact, for test/live_plot.py to draw
//                (--stream-decim N sets the decimation, default 10 = the ballpark
//                of SORTER_STREAM_DECIM; --stream and --peaks/--sweep do not mix)

#include "../event_aggregator.h"
#include "../signal_model.h"

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

using sorter::SignalModel;

// ---------------------------------------------------------------------------
// The bench build, mirrored from config.h (SORTER_SIGNAL_GEN = 1)
// ---------------------------------------------------------------------------
// These mirror config.h exactly; test_signal_model.cpp already pins the
// SignalModel::Config defaults to the same file, so the waveform is not repeated
// here - only the bits the *harness* needs to know about.
const uint32_t kCycleSamples    = 20000u;             // SIGNALGEN_CYCLE_SAMPLES
const uint32_t kDtUs            = 100u;               // 1e6 / SORTER_SAMPLE_RATE_HZ
const uint32_t kActiveMask      = (1u << 0) | (1u << 8);  // SORTER_ACTIVE_CHANNEL_MASK
const uint32_t kInvertMask      = 0u;                 // SORTER_INVERT_CHANNEL_MASK
const uint8_t  kPrimaryCh       = 8u;                 // SORTER_PRIMARY_CHANNEL
const uint32_t kSeedDefault     = 0x2F6E2B1u;         // SIGNALGEN_SEED
const uint32_t kQuietSamples    = 3000u;              // DET_QUIET_SAMPLES
const uint32_t kEndDebounce     = 2u;                 // DET_END_DEBOUNCE
const uint32_t kMinRiseSamples  = 20u;                // DET_MIN_RISE_SAMPLES
const uint32_t kMinWidthSamples = 35u;                // DET_MIN_WIDTH_SAMPLES
// The bench's own resolution when it decides whether a rejection was justified: the two
// crossings of the trigger carry a sample or two of noise each, and the detector's
// duration estimate scales that uncertainty by as much as 2 (the envelope correction in
// EventDetector::envelopeDuration), so a pulse within this many samples of a bound is
// settled by the measurement rather than by the model - see rejectionJustified().
const uint32_t kWidthResolve    = 4u;                 // 0.4 ms at 10 kSPS
const uint32_t kDipTolerance    = 3u;                 // DET_DIP_TOLERANCE
const float    kAdcPerDac       = 2.64f;              // SIGNALGEN_ADC_PER_DAC (+/-10 V)
const int32_t  kNoiseCode       = 16;                 // SIGNALGEN_NOISE_CODE
const int32_t  kBaselineCode    = 2048;               // SIGNALGEN_BASELINE_CODE
const int32_t  kMinCode         = 250;                // SIGNALGEN_MIN_CODE
const int32_t  kMaxCode         = 3850;               // SIGNALGEN_MAX_CODE
// The slow wander of the DC level: 150 DAC codes (+/-396 counts at +/-10 V) over one
// 30 s sine at 10 kSPS.  It is part of the *signal*, not of the noise, so the truth
// the detector is judged against has to carry it too (see cleanCode below).
const int32_t  kBaselineWaveCode    = 150;            // SIGNALGEN_BASELINE_WAVE_CODE
const uint32_t kBaselineWaveSamples = 300000u;        // SIGNALGEN_BASELINE_WAVE_SAMPLES
// The floor ladder (SIGNALGEN_LADDER_*): the pattern `--ladder` runs instead of the
// random train - rungs of one width, one per cycle slot, apexes stepping up from below
// the detector's floor.  config.h's SIGNALGEN_LADDER_* block has the reasoning and the
// measured floor; these are its numbers, mirrored like every other k* above.
const uint32_t kLadderWidth = 600u;                   // SIGNALGEN_LADDER_WIDTH
const int32_t  kLadderMin   = 20;                     // SIGNALGEN_LADDER_MIN
const int32_t  kLadderStep  = 40;                     // SIGNALGEN_LADDER_STEP
const uint32_t kLadderCount = 12u;                    // SIGNALGEN_LADDER_COUNT

// One run's ladder: the shipped numbers above, overridable per run so the staircase can
// be *walked* instead of only drawn - a finer min/step hunts the floor to the count, and
// a different width moves to another row of config.h's floor table.  A null pointer
// where a LadderSpec is expected means "the random train".
struct LadderSpec {
    uint32_t width = kLadderWidth;
    int32_t  min   = kLadderMin;
    int32_t  step  = kLadderStep;
    uint32_t count = kLadderCount;
};

// EventAggregator::Config for the bench build (config.h's makeAggregatorConfig()).
EventAggregator::Config benchConfig() {
    EventAggregator::Config c;
    EventDetector::Config&  d = c.detector;

    d.baseline_alpha      = 0.001f;   // DET_BASELINE_ALPHA
    d.noise_alpha         = 0.001f;   // DET_NOISE_ALPHA
    d.k_on                = 5.0f;     // DET_K_ON
    d.k_off               = 2.0f;     // DET_K_OFF
    d.k_gate              = 3.0f;     // DET_K_GATE
    d.sigma_floor         = 1;        // DET_SIGMA_FLOOR
    d.quiet_samples       = kQuietSamples;      // DET_QUIET_SAMPLES
    d.quiet_seed_points   = 128u;     // DET_QUIET_SEED_POINTS
    d.gate_max            = 160u;     // DET_GATE_MAX
    d.min_rise_samples    = kMinRiseSamples;    // DET_MIN_RISE_SAMPLES (the rise test)
    d.dip_tolerance       = kDipTolerance;      // DET_DIP_TOLERANCE
    d.end_debounce        = kEndDebounce;       // DET_END_DEBOUNCE
    d.impulse_filter_taps = 7;        // DET_IMPULSE_FILTER_TAPS
    d.min_max_value       = 80;       // DET_MIN_MAX_VALUE
    d.min_width_samples   = kMinWidthSamples;   // DET_MIN_WIDTH_SAMPLES
    d.max_width_samples   = 20000u;   // DET_MAX_WIDTH_SAMPLES
    d.min_area            = 0ull;     // DET_MIN_AREA
    d.max_slope           = 100.0f;   // DET_MAX_SLOPE

    c.channel_mask = kActiveMask;
    c.mode         = METRIC_MODE_LINKED;
    c.primary      = kPrimaryCh;
    c.rail_level    = 32000;          // DET_RAIL_LEVEL
    c.rail_width    = 8u;             // DET_RAIL_WIDTH
    c.rail_cooldown = 1000u;          // DET_RAIL_COOLDOWN
    return c;
}

// --min-rise / --min-width override DET_MIN_RISE_SAMPLES / DET_MIN_WIDTH_SAMPLES for
// one run, so a candidate bound can be examined with --detail, --peaks or --stream
// without recompiling - and so any row of --sweep can be reproduced on its own.
// 0 keeps what benchConfig() (i.e. config.h) says.
void applyBoundOverrides(EventAggregator::Config& c, uint32_t min_rise,
                         uint32_t min_width) {
    if (min_rise  != 0u) { c.detector.min_rise_samples  = min_rise; }
    if (min_width != 0u) { c.detector.min_width_samples = min_width; }
}

// ---------------------------------------------------------------------------
// The two modules are not identical
// ---------------------------------------------------------------------------
// In the loopback A12 feeds AD7606 module 0 (ch0 is the wired input there) and
// A13 feeds module 1 (ch8): two converters, two front ends, one shared RANGE
// strap.  Their offsets and noise floors therefore differ a little, which is
// what makes the *same* waveform cross a threshold on one module before the
// other.  The numbers are illustrative (datasheet-scale), not measured: the
// point of the mode is to show how much mismatch it takes to move ch0's
// [truncated] flag or to split an event, not to predict a particular board.
struct HwPath {
    int32_t ch0_offset;  // counts, module 0's offset error
    int32_t ch8_offset;  // counts, module 1's offset error
    double  adc_sigma;   // counts, each module's own noise (independent per channel)
};

const HwPath kIdealHw    = {0, 0, 0.0};
const HwPath kMismatchHw = {8, -6, 3.0};

// ---------------------------------------------------------------------------
// Truth: pulses, spans, and the noise-free render
// ---------------------------------------------------------------------------

// One pulse of one cycle, straight out of SignalModel::pulses().
struct Pulse {
    uint32_t start;
    uint32_t width;
    int32_t  amp;
    bool     rect;
};

// A group of pulses that overlap in time: what the model calls one event.
struct Span {
    uint32_t start = 0;      // first sample of the first pulse
    uint32_t end   = 0;      // one past the last sample of the last pulse
    uint32_t apex  = 0;      // sample of the tallest pulse's nominal apex
    int      n_peaks = 0;    // raised-cosine pulses in the span
    int      n_rect  = 0;    // rectangular glitches in the span
    int32_t  peak_amp = 0;   // tallest raised-cosine amplitude (counts)
    uint32_t index = 0;      // span number, for the detail printout
};

// The 256-entry raised-cosine table of signal_model.h (shapeLut()): the model
// maps (i - start) * 256 / width onto it.  0 at both ends, 1 in the middle.
const float* shapeLut() {
    static float lut[256];
    static bool  ready = false;
    if (!ready) {
        for (int i = 0; i < 256; i++) {
            lut[i] = 0.5f * (1.0f - std::cos(2.0f * sorter::kPi * (float)i / 256.0f));
        }
        ready = true;
    }
    return lut;
}

// The DAC code of cycle sample `i` for a capture whose global sample index is
// `total`: the pulses, sitting on the wandering DC level, with no dither and no
// impulse.  This is signal_model.h::sampleCode() with those two terms left out, i.e.
// the waveform a noiseless board would hand to the detector - and it shares
// baselineWaveCode() with the model, so the drift cannot get out of sync between the
// two.  `i` picks the pulse window (the position inside the pattern cycle) while
// `total` is the wander's phase, which is why a capture that starts mid-cycle still
// reconstructs correctly.  main() verifies this against a noise-free twin model, so
// the reconstruction cannot drift.
int32_t cleanCode(const std::vector<Pulse>& train, uint32_t i, uint32_t total) {
    const float* shape = shapeLut();
    float        v     = (float)kBaselineCode +
                         sorter::baselineWaveCode(kBaselineWaveCode,
                                                  kBaselineWaveSamples, total);
    for (std::size_t p = 0; p < train.size(); p++) {
        const Pulse& q = train[p];
        if (i < q.start || i >= (q.start + q.width)) { continue; }
        const float a = (float)q.amp / kAdcPerDac;      // counts -> DAC codes
        if (q.rect) {
            v += a;
        } else {
            uint32_t u = ((i - q.start) * 256u) / q.width;
            if (u >= 256u) { u = 255u; }
            v += a * shape[u];
        }
    }
    if (v < (float)kMinCode) { v = (float)kMinCode; }
    if (v > (float)kMaxCode) { v = (float)kMaxCode; }
    return (int32_t)lroundf(v);
}

// Just the train's own contribution (DAC codes), added exactly the way cleanCode() adds
// it.  Subtracting this from the model's sample holds the *pulses* off and leaves
// everything else - the dither, the impulses and the slow wander - in place, which is
// what the quiet period is supposed to measure.  Holding the wander off as well would put
// a step in the trace where the hold ends, and a step of that size is outside the
// tracker's gate: the reference can only follow what is within k_gate * sigma of it.
int32_t pulseCode(const std::vector<Pulse>& train, uint32_t i) {
    const float* shape = shapeLut();
    float        v     = 0.0f;
    for (std::size_t p = 0; p < train.size(); p++) {
        const Pulse& q = train[p];
        if (i < q.start || i >= (q.start + q.width)) { continue; }
        const float a = (float)q.amp / kAdcPerDac;      // counts -> DAC codes
        if (q.rect) {
            v += a;
        } else {
            uint32_t u = ((i - q.start) * 256u) / q.width;
            if (u >= 256u) { u = 255u; }
            v += a * shape[u];
        }
    }
    return (int32_t)lroundf(v);
}

// Group a cycle's pulses into spans by overlap, like signal_model.py::event_spans:
// a pulse that starts inside the span the previous one built belongs to it.
std::vector<Span> groupSpans(const std::vector<Pulse>& train) {
    std::vector<Span> spans;
    for (std::size_t p = 0; p < train.size(); p++) {
        const Pulse& q = train[p];
        if (!spans.empty() && q.start < spans.back().end) {
            Span& s = spans.back();
            if ((q.start + q.width) > s.end) { s.end = q.start + q.width; }
            if (q.rect) {
                s.n_rect++;
            } else {
                s.n_peaks++;
                if (s.n_peaks == 1 || q.amp > s.peak_amp) {
                    s.peak_amp = q.amp;
                    s.apex     = q.start + q.width / 2u;
                }
            }
        } else {
            Span s;
            s.start = q.start;
            s.end   = q.start + q.width;
            s.index = (uint32_t)spans.size();
            s.apex  = q.start + q.width / 2u;
            if (q.rect) { s.n_rect = 1; } else { s.n_peaks = 1; s.peak_amp = q.amp; }
            spans.push_back(s);
        }
    }
    return spans;
}

// A pulse seen from outside: where its apex lands, which span it belongs to.
struct PulseGlobal {
    uint32_t start;    // global sample of the pulse's first sample
    uint32_t width;    // samples
    uint32_t apex;     // global sample of the pulse's nominal apex
    int32_t  amp;      // counts
    int      span;     // index into the span list
    bool     rect;     // glitch?
};

// Spans for every cycle (global sample indices) plus the flat pulse list used to
// decide which span an event covers.
void buildTruth(uint32_t cycles, const std::vector<std::vector<Pulse> >& trains,
                std::vector<Span>& spans, std::vector<PulseGlobal>& flat,
                uint32_t skip_before) {
    std::size_t span_cursor = 0;
    for (uint32_t c = 0; c < cycles; c++) {
        const uint32_t base = c * kCycleSamples;
        const std::vector<Span> sp = groupSpans(trains[c]);
        for (std::size_t s = 0; s < sp.size(); s++) {
            if (sp[s].start + base < skip_before) { continue; }   // held at the baseline
            Span g = sp[s];
            g.start += base;
            g.end   += base;
            g.apex  += base;
            g.index  = (uint32_t)spans.size();
            spans.push_back(g);
        }
        for (std::size_t p = 0; p < trains[c].size(); p++) {
            const Pulse& q = trains[c][p];
            const uint32_t gstart = q.start + base;
            if (gstart < skip_before) { continue; }   // part of a held span (see above)
            while (span_cursor < spans.size() && gstart >= spans[span_cursor].end) {
                span_cursor++;
            }
            PulseGlobal f;
            f.start = gstart;
            f.width = q.width;
            f.apex  = q.start + q.width / 2u + base;
            f.amp   = q.amp;
            f.rect  = q.rect;
            f.span  = (span_cursor < spans.size()) ? (int)span_cursor : -1;
            flat.push_back(f);
        }
    }
}

// ---------------------------------------------------------------------------
// One closed event, as the board would report it, plus the truth it covers
// ---------------------------------------------------------------------------
struct EventOut {
    uint32_t start = 0, end = 0;    // window, in global samples
    uint32_t width = 0;             // == bundle.width_samples: the primary's EXCURSION
    uint32_t judged = 0;            // == the primary channel's width_samples: the
                                    // event's DURATION - the width the verdict used
    bool     valid = false;
    uint8_t  reason = 0;
    int32_t  ch0_max = 0, ch8_max = 0;
    int32_t  ch0_base = 0, ch8_base = 0;    // the reference each channel froze
    uint8_t  ch0_flags = 0, ch8_flags = 0;
    uint8_t  ch0_reason = 0, ch8_reason = 0;
    int      span_first = -1;       // first span it covers a pulse of
    int      span_last  = -1;       // last one (span_first != span_last => FUSED)
    int      n_peaks = 0, n_rect = 0;
    int32_t  peak_amp = 0;          // tallest covered raised-cosine pulse (counts)
    bool     warmup = false;        // landed in the 3000-sample warm-up
};

struct Counters {
    int spans_peak = 0, spans_rect_only = 0, spans_warmup = 0;
    int events = 0, events_valid = 0;
    int missed = 0;
    int rejected_peak = 0;          // a real peak, INSIDE the documented envelope,
                                    // whose only events came back rejected
    int rejected_below_floor = 0;   // ... and the needles that are outside it: the
                                    // waveform injects events narrower than
                                    // DET_MIN_WIDTH_SAMPLES / faster than
                                    // DET_MAX_SLOPE on purpose, and the detector is
                                    // supposed to reject them - as long as it *says*
                                    // so (a rejected record, not silence)
    int below_floor_odd_reason = 0; // a below-floor span rejected for something other
                                    // than a width / rise / height bound
    int spike_max = 0;              // VALID event whose max= is far above its pulse
    int split_just = 0, split_unjust = 0;
    int fused_just = 0, fused_unjust = 0;
    int ghost_spike = 0, ghost_glitch = 0;
    int swallowed_rect = 0;         // VALID event with a glitch inside its window
    int trunc_ch0 = 0, trunc_ch8 = 0;
    // Small-peak coverage: a peak span drawn within a few times the on-threshold is
    // the detector's own small-peak test case (config.h's SIGNALGEN_FRONTIER_* block
    // draws those on purpose).  Counted here so every run *shows* how many of them it
    // had - the checks above are what judge them.
    int small_spans = 0, small_valid = 0;
    int width_cut = 0, width_over = 0;        // < 50 % / > 150 % of the span
    int width_n = 0;
    double width_err_abs_max = 0.0;           // |reported - nominal span| samples
    double width_err_sum = 0.0;
    int    dur_n = 0;                         // the same comparison for the number the
    double dur_err_sum = 0.0;                 // verdict actually used: the event's
    double dur_err_abs_max = 0.0;             // duration (EventDetector::envelopeDuration)
    int    dur_under_floor = 0;               // a span the model drew at or over the floor
                                              // whose judged duration came out under it
    int    render_n = 0;                      // events whose width could be checked
    double render_err_sum = 0.0;              // against the trace above k_on*sigma
    double render_err_abs_max = 0.0;
    int    spike_max_worst = 0;               // the worst such inflation (counts)
    int    by_reason[8] = {0};                // events per PeakRejectReason (0 = valid)
};

// Length of the longest run of samples below `thr` in s[a, b); `deepest` gets the
// lowest sample of that run.  This is the "could the detector have closed here?"
// question asked of the *noise-free* trace: the detector can only act on what the
// signal does, so a split is honest when the trace really did come back into the
// noise band for end_debounce samples, and a bug when it never left it.
uint32_t longestBelow(const std::vector<int32_t>& s, uint32_t a, uint32_t b,
                      int32_t thr, int32_t* deepest) {
    uint32_t best = 0, run = 0;
    int32_t  deep = 0, run_deep = 0;
    for (uint32_t i = a; i < b && i < s.size(); i++) {
        if (s[i] < thr) {
            if (run == 0) { run_deep = s[i]; }
            run++;
            if (s[i] < run_deep) { run_deep = s[i]; }
        } else {
            if (run > best) { best = run; deep = run_deep; }
            run = 0;
        }
    }
    if (run > best) { best = run; deep = run_deep; }
    if (deepest != nullptr) { *deepest = deep; }
    return best;
}

// The contiguous run of noise-free samples above `thr` that contains sample `ix`.
// False when `ix` is not above the threshold there (a spike/glitch event has no
// run to compare with).
bool renderRun(const std::vector<int32_t>& s, uint32_t ix, int32_t thr,
               uint32_t* a, uint32_t* b) {
    if (ix >= s.size() || s[ix] <= thr) { return false; }
    uint32_t lo = ix, hi = ix;
    while (lo > 0 && s[lo - 1] > thr) { lo--; }
    while (hi + 1 < s.size() && s[hi + 1] > thr) { hi++; }
    *a = lo;
    *b = hi + 1;
    return true;
}

struct RunResult {
    std::vector<EventOut> events;
    std::vector<Span>     spans;
    std::vector<PulseGlobal> flat;
    std::vector<int32_t>  clean;        // noise-free counts, one per sample
    std::vector<int32_t>  clean_base;   // ... and the level they sit on (the wander)
    std::vector<int32_t>  excursion;    // clean - clean_base: the trace as counts
                                        // above the *local* DC level
    std::vector<int32_t>  dirty8;       // ch8 as the detector saw it (counts)
    std::vector<int32_t>  ref_base;     // what a shadow EventDetector on ch8 tracked
    std::vector<float>    ref_sigma;    // ... as its baseline and sigma
    int32_t base_counts = 0;
    // Both thresholds are counts ABOVE the local baseline, not absolute levels: every
    // bound in the firmware is written as base + k*sigma, and the baseline is the
    // detector's own moving reference.  With the DC level drifting, an absolute
    // threshold would compare the drift itself against the band and call every sample
    // of the low half of the wander "above the on-threshold".
    int32_t thr_on = 0, thr_off = 0;
    double  sigma_est = 0.0;
    double  sigma_seen = 0.0;           // measured on the quiet part of dirty8
    int32_t noise_mean = 0;
    int     impulses = 0;

    // The DC level of the noise-free trace at `i` (the wander included, and held at
    // the nominal code wherever the quiet period held the pulses back).
    int32_t base_at(uint32_t i) const {
        return (i < clean_base.size()) ? clean_base[i] : base_counts;
    }

    // The noise-free trace, and the raw channel, as counts above that level.  These
    // are what the truth comparison works in (see thr_on / thr_off above).
    int32_t clean_above(uint32_t i) const {
        return (i < excursion.size()) ? excursion[i] : 0;
    }
    int32_t dirty_above(uint32_t i) const {
        return (i < dirty8.size() && i < clean_base.size())
                   ? (dirty8[i] - clean_base[i]) : 0;
    }
};

// ---------------------------------------------------------------------------
// --stream: the M7's own serial text, out of the firmware's own loop
// ---------------------------------------------------------------------------
// test/live_plot.py draws whatever the board prints: a `STREAM on:` metadata line,
// one `S,<index>,<gen>,<ch>:<count>` line per stream point and printBundle()'s two
// lines per closed event (sorter-firmware.ino).  --stream makes this harness print
// exactly that - the same EventBundle the M4 ships, over the same sample values the
// AD7606 would digitize - so a peak can be reviewed in the browser against the
// firmware's own verdict.  What it replaces is a *mirror*: live_plot.py --sim used
// to draw model_check.py's opinions, and that second implementation had quietly
// drifted away from the firmware it was supposed to be showing.
//
// The stream goes to stdout and nothing else does, so it composes:
//
//     test\test_bench_detect.exe --stream > bench.txt              (a capture)
//     test\test_bench_detect.exe --stream | py -3 test\live_plot.py --stdin
//
// The run's verdict goes to stderr instead, so a failing run still says so in the
// terminal without one line of it landing in the stream.
//
// The two event lines are pinned by the viewer's regular expressions (EVENT_HEAD /
// EVENT_CHANNEL in test/live_plot.py), the header and the `S,` lines by its _info()
// and _point().  Reword them here and the overlay stops being drawn - the price of
// having the firmware speak for itself instead of a model of it.
struct StreamOut {
    FILE*    fh     = nullptr;  // where the text goes (stdout for --stream)
    uint32_t decim  = 10u;      // SORTER_STREAM_DECIM: one printed point per N samples
    uint32_t points = 64u;      // SORTER_STREAM_POINTS: what one frame carries
};

// The channels the board streams, in the order it prints them: the same
// `chans=0,8` the M7's printStreamInfo() derives from SORTER_STREAM_CHANNELS.
const uint8_t kStreamChans[2] = {0u, (uint8_t)kPrimaryCh};
const uint8_t kStreamChanCount = 2u;

const char* reasonName(uint8_t reason);      // defined with the report, below
bool rejectionJustified(const RunResult& r, const EventAggregator::Config& cfg,
                        const Span& s, uint8_t reason);   // ... and so is this

// One channel of a closed event, word for word printBundle().
void printChannelLog(const ChannelMetrics& c, FILE* fh) {
    if ((c.flags & CH_FLAG_RAILED) != 0u) {
        std::fprintf(fh, "  ch%u  RAILED (input saturated)\n", (unsigned)c.channel);
        return;
    }
    if ((c.flags & CH_FLAG_PRESENT) == 0u) {
        std::fprintf(fh, "  ch%u  absent\n", (unsigned)c.channel);
        return;
    }
    std::fprintf(fh, "  ch%u  max=%d  area=%u  width=%uus/%usmp  base=%d  ",
                 (unsigned)c.channel, (int)c.max_value, (unsigned)c.area,
                 (unsigned)c.width_us, (unsigned)c.width_samples, (int)c.baseline);
    if (metricsValid(c.reason)) {
        std::fprintf(fh, "VALID");
    } else {
        std::fprintf(fh, "rejected %s", reasonName(c.reason));
    }
    if ((c.flags & CH_FLAG_TRUNCATED) != 0u) { std::fprintf(fh, "  [truncated]"); }
    std::fprintf(fh, "\n");
}

// One closed event: the head line (window + verdict), then one line per channel.
void printEventLog(const EventRecord& rec, FILE* fh) {
    const EventBundle& b = rec.bundle;
    std::fprintf(fh, "Event #%u  %sreason=%u %s  window=%u..%uus/%usmp  primary=ch%u"
                     "  mode=%s\n",
                 (unsigned)b.index, b.valid ? "VALID     " : "rejected  ",
                 (unsigned)b.reason, reasonName(b.reason), (unsigned)b.start_us,
                 (unsigned)b.end_us, (unsigned)b.width_samples, (unsigned)b.primary,
                 (b.mode == (uint8_t)METRIC_MODE_LINKED) ? "linked" : "per-channel");
    if (b.reason == REASON_RAILED) {
        std::fprintf(fh, "*** SIGNAL TOO BRIGHT: an input railed (saturated). Reduce "
                         "the source intensity; that channel's detector was reset. "
                         "***\n");
    }
    for (uint8_t i = 0; i < b.n_channels; i++) { printChannelLog(rec.ch[i], fh); }
}

// One stream point:  S,<sample index>,<gen code>,<ch>:<count>[,...]
void printStreamPoint(const int32_t* channels, uint32_t index, int32_t gen_code,
                      FILE* fh) {
    std::fprintf(fh, "S,%u,%d", (unsigned)index, (int)gen_code);
    for (uint8_t k = 0; k < kStreamChanCount; k++) {
        std::fprintf(fh, ",%u:%d", (unsigned)kStreamChans[k],
                     (int)channels[kStreamChans[k]]);
    }
    std::fprintf(fh, "\n");
}

// ---------------------------------------------------------------------------
// The run: waveform -> counts -> the real aggregator
// ---------------------------------------------------------------------------
// `frontend` models the analog path in front of the ADC: the DAC is written just
// before the conversion, and neither the DAC nor the AD7606's anti-alias filter
// settles inside one 100 us sample period, so a one-sample step comes back as a
// ramp a few samples long.  The README says as much ("the AD7606 analog front end
// ... smears an impulse over more samples").  A `frontend`-tap moving average per
// module is that smear: 1 = off (the model's own sample, what the Python tools
// simulate), 2-3 = what a real board does to the model's one-sample impulses.
//
// `stream` (--stream) makes the same loop print the board's serial text as it runs:
// same samples, same bundles, no second pass and no second implementation.
//
// `ladder` (not null) runs the floor ladder instead of the random train (config.h's
// SIGNALGEN_LADDER_* block): same loop, same detector, a waveform whose rungs step up
// from below the floor - what `--ladder` reports and `--ladder --stream` draws.
RunResult runAudit(uint32_t cycles, uint32_t seed, const HwPath& hw,
                   uint32_t frontend, const EventAggregator::Config& cfg,
                   bool hold_pulses = true, const StreamOut* stream = nullptr,
                   const LadderSpec* ladder = nullptr) {
    if (frontend < 1u) { frontend = 1u; }
    if (frontend > 8u) { frontend = 8u; }
    RunResult r;

    if (stream != nullptr) {
        // printStreamInfo(): the host learns the decimation (=> the sample rate), the
        // channel columns and the width floor the detector below judges with from here
        // instead of from config.h.  `minw` is that floor in samples at
        // SORTER_SAMPLE_RATE_HZ (DET_MIN_WIDTH_SAMPLES), read out of the very cfg this
        // run uses, so the floor band and the duration rule live_plot.py draws cannot
        // disagree with the verdict printed next to them (the floor is judged against the
        // event's duration: EventDetector::envelopeDuration).
        std::fprintf(stream->fh, "STREAM on: decim=%u chans=", (unsigned)stream->decim);
        for (uint8_t k = 0; k < kStreamChanCount; k++) {
            std::fprintf(stream->fh, "%s%u", (k > 0u) ? "," : "",
                         (unsigned)kStreamChans[k]);
        }
        std::fprintf(stream->fh, " wf=1 points=%u minw=%u\n", (unsigned)stream->points,
                     (unsigned)cfg.detector.min_width_samples);
    }

    // Truth reference levels.  The dither is white and uniform in +/-NOISE_CODE
    // DAC codes, so the noise floor the detector gates on is sigma = noise/sqrt(3)
    // in counts (README: "~25 counts at +/-10 V with the defaults").
    r.base_counts = (int32_t)lroundf((float)kBaselineCode * kAdcPerDac);
    r.sigma_est   = (double)kNoiseCode * (double)kAdcPerDac / std::sqrt(3.0);
    // Counts above the local baseline, never absolute levels: the baseline drifts, and
    // the bounds have to be read against the level the trace was on at that sample.
    r.thr_on      = (int32_t)lroundf(cfg.detector.k_on  * (float)r.sigma_est);
    r.thr_off     = (int32_t)lroundf(cfg.detector.k_off * (float)r.sigma_est);

    // ---- the waveform: the same model the M4 drives the DACs with -----------
    SignalModel gen;
    SignalModel::Config gcfg;          // defaults == config.h's SIGNALGEN_* block
    gcfg.seed        = seed;
    gcfg.adc_per_dac = kAdcPerDac;
    if (ladder != nullptr) {
        // The floor ladder (see the note above): one width, the rungs at the levels the
        // spec carries (config.h's SIGNALGEN_LADDER_* unless a --ladder-* flag moved
        // them), and everything the random train draws left off.
        gcfg.ladder       = 1u;
        gcfg.ladder_width = ladder->width;
        gcfg.ladder_min   = ladder->min;
        gcfg.ladder_step  = ladder->step;
        gcfg.ladder_count = ladder->count;
    }
    gen.configure(gcfg);
    gen.begin();

    const uint32_t total = cycles * kCycleSamples;
    std::vector<std::vector<Pulse> > trains(cycles);
    r.clean.reserve(total);
    r.dirty8.reserve(total);

    // The bench has two independent module paths; each is its own little ADC.
    const int32_t ch_of[2]  = {0, (int32_t)kPrimaryCh};
    const int32_t off_of[2] = {hw.ch0_offset, hw.ch8_offset};
    uint32_t      rng[2]    = {0x1BADB002u, 0x51ED5EEDu};
    double        spare[2]  = {0.0, 0.0};
    bool          have_spare[2] = {false, false};

    EventAggregator agg(cfg);
    // A second detector on the *same* ch8 samples as the one inside the aggregator:
    // its baseline()/sigma() are what the firmware's thresholds were actually taken
    // off, which is the number a disputed peak has to be judged against.  It sees the
    // same sequence of samples, so its state tracks the primary's.
    EventDetector ref_probe(cfg.detector);
    int32_t  channels[SORTER_MAX_CHANNELS] = {0};
    int      last_cycle = -1;

    // The detector's contract: nothing happens during its quiet period, so the level it
    // locks onto when detection arms is the *noise*.  The model draws pulses from
    // sample 0, so for the first cycle every pulse group that overlaps the quiet period
    // is not injected at all - for the group's whole span: the generator keeps ticking
    // (its phase has to stay put) and only the train is subtracted, so the code fed to the
    // converter is the model's own sample without it, dither, impulses and slow wander
    // included (see pulseCode below).  buildTruth() drops
    // the held groups from the truth, so the waveform and the comparison stay
    // consistent.
    uint32_t quiet_hold = 0;

    // The analog path in front of each module (see the note above).
    int32_t fe_ring[2][8];
    uint8_t fe_head[2] = {0u, 0u};
    for (int k = 0; k < 2; k++) {
        for (uint32_t t = 0; t < 8u; t++) {
            fe_ring[k][t] = (int32_t)lroundf((float)kBaselineCode * kAdcPerDac) + off_of[k];
        }
    }

    for (uint32_t i = 0; i < total; i++) {
        const int32_t gen_code = gen.tick();

        // Snapshot this cycle's train: tick() has just drawn it for the first
        // sample of the cycle (signal_model.h).
        const int cycle = (int)(i / kCycleSamples);
        if (cycle != last_cycle) {
            trains[cycle].clear();
            const sorter::PulseSpec* p = gen.pulses();
            for (uint32_t k = 0; k < gen.pulseCount(); k++) {
                Pulse q;
                q.start = p[k].start;
                q.width = p[k].width;
                q.amp   = p[k].amplitude;
                q.rect  = (p[k].rect != 0u);
                trains[cycle].push_back(q);
            }
            if (cycle == 0) {
                // Hold every pulse group that overlaps the quiet period (see above), for
                // the group's *whole* span: a group is one event to the detector, so
                // holding half of one would leave a shape nothing drew.
                quiet_hold = kQuietSamples;
                const std::vector<Span> sp = groupSpans(trains[0]);
                for (std::size_t s = 0; s < sp.size(); s++) {
                    if (sp[s].start < kQuietSamples && sp[s].end > quiet_hold) {
                        quiet_hold = sp[s].end;
                    }
                }
            }
            last_cycle = cycle;
        }

        // `--start live` feeds the model as the board does, pulses and all, so the
        // quiet period is the contaminated one the firmware really gets; the default
        // (`--start hold`) keeps the detector's contract and drops just the pulses - the
        // dither, the impulses and the slow wander the reference is meant to follow all
        // stay (see pulseCode above for why the wander has to stay).
        const int32_t code = (hold_pulses && i < quiet_hold)
                                 ? (gen_code - pulseCode(trains[cycle], i % kCycleSamples))
                                 : gen_code;

        // The AD7606 reading: the DAC code scaled to counts, plus whatever this
        // channel's own converter adds.  Both modules see the same waveform
        // (SignalGen::writeOutputs writes one code to both DACs).
        int32_t raw[2];
        for (int k = 0; k < 2; k++) {
            double v = std::floor((double)code * (double)kAdcPerDac + 0.5) +
                       (double)off_of[k];
            if (hw.adc_sigma > 0.0) {
                if (!have_spare[k]) {
                    // Box-Muller, so the two paths get independent noise.
                    rng[k] ^= rng[k] << 13; rng[k] ^= rng[k] >> 17; rng[k] ^= rng[k] << 5;
                    const double u1 = (double)(rng[k] & 0xFFFFFFu) / 16777216.0 + 1e-12;
                    rng[k] ^= rng[k] << 13; rng[k] ^= rng[k] >> 17; rng[k] ^= rng[k] << 5;
                    const double u2 = (double)(rng[k] & 0xFFFFFFu) / 16777216.0;
                    const double rad = std::sqrt(-2.0 * std::log(u1));
                    spare[k]      = rad * std::sin(6.283185307179586 * u2) * hw.adc_sigma;
                    have_spare[k] = true;
                    v += rad * std::cos(6.283185307179586 * u2) * hw.adc_sigma;
                } else {
                    have_spare[k] = false;
                    v += spare[k];
                }
            }
            raw[k] = (int32_t)v;
        }

        // ... and the analog path in front of the converter smears it (see above).
        for (int k = 0; k < 2; k++) {
            if (frontend <= 1u) {
                channels[ch_of[k]] = raw[k];
            } else {
                fe_ring[k][fe_head[k]] = raw[k];
                fe_head[k] = (uint8_t)((fe_head[k] + 1u) % (uint8_t)frontend);
                int64_t sum = 0;
                for (uint32_t t = 0; t < frontend; t++) { sum += (int64_t)fe_ring[k][t]; }
                channels[ch_of[k]] = (int32_t)lround((double)sum / (double)frontend);
            }
        }
        r.dirty8.push_back(raw[1]);   // the dither, before the front end

        applyInputPolarity(channels, kInvertMask);

        // The point the M7 would print for this sample: the code the DACs were
        // written with and the counts the AD7606 read back, decimated the same way.
        if (stream != nullptr && (i % stream->decim) == 0u) {
            printStreamPoint(channels, i, code, stream->fh);
        }

        PeakMetrics probe_ev;
        ref_probe.update(channels[kPrimaryCh], i * kDtUs, i, probe_ev);
        r.ref_base.push_back((int32_t)lroundf(ref_probe.baseline()));
        r.ref_sigma.push_back(ref_probe.sigma());

        EventRecord rec;
        if (agg.update(channels, i * kDtUs, i, rec)) {
            // The board prints the bundle the moment the window closes.
            if (stream != nullptr) { printEventLog(rec, stream->fh); }
            EventOut e;
            e.start  = (rec.bundle.width_samples <= i) ? (i - rec.bundle.width_samples) : 0u;
            e.end    = i;
            e.width  = rec.bundle.width_samples;
            e.valid  = (rec.bundle.valid != 0u);
            e.reason = rec.bundle.reason;
            e.warmup = (e.end <= kQuietSamples);
            for (uint8_t c = 0; c < rec.bundle.n_channels; c++) {
                const ChannelMetrics& cm = rec.ch[c];
                if (cm.channel == 0u) {
                    e.ch0_max = cm.max_value; e.ch0_flags = cm.flags; e.ch0_reason = cm.reason;
                    e.ch0_base = cm.baseline;
                } else if (cm.channel == kPrimaryCh) {
                    e.ch8_max = cm.max_value; e.ch8_flags = cm.flags; e.ch8_reason = cm.reason;
                    e.ch8_base = cm.baseline;
                }
                // The width the verdict compared against the bounds: the channel record's
                // own width, which is the event's *duration* (see EventOut::judged).
                if (cm.channel == rec.bundle.primary) { e.judged = cm.width_samples; }
            }
            r.events.push_back(e);
        }

        // The noise-free trace of the same sample, for the truth checks, and the level it
        // sits on (the baseline under it - the wander), which is what every threshold in
        // this file is compared against.  A held sample is that level with the train left
        // out: the pulses are held off, nothing else is (see the held-code computation
        // above), so the trace and the level are the same there.
        const int32_t base_code = (int32_t)lroundf((float)kBaselineCode +
                                   sorter::baselineWaveCode(kBaselineWaveCode,
                                                            kBaselineWaveSamples, i));
        const int32_t trace_code = (i < quiet_hold)
            ? base_code
            : cleanCode(trains[cycle], i % kCycleSamples, i);
        r.clean_base.push_back((int32_t)lroundf((float)base_code * kAdcPerDac));
        r.clean.push_back((int32_t)lroundf((float)trace_code * kAdcPerDac));
        r.excursion.push_back(r.clean.back() - r.clean_base.back());
    }
    r.impulses = (int)gen.impulses();

    buildTruth(cycles, trains, r.spans, r.flat, quiet_hold);

    // The noise floor the detector actually saw, over the samples where the trace
    // is idle (no pulse at all): the dither alone.  It is measured as counts above the
    // *local* baseline, so the drift cannot leak into it.  sanity-checks the truth
    // threshold used above (README: "~25 counts at +/-10 V with the defaults").
    {
        double      sum = 0.0, sum2 = 0.0;
        std::size_t n = 0;
        for (std::size_t i = 0; i < r.clean.size(); i++) {
            if (r.clean_above((uint32_t)i) == 0) {
                const double v = (double)r.dirty_above((uint32_t)i);
                sum  += v;
                sum2 += v * v;
                n++;
            }
        }
        if (n > 0) {
            const double mean = sum / (double)n;
            const double var  = sum2 / (double)n - mean * mean;
            r.sigma_seen = (var > 0.0) ? std::sqrt(var) : 0.0;
            r.noise_mean = (int32_t)lroundf(mean);
        }
    }

    // ---- which pulse (and which span) does each event cover? ---------------
    // A pulse is covered when its own nominal apex sits inside the window.  This
    // is what turns "25 events" into "MISSED / SPLIT / FUSED / GHOST".
    for (std::size_t ei = 0; ei < r.events.size(); ei++) {
        EventOut& e = r.events[ei];
        for (std::size_t fi = 0; fi < r.flat.size(); fi++) {
            const PulseGlobal& f = r.flat[fi];
            if (f.apex < e.start || f.apex > e.end) { continue; }
            if (f.span >= 0) {
                if (e.span_first < 0) { e.span_first = f.span; }
                e.span_last = f.span;
            }
            if (f.rect) {
                e.n_rect++;
            } else {
                e.n_peaks++;
                if (f.amp > e.peak_amp) { e.peak_amp = f.amp; }
            }
        }
    }
    return r;
}

// ---------------------------------------------------------------------------
// The report: classify every event and every span, then print the summary
// ---------------------------------------------------------------------------
std::vector<std::vector<int> > eventsPerSpan(const RunResult& r) {
    std::vector<std::vector<int> > out(r.spans.size());
    for (std::size_t ei = 0; ei < r.events.size(); ei++) {
        const EventOut& e = r.events[ei];
        for (std::size_t fi = 0; fi < r.flat.size(); fi++) {
            const PulseGlobal& f = r.flat[fi];
            if (f.apex >= e.start && f.apex <= e.end && f.span >= 0) {
                std::vector<int>& v = out[(std::size_t)f.span];
                if (v.empty() || v.back() != (int)ei) { v.push_back((int)ei); }
            }
        }
    }
    return out;
}

// One line for a span: what the model drew there.
void printSpan(const RunResult& r, const Span& s) {
    (void)r;
    std::printf("    span #%u [%u..%u] %u smp  %d peak(s) %d glitch(es)  apex %d counts at %u\n",
                (unsigned)s.index, (unsigned)s.start, (unsigned)s.end,
                (unsigned)(s.end - s.start), s.n_peaks, s.n_rect,
                (int)s.peak_amp, (unsigned)s.apex);
}

const char* reasonName(uint8_t reason) {
    static const char* const kNames[8] = {"OK", "MAX_TOO_SMALL", "WIDTH_TOO_NARROW",
                                          "WIDTH_TOO_WIDE", "AREA_TOO_SMALL",
                                          "SHORT_FOR_HEIGHT", "RAILED", "?"};
    return kNames[(reason < 8u) ? reason : 7u];
}

// The tallest sample of the *noise-free* trace in [a, b), as a height above the
// local baseline: the number the detector's max= should be reporting for that span.
int32_t cleanMaxAbove(const RunResult& r, uint32_t a, uint32_t b) {
    int32_t m = 0;
    for (uint32_t i = a; i < b && i < r.clean.size(); i++) {
        const int32_t h = r.clean_above(i);
        if (h > m) { m = h; }
    }
    return m;
}

// The samples the detector actually saw inside a *short* window, as counts above
// the local baseline: impulse noise, a glitch and a 2 ms step all look alike in the
// summary, and this is what tells them apart.  Long windows (a real peak) are
// skipped - their shape is not in question.
void printWindowSamples(const RunResult& r, uint32_t a, uint32_t b) {
    const uint32_t n = (b > a) ? (b - a) : 1u;
    if (n > 300u) { return; }
    const uint32_t step = (n > 48u) ? (n / 48u) : 1u;
    std::printf("              raw ch%d, counts above the local baseline (the "
                "wander removed):", (int)kPrimaryCh);
    for (uint32_t i = a; i < b && i < r.dirty8.size(); i += step) {
        std::printf(" %d", (int)r.dirty_above(i));
    }
    std::printf("\n");
}

// Impulses: samples where the raw reading is far from the noise-free trace (the
// dither alone is +/-42 counts, an impulse is up to +/-1056).  This is what shows
// whether an excursion started on noise or on the signal.
void printImpulses(const RunResult& r, uint32_t a, uint32_t b) {
    std::printf("              impulse samples in [%u..%u) (|raw - trace| > 300 "
                "counts):", (unsigned)a, (unsigned)b);
    int n = 0;
    for (uint32_t i = a; i < b && i < r.dirty8.size() && i < r.clean.size(); i++) {
        const int32_t d = r.dirty8[i] - r.clean[i];
        if (d > 300 || d < -300) {
            std::printf(" %u(%+d)", (unsigned)i, (int)d);
            n++;
        }
    }
    if (n == 0) { std::printf(" none"); }
    std::printf("\n");
}

// What the *detector's own* reference was over this window: everything in
// event_detector.h is measured against base + k_on/k_off * sigma, so this is the
// level a disputed excursion had to beat.
void printDetectorRef(const RunResult& r, const EventAggregator::Config& cfg, uint32_t a,
                      uint32_t b) {
    if (a >= r.ref_base.size()) { return; }
    if (b > r.ref_base.size()) { b = (uint32_t)r.ref_base.size(); }
    const uint32_t lo   = a;
    const uint32_t hi   = (b > a) ? (b - 1u) : a;
    const int32_t  base = r.ref_base[lo];
    const float    sig  = r.ref_sigma[lo];
    std::printf("              detector ref @%u: base %d sigma %.1f -> on +%d, off +%d "
                "counts (was base %d sigma %.1f at the end)\n",
                (unsigned)lo, (int)base, (double)sig,
                (int)std::lroundf((float)base + cfg.detector.k_on * sig) - (int)base,
                (int)std::lroundf((float)base + cfg.detector.k_off * sig) - (int)base,
                (int)r.ref_base[hi], (double)r.ref_sigma[hi]);
}

// Events with no pulse of any kind inside their window: either a false accept or a
// noise excursion that got a log line instead of being suppressed.
void printOrphans(const RunResult& r) {
    std::printf("\n  events covering no model pulse at all:\n");
    int orphan = 0;
    for (std::size_t ei = 0; ei < r.events.size(); ei++) {
        const EventOut& e = r.events[ei];
        if (e.n_peaks > 0 || e.n_rect > 0) { continue; }
        orphan++;
        std::printf("    e#%-3u [%u..%u)  w %u (%.1f ms)  %s %s  ch8 max %d, trace max "
                    "+%d\n",
                    (unsigned)ei, (unsigned)e.start, (unsigned)e.end, (unsigned)e.width,
                    (double)e.width * 0.1, e.valid ? "VALID  " : "rejected",
                    reasonName(e.reason), (int)e.ch8_max,
                    (int)cleanMaxAbove(r, e.start, e.end));
    }
    if (orphan == 0) { std::printf("    (none)\n"); }
    std::printf("\n");
}

// ---------------------------------------------------------------------------
// The peak inventory (--peaks): every span the model drew, with the detector's
// answer on it, in time order.  Nothing is filtered - a span with no event, a
// span with two, and an event with no span all show up - because the point is to
// walk the list and argue about one line at a time.
// ---------------------------------------------------------------------------
void printPeakInventory(const RunResult& r, const EventAggregator::Config& cfg) {
    const std::vector<std::vector<int> > span_ev = eventsPerSpan(r);

    // Tally for the walk-through: one bucket per way a span can be judged.
    enum Judge { J_OK = 0, J_NOISE_LOG, J_GHOST, J_MISSED, J_REJECTED, J_BELOW_FLOOR,
                 J_SPLIT, J_WIDTH, J_HEIGHT, J_COUNT };
    int judge[J_COUNT];
    for (int k = 0; k < J_COUNT; k++) { judge[k] = 0; }

    std::printf("\n---- peak inventory: one block per span the model drew ----\n");
    std::printf("  baseline %d counts, wandering +/-%d codes (~%.0f counts) over %.0f s:"
                " every bound below is counts above that level\n"
                "  on-threshold +%d, off-threshold +%d (%.1f sigma seen)\n"
                "  bounds: max >= %d counts, width %u..%u smp (= %.1f..%.0f ms), "
                "rise >= %u smp (= %.1f ms), slope <= %.0f counts/smp\n",
                (int)r.base_counts, (int)kBaselineWaveCode,
                (double)kBaselineWaveCode * kAdcPerDac,
                (double)kBaselineWaveSamples / 10000.0,
                (int)r.thr_on, (int)r.thr_off, r.sigma_seen,
                (int)cfg.detector.min_max_value,
                (unsigned)cfg.detector.min_width_samples,
                (unsigned)cfg.detector.max_width_samples,
                (double)cfg.detector.min_width_samples * 0.1,
                (double)cfg.detector.max_width_samples * 0.1,
                (unsigned)cfg.detector.min_rise_samples,
                (double)cfg.detector.min_rise_samples * 0.1,
                (double)cfg.detector.max_slope);

    // The reference the detector seeded at the end of its quiet period, and the range
    // it held over the run: every level in the firmware comes off these two numbers, so
    // a peak has to be judged against them and not against the model's noise.
    if (r.ref_sigma.size() > kQuietSamples) {
        float mn = r.ref_sigma[kQuietSamples], mx = mn;
        for (std::size_t i = kQuietSamples; i < r.ref_sigma.size(); i++) {
            if (r.ref_sigma[i] < mn) { mn = r.ref_sigma[i]; }
            if (r.ref_sigma[i] > mx) { mx = r.ref_sigma[i]; }
        }
        std::printf("  detector reference after the quiet period: base %d, sigma %.1f "
                    "-> on +%.0f, off +%.0f counts\n  ... and over the run: sigma %.1f "
                    ".. %.1f, base %d .. %d\n\n",
                    (int)r.ref_base[kQuietSamples], (double)r.ref_sigma[kQuietSamples],
                    (double)cfg.detector.k_on * (double)r.ref_sigma[kQuietSamples],
                    (double)cfg.detector.k_off * (double)r.ref_sigma[kQuietSamples],
                    (double)mn, (double)mx, (int)r.ref_base[kQuietSamples],
                    (int)r.ref_base.back());
    }

    for (std::size_t si = 0; si < r.spans.size(); si++) {
        const Span& s = r.spans[si];
        if (s.start < kQuietSamples + 200u) { continue; }   // warm-up: not judged
        const std::vector<int>& ev = span_ev[si];

        std::printf("\n  span #%u  [%u..%u)  %u smp = %.1f ms   apex %u\n",
                    (unsigned)s.index, (unsigned)s.start, (unsigned)s.end,
                    (unsigned)(s.end - s.start), (double)(s.end - s.start) * 0.1,
                    (unsigned)s.apex);
        for (std::size_t fi = 0; fi < r.flat.size(); fi++) {
            const PulseGlobal& f = r.flat[fi];
            if (f.span != (int)si) { continue; }
            std::printf("    drawn   %-9s start %u  w %u (%.1f ms)  amp %d counts\n",
                        f.rect ? "glitch" : "cos-peak", (unsigned)f.start,
                        (unsigned)f.width, (double)f.width * 0.1, (int)f.amp);
        }

        uint32_t      ra = 0, rb = 0;
        const int32_t trace_max = cleanMaxAbove(r, s.start, s.end);
        std::printf("    trace   max +%d counts above the local baseline; above the "
                    "on-threshold for ", (int)trace_max);
        if (renderRun(r.excursion, s.apex, r.thr_on, &ra, &rb)) {
            std::printf("%u smp (%.1f ms)\n", (unsigned)(rb - ra),
                        (double)(rb - ra) * 0.1);
        } else {
            std::printf("0 smp: the trace never reaches it\n");
        }
        bool any_valid = false;
        for (std::size_t k = 0; k < ev.size(); k++) {
            if (r.events[(std::size_t)ev[k]].valid) { any_valid = true; }
        }
        if (ev.empty()) {
            std::printf("    got     no event at all\n");
        } else {
            for (std::size_t k = 0; k < ev.size(); k++) {
                const EventOut& e = r.events[(std::size_t)ev[k]];
                std::printf("    got     e#%-3d [%u..%u)  w %u (%.1f ms)  %s %s\n",
                            ev[k], (unsigned)e.start, (unsigned)e.end,
                            (unsigned)e.width, (double)e.width * 0.1,
                            e.valid ? "VALID  " : "rejected", reasonName(e.reason));
                std::printf("              ch0 base %d max %d %s / ch%d base %d max %d "
                            "%s%s%s\n",
                            (int)e.ch0_base, (int)e.ch0_max, reasonName(e.ch0_reason),
                            (int)kPrimaryCh, (int)e.ch8_base, (int)e.ch8_max,
                            reasonName(e.ch8_reason),
                            (e.ch8_flags & CH_FLAG_TRUNCATED) ? " [truncated]" : "",
                            (e.ch8_flags & CH_FLAG_RAILED) ? " [railed]" : "");
                printWindowSamples(r, e.start, e.end);
                printImpulses(r, e.start - 40u, e.end + 10u);
                printDetectorRef(r, cfg, e.start, e.end);
            }
        }

        // The judgment, from the model's point of view: a span that holds a
        // raised-cosine pulse is an event the bench log has to carry, one event per
        // span (a merged pair is one event by construction - see this file's header);
        // a glitch-only span is not an event at all.
        const double span_smp = (double)(s.end - s.start);
        if (s.n_peaks == 0) {
            if (ev.empty()) {
                judge[J_OK]++;
                std::printf("    judge   OK: a %u-smp glitch was suppressed, as it must "
                            "be\n", (unsigned)(s.end - s.start));
            } else if (!any_valid) {
                judge[J_NOISE_LOG]++;
                std::printf("    judge   WRONG: a glitch-only span still produced a log "
                            "line (`rejected %s`)\n",
                            reasonName(r.events[(std::size_t)ev[0]].reason));
            } else {
                judge[J_GHOST]++;
                std::printf("    judge   WRONG: a glitch-only span became a VALID "
                            "event\n");
            }
        } else if (ev.empty()) {
            judge[J_MISSED]++;
            std::printf("    judge   WRONG: MISSED - a %d-count pulse of %.1f ms was "
                        "never reported\n", (int)s.peak_amp, span_smp * 0.1);
        } else if (!any_valid &&
                   rejectionJustified(r, cfg, s,
                                      r.events[(std::size_t)ev[0]].reason)) {
            judge[J_BELOW_FLOOR]++;
            std::printf("    judge   ok: outside the documented envelope - narrower than "
                        "%u smp (+%u for the measurement's own resolution), an edge under "
                        "2 ms or one steeper than %.0f counts/smp - and reported as "
                        "`rejected %s` (max %d, w %u smp)\n",
                        (unsigned)cfg.detector.min_width_samples,
                        (unsigned)kWidthResolve,
                        (double)cfg.detector.max_slope,
                        reasonName(r.events[(std::size_t)ev[0]].reason),
                        (int)r.events[(std::size_t)ev[0]].ch8_max,
                        (unsigned)r.events[(std::size_t)ev[0]].width);
        } else if (!any_valid) {
            judge[J_REJECTED]++;
            std::printf("    judge   WRONG: the peak came back `rejected %s` (max %d, "
                        "w %u smp)\n",
                        reasonName(r.events[(std::size_t)ev[0]].reason),
                        (int)r.events[(std::size_t)ev[0]].ch8_max,
                        (unsigned)r.events[(std::size_t)ev[0]].width);
        } else if (ev.size() > 1u) {
            judge[J_SPLIT]++;
            std::printf("    judge   WRONG: SPLIT - one span, %u events\n",
                        (unsigned)ev.size());
        } else {
            const EventOut& e  = r.events[(std::size_t)ev[0]];
            const double    dw = (double)e.width - span_smp;
            const int32_t   dh = e.ch8_max - trace_max;
            if (std::fabs(dw) > span_smp * 0.5) { judge[J_WIDTH]++; } else { judge[J_OK]++; }
            if (std::fabs((double)dh) > 0.25 * (double)trace_max) { judge[J_HEIGHT]++; }
            std::printf("    judge   %s: w %+.0f smp vs the span, max %+d counts vs the "
                        "trace\n",
                        (std::fabs(dw) > span_smp * 0.5) ? "WRONG width" : "OK", dw,
                        (int)dh);
        }
    }
    printOrphans(r);
    std::printf("  judged %d span(s): %d OK, %d MISSED, %d rejected, %d below the "
                "floor, %d split, %d glitch log line(s), %d ghost(s), %d wrong width, "
                "%d suspicious max\n\n",
                judge[J_OK] + judge[J_NOISE_LOG] + judge[J_GHOST] + judge[J_MISSED] +
                    judge[J_REJECTED] + judge[J_BELOW_FLOOR] + judge[J_SPLIT] +
                    judge[J_WIDTH],
                judge[J_OK], judge[J_MISSED], judge[J_REJECTED], judge[J_BELOW_FLOOR],
                judge[J_SPLIT], judge[J_NOISE_LOG], judge[J_GHOST], judge[J_WIDTH],
                judge[J_HEIGHT]);
}

// Events that cover no genuine pulse, or that swallow two spans, or that carry
// the [truncated] flag: the false accepts and the grouping mistakes.
void classifyEvents(const RunResult& r, Counters& c, bool detail,
                    const EventAggregator::Config& cfg) {
    for (std::size_t ei = 0; ei < r.events.size(); ei++) {
        const EventOut& e = r.events[ei];
        c.events++;
        if (e.valid) { c.events_valid++; }
        c.by_reason[(e.reason < 8u) ? e.reason : 7u]++;
        if ((e.ch0_flags & CH_FLAG_TRUNCATED) != 0u) { c.trunc_ch0++; }
        if ((e.ch8_flags & CH_FLAG_TRUNCATED) != 0u) { c.trunc_ch8++; }

        if (e.valid && e.n_peaks == 0 && !e.warmup) {
            if (e.n_rect > 0) { c.ghost_glitch++; } else { c.ghost_spike++; }
            if (detail) {
                std::printf("      FALSE ACCEPT: event #%u [%u..%u] w=%u ch8max=%d covers "
                            "span #%d (%d peaks, %d glitches)\n",
                            (unsigned)ei, (unsigned)e.start, (unsigned)e.end,
                            (unsigned)e.width, (int)e.ch8_max, e.span_first, e.n_peaks,
                            e.n_rect);
                if (e.span_first >= 0) {
                    printSpan(r, r.spans[(std::size_t)e.span_first]);
                }
            }
        }
        if (e.valid && e.n_rect > 0 && e.n_peaks > 0) { c.swallowed_rect++; }

        // A spike set the max=: compare the reported max with the highest sample the
        // *noise-free* trace reaches inside the same window.  The dither alone can
        // explain ~4 sigma; anything above that is a spike that survived the input
        // filter and became the number the operator reads as the peak's height (the
        // README's "a max= the window's own samples cannot carry").  Comparing with
        // the trace instead of the nominal amplitudes is what makes this honest for a
        // merged pair, where one hump legitimately rides on the other's tail.
        if (e.valid && e.n_peaks > 0 && e.start < r.clean.size()) {
            int32_t clean_max = 0;
            for (uint32_t i = e.start; i <= e.end && i < r.clean.size(); i++) {
                const int32_t h = r.clean_above(i);
                if (h > clean_max) { clean_max = h; }
            }
            const int32_t excess = e.ch8_max - clean_max;
            if (excess > 150) {          // ~5 sigma of the dither: a real spike
                c.spike_max++;
                if (excess > c.spike_max_worst) { c.spike_max_worst = excess; }
                if (detail) {
                    std::printf("  SPIKE MAX  event #%u [%u..%u] w=%u  max=%d but the "
                                "trace in that window only reaches %d (+%d)\n",
                                (unsigned)ei, (unsigned)e.start, (unsigned)e.end,
                                (unsigned)e.width, (int)e.ch8_max, (int)clean_max,
                                (int)excess);
                }
            }
        }

        // One event over two spans: two close events came back as one.
        if (e.span_first >= 0 && e.span_last > e.span_first) {
            bool avoidable = false;
            for (int si = e.span_first; si < e.span_last; si++) {
                int32_t        deep = 0;
                const uint32_t a = r.spans[(std::size_t)si].end;
                const uint32_t b = r.spans[(std::size_t)si + 1u].start;
                if (longestBelow(r.excursion, a, b, r.thr_off, &deep) >=
                    cfg.detector.end_debounce) {
                    avoidable = true;
                }
            }
            if (avoidable) { c.fused_unjust++; } else { c.fused_just++; }
            if (detail) {
                std::printf("  FUSED   event #%u [%u..%u] %u smp covers spans #%d..#%d: %s\n",
                            (unsigned)ei, (unsigned)e.start, (unsigned)e.end,
                            (unsigned)e.width, e.span_first, e.span_last,
                            avoidable ? "the trace DID dip back into the noise band"
                                      : "the trace never left the noise band");
            }
        }
    }
}

// Is this span outside the envelope the detector's own bounds document?
//
// This is the one judgement the bench makes from the *model's* geometry rather than
// from the detector's answer - and it has to be, or the exemption would be circular.
// Three documented bounds draw the envelope (config.h, the DET_* block):
//
//   DET_MIN_WIDTH_SAMPLES  the shortest event worth reporting.  A pulse so narrow that
//                          the trace never spends that many samples above the
//                          on-threshold is narrower than any event the detector
//                          promises to report.
//   DET_MIN_RISE_SAMPLES   the shortest leading edge worth reporting.  A pulse's apex
//                          sits at the middle of its excursion, so the trace's own
//                          crossing -> apex distance is the rise the model drew.  A
//                          *faint* narrow pulse crosses the on-threshold high up its
//                          flank: its edge can be under 2 ms even when its width is
//                          over 3.5 ms, and a step is not a peak.
//   DET_MAX_SLOPE          the fastest leading edge a real event may show.  A raised
//                          cosine reaches its apex in half its width, so
//                          amplitude / (width / 2) is the slope of the pulse the model
//                          drew - the number DET_MAX_SLOPE is written against.
//
// A span that trips any one of them is one the bench *expects* to come back rejected,
// and the rejected record is what keeps it visible to the operator.  Anything else
// coming back rejected is a defect.
// Which bound does this rejection belong to - and do the model's own numbers justify it?
//
// This is the judgement the bench makes about a span whose events all came back
// rejected.  It is made from the MODEL's geometry (the width the pulse was drawn at and
// the slope its flanks imply) and from the trace only where the detector's bound is
// itself a statement about the trace.  It is deliberately NOT made from the number the
// rejection was based on: the version that asked "does the trace spend the floor's worth
// of samples above the on-threshold?" exempted every WIDTH_TOO_NARROW rejection by
// construction, because that trace width IS the number the event was rejected from.  A
// 5.0 ms pulse the floor killed therefore came back as "one of the needles the waveform
// injects on purpose", and no check in this file could see it (see the witness in
// config.h's DET_MIN_WIDTH_SAMPLES note).
//
// The resolution above (kWidthResolve) is what keeps the width clause honest: a pulse
// within that many samples of the floor may go either way, a pulse clearly over it may
// not be rejected by it at all.
bool rejectionJustified(const RunResult& r, const EventAggregator::Config& cfg,
                        const Span& s, uint8_t reason) {
    // What the model drew over this span: the widest raised-cosine pulse is the event the
    // detector had to judge, and a raised cosine reaches its apex in half its width, so
    // amp / (width / 2) is the fastest edge those pulses show.
    uint32_t drawn   = 0;
    int32_t  tallest = 0;
    double   edge    = 0.0;
    for (std::size_t i = 0; i < r.flat.size(); i++) {
        const PulseGlobal& f = r.flat[i];
        if (f.span != (int)s.index) { continue; }
        if ((int32_t)f.amp > tallest) { tallest = (int32_t)f.amp; }
        if (f.rect) { continue; }
        if (f.width > drawn) { drawn = f.width; }
        const double rise = (double)f.width / 2.0;
        if (rise > 0.0 && (double)f.amp / rise > edge) { edge = (double)f.amp / rise; }
    }

    switch (reason) {
        case REASON_WIDTH_TOO_NARROW:
            // The floor is a statement about the pulse's *length*, and the model's own
            // width is that length.  A pulse within kWidthResolve of the bound may go
            // either way - that is the bound working at its own resolution - so a
            // rejection there is expected; a pulse clearly over the bound may not be.
            return drawn <= cfg.detector.min_width_samples + kWidthResolve;

        case REASON_SHORT_FOR_HEIGHT: {
            // The shape bounds, each judged where the detector judges it: the rise bound
            // reads the *trace*'s crossing-to-apex edge (a faint pulse crosses the
            // on-threshold high up its flank, so its measured edge can be under 2 ms
            // even when the pulse is wider than the floor - see DET_MIN_RISE_SAMPLES in
            // config.h), while DET_MAX_SLOPE is written against the edge the model drew.
            uint32_t ra = 0, rb = 0;
            if (renderRun(r.excursion, s.apex, r.thr_on, &ra, &rb) &&
                (s.apex - ra) < cfg.detector.min_rise_samples) {
                return true;
            }
            return edge > (double)cfg.detector.max_slope;
        }

        case REASON_MAX_TOO_SMALL:
            // The amplitude bound is written against counts above the local level, which
            // is exactly what the model's amplitudes are.
            return (double)tallest < (double)cfg.detector.min_max_value;

        default:
            return false;   // nothing in config.h's bounds explains this one
    }
}

// Spans with no event (missed) or with several (split), and the width of the
// event that did cover a span.
void classifySpans(const RunResult& r, Counters& c, bool detail,
                   const EventAggregator::Config& cfg) {
    const std::vector<std::vector<int> > span_ev = eventsPerSpan(r);

    for (std::size_t si = 0; si < r.spans.size(); si++) {
        const Span& s = r.spans[si];
        if (s.start < kQuietSamples + 200u) { c.spans_warmup++; continue; }
        if (s.n_peaks > 0) { c.spans_peak++; } else { c.spans_rect_only++; }

        const std::vector<int>& ev = span_ev[si];
        // Small-peak coverage (see Counters): a raised-cosine span whose tallest pulse
        // was drawn within 4x the on-threshold this run worked with - the corner
        // config.h's SIGNALGEN_FRONTIER_* block exists to present, and the counter the
        // report prints.
        if (s.n_peaks > 0 && s.peak_amp <= 4 * r.thr_on) {
            c.small_spans++;
            for (std::size_t k = 0; k < ev.size(); k++) {
                if (r.events[(std::size_t)ev[k]].valid) { c.small_valid++; break; }
            }
        }
        if (!ev.empty() && s.n_peaks > 0) {
            // A real peak whose events all came back rejected is not "detected" at
            // all as far as the operator is concerned: the log line for it says
            // `rejected SHORT_FOR_HEIGHT` (or another reason), so the peak is lost.
            bool any_valid = false;
            for (std::size_t k = 0; k < ev.size(); k++) {
                if (r.events[(std::size_t)ev[k]].valid) { any_valid = true; }
            }
            if (!any_valid) {
                // In the envelope, or outside it?  "Outside" means the model's own
                // numbers put this pulse under one of the bounds config.h documents:
                // narrower than DET_MIN_WIDTH_SAMPLES (allowing the measurement its own
                // resolution), its trace edge shorter than DET_MIN_RISE_SAMPLES, or its
                // drawn edge steeper than DET_MAX_SLOPE - see rejectionJustified().
                // Those the bench expects to be rejected *and reported*: the log line is
                // how the operator sees a 3 ms event at all.  A reason that is not one
                // of the width / rise / height bounds - or a rejection none of them
                // explains - means something else went wrong.
                const uint8_t reason = r.events[(std::size_t)ev[0]].reason;
                const bool    odd    = reason != REASON_MAX_TOO_SMALL &&
                                        reason != REASON_WIDTH_TOO_NARROW &&
                                        reason != REASON_SHORT_FOR_HEIGHT;
                if (rejectionJustified(r, cfg, s, reason)) {
                    c.rejected_below_floor++;
                    if (odd) { c.below_floor_odd_reason++; }
                    if (detail) {
                        printSpan(r, s);
                        std::printf("      BELOW THE FLOOR: every event came back "
                                    "`rejected %s` - outside the envelope config.h "
                                    "documents, and reported rather than dropped "
                                    "(event #%d)\n",
                                    reasonName(reason), ev[0]);
                    }
                } else {
                    c.rejected_peak++;
                    if (detail) {
                        printSpan(r, s);
                        std::printf("      REJECTED PEAK: every event over this span "
                                    "came back rejected (event #%d reason=%u)\n",
                                    ev[0], (unsigned)reason);
                    }
                }
            }
        }

        if (ev.empty() && s.n_peaks > 0) {
            c.missed++;
            if (detail) {
                printSpan(r, s);
                std::printf("      MISSED: no event covers any of its pulses\n");
            }
        } else if (ev.size() >= 2u) {
            // Split: did the trace really return to the noise band in between?
            bool avoidable = false;
            for (std::size_t k = 0; k + 1u < ev.size(); k++) {
                int32_t        deep = 0;
                const uint32_t a = r.events[(std::size_t)ev[k]].end;
                const uint32_t b = r.events[(std::size_t)ev[k + 1u]].start;
                if (longestBelow(r.excursion, a, b, r.thr_off, &deep) >=
                    cfg.detector.end_debounce) {
                    avoidable = true;
                }
            }
            if (avoidable) { c.split_just++; } else { c.split_unjust++; }
            if (detail) {
                printSpan(r, s);
                std::printf("      SPLIT into %u events: ", (unsigned)ev.size());
                for (std::size_t k = 0; k < ev.size(); k++) {
                    const EventOut& e = r.events[(std::size_t)ev[k]];
                    std::printf("#%d[%u..%u] w=%u %s  ", ev[k], (unsigned)e.start,
                                (unsigned)e.end, (unsigned)e.width,
                                e.valid ? "VALID" : "rejected");
                }
                int32_t        deep = 0;
                const uint32_t a = r.events[(std::size_t)ev[0]].end;
                const uint32_t b = r.events[(std::size_t)ev[1]].start;
                const uint32_t run = longestBelow(r.excursion, a, b, r.thr_off, &deep);
                std::printf("\n      valley %u smp, lowest %d counts above the local "
                            "baseline vs off-threshold +%d -> %s\n",
                            (unsigned)run, (int)deep, (int)r.thr_off,
                            avoidable ? "a real dip: the detector had to close"
                                      : "never left the band: the split is a bug");
            }
        } else if (s.n_peaks > 0) {
            const EventOut& e    = r.events[(std::size_t)ev[0]];
            const double    span = (double)(s.end - s.start);
            const double    err  = (double)e.width - span;
            c.width_n++;
            c.width_err_sum += err;
            if (std::fabs(err) > c.width_err_abs_max) { c.width_err_abs_max = std::fabs(err); }

            // ... and the same comparison for the number the VERDICT used: the event's
            // duration, which is the width the bounds (and the operator) are given.  This
            // is where the fix for the defect under DET_MIN_WIDTH_SAMPLES shows up - the
            // excursion above reads well under a faint pulse's length, the duration does
            // not - and where a pulse the model drew at or over the floor coming back
            // judged *under* it would show up as a regression.
            if (e.judged > 0u) {
                const double derr = (double)e.judged - span;
                c.dur_n++;
                c.dur_err_sum += derr;
                if (std::fabs(derr) > c.dur_err_abs_max) {
                    c.dur_err_abs_max = std::fabs(derr);
                }
                if (span >= (double)cfg.detector.min_width_samples &&
                    e.judged < cfg.detector.min_width_samples) {
                    c.dur_under_floor++;
                }
            }

            // ... and against the width the *trace* really spends above the
            // on-threshold.  That, not the pulse's nominal extent, is what the event
            // has to match: a raised cosine's flanks cross the threshold late, so a
            // *faint* wide pulse - an apex only just over k_on*sigma - legitimately
            // has only its middle above the threshold, and the span test alone would
            // call that "cut in half" when nothing is wrong.  A window that really did
            // close early is shorter than the *trace*, which is the symptom this
            // counts (the span stays the basis only where the trace never clears the
            // threshold at all, i.e. for events that are rejected anyway).
            uint32_t ra = 0, rb = 0;
            double   basis  = span;
            bool     traced = false;
            if (renderRun(r.excursion, s.apex, r.thr_on, &ra, &rb)) {
                const double rerr = (double)e.width - (double)(rb - ra);
                c.render_n++;
                c.render_err_sum += rerr;
                if (std::fabs(rerr) > c.render_err_abs_max) {
                    c.render_err_abs_max = std::fabs(rerr);
                }
                basis  = (double)(rb - ra);
                traced = true;
            }
            if ((double)e.width * 2.0 < basis)       { c.width_cut++; }
            if ((double)e.width * 2.0 > basis * 3.0) { c.width_over++; }
            if (detail && ((double)e.width * 2.0 < basis ||
                           (double)e.width * 3.0 > basis * 2.0)) {
                printSpan(r, s);
                std::printf("      WIDTH   event #%d [%u..%u] w=%u  (%s %.0f, %+.0f smp)  "
                            "%s\n",
                            ev[0], (unsigned)e.start, (unsigned)e.end, (unsigned)e.width,
                            traced ? "trace" : "span", basis, (double)e.width - basis,
                            e.valid ? "VALID" : "rejected");
            }
        }
    }
}

void printSummary(const char* label, const Counters& c, const RunResult& r,
                  const EventAggregator::Config& cfg) {
    std::printf("== %s ==\n", label);
    std::printf("  spans: %d peak, %d glitch-only, %d in warm-up (skipped)\n",
                c.spans_peak, c.spans_rect_only, c.spans_warmup);
    std::printf("  small peaks: %d span(s) drawn within 4x the on-threshold (the "
                "waveform's own small-peak cases), %d VALID\n",
                c.small_spans, c.small_valid);
    std::printf("  events: %d (%d VALID)   impulses injected: %d\n",
                c.events, c.events_valid, r.impulses);
    std::printf("  MISSED %d   SPLIT %d (justified %d)   FUSED %d (justified %d)\n",
                c.missed, c.split_just + c.split_unjust, c.split_just,
                c.fused_just + c.fused_unjust, c.fused_just);
    std::printf("  false accepts: spike-only %d, glitch-only %d   glitches measured "
                "inside a VALID event: %d\n",
                c.ghost_spike, c.ghost_glitch, c.swallowed_rect);
    std::printf("  a spike set the max= in %d VALID event(s) (worst +%d counts)\n",
                c.spike_max, c.spike_max_worst);
    std::printf("  real peaks reported as rejected: %d inside the documented envelope "
                "(width >= %u smp, edge <= %.0f counts/smp), %d outside it - the needles "
                "the waveform injects on purpose, each carrying its own log line; the "
                "width check gives the measurement its own %u-sample resolution\n",
                c.rejected_peak, (unsigned)cfg.detector.min_width_samples,
                (double)cfg.detector.max_slope, c.rejected_below_floor,
                (unsigned)kWidthResolve);
    std::printf("  [truncated] flags: ch0 %d, ch%d %d\n",
                c.trunc_ch0, (int)kPrimaryCh, c.trunc_ch8);
    if (c.width_n > 0) {
        std::printf("  width vs the span it was drawn as: %d events, mean %+.0f smp, "
                    "|max| %.0f smp\n",
                    c.width_n, c.width_err_sum / (double)c.width_n, c.width_err_abs_max);
    }
    if (c.dur_n > 0) {
        std::printf("  judged duration vs the span it was drawn as: %d events, mean %+.0f "
                    "smp, |max| %.0f smp; %d judged under the %u-sample floor on a span "
                    "drawn at or over it\n",
                    c.dur_n, c.dur_err_sum / (double)c.dur_n, c.dur_err_abs_max,
                    c.dur_under_floor, (unsigned)cfg.detector.min_width_samples);
    }
    if (c.render_n > 0) {
        std::printf("  width vs the trace above the on-threshold: %d events, mean %+.0f "
                    "smp, |max| %.0f smp, cut (<50%%) %d, far over (>150%%) %d%s\n",
                    c.render_n, c.render_err_sum / (double)c.render_n,
                    c.render_err_abs_max, c.width_cut, c.width_over,
                    (c.render_n == c.width_n)
                        ? ""
                        : "   (spans with no measurable trace fall back to the span)");
    }
    std::printf("  noise floor seen by the detector: mean %d counts, sigma %.1f "
                "(model %.1f)\n",
                (int)r.noise_mean, r.sigma_seen, r.sigma_est);
    std::printf("  verdicts: %d OK, %d MAX_TOO_SMALL, %d WIDTH_TOO_NARROW, "
                "%d WIDTH_TOO_WIDE, %d AREA_TOO_SMALL, %d SHORT_FOR_HEIGHT, "
                "%d RAILED\n\n",
                c.by_reason[0], c.by_reason[1], c.by_reason[2], c.by_reason[3],
                c.by_reason[4], c.by_reason[5], c.by_reason[6]);
}

// ---------------------------------------------------------------------------
// The checks
// ---------------------------------------------------------------------------
int g_failures = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::printf("FAIL: ");                                              \
            std::printf(__VA_ARGS__);                                           \
            std::printf("\n");                                                  \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

// The reconstruction cleanCode() uses is this file's ground truth, so prove it: a
// noise-free twin model (same seed and shape, dither and impulses off) must produce
// exactly the codes cleanCode() rebuilds from the pulse list.  The same seed draws
// the same train (SignalModel::buildTrain runs at the first sample of the cycle),
// so this is an exact integer comparison, not a tolerance.
bool verifyClean(uint32_t seed) {
    SignalModel twin;
    SignalModel::Config c;      // defaults == config.h's SIGNALGEN_* block
    c.seed        = seed;
    c.adc_per_dac = kAdcPerDac;
    c.noise_code  = 0;          // no dither
    c.spike_pct   = 0.0f;       // no impulse
    twin.configure(c);
    twin.begin();

    std::vector<Pulse> train;
    bool               bad = false;
    for (uint32_t i = 0; i < kCycleSamples; i++) {
        const int32_t code = twin.tick();
        if (i == 0) {
            const sorter::PulseSpec* p = twin.pulses();
            for (uint32_t k = 0; k < twin.pulseCount(); k++) {
                Pulse q;
                q.start = p[k].start;
                q.width = p[k].width;
                q.amp   = p[k].amplitude;
                q.rect  = (p[k].rect != 0u);
                train.push_back(q);
            }
        }
        if (cleanCode(train, i, i) != code) { bad = true; }
    }
    return !bad;
}

// ---------------------------------------------------------------------------
// The floor ladder (--ladder): what the detector made of each rung
// ---------------------------------------------------------------------------
// One line per rung, in the order the cycle draws them, with the answer the firmware
// itself gave: no event, rejected + reason, or VALID + the width and max= it reported.
// The lowest rung the detector took *is* the floor at this width - seen rather than
// argued, which is the thing the random train cannot show (config.h's
// SIGNALGEN_LADDER_* block).  Nothing here is a mirror: the waveform, the detector and
// the numbers are the same ones the board runs.
//
// The table is the LAST cycle of the run.  The ladder is identical in every cycle, and
// the default `--start hold` mutes the model's pulses inside the 3000-sample quiet
// period - which is the first 0.3 s of the *run*, so it hides the first rungs of cycle
// 0 only.  `--start live` shows those too, contaminated reference and all.
// CHECK's counterpart for a report that goes to a stream of the caller's choosing.
// `--ladder --stream` prints its table on *stderr*, because stdout there has to stay
// pure board text: test/live_plot.py logs every line it does not understand as noise,
// and stream_check.py fails on one.  The text is CHECK's, so a failure reads the same
// wherever it lands.
bool checkTo(FILE* fh, bool ok, const char* fmt, ...) {
    if (ok) { return true; }
    va_list ap;
    std::fprintf(fh, "FAIL: ");
    va_start(ap, fmt);
    std::vfprintf(fh, fmt, ap);
    va_end(ap);
    std::fprintf(fh, "\n");
    return false;
}

void printLadder(FILE* fh, const RunResult& r, const EventAggregator::Config& cfg,
                 uint32_t cycles, const LadderSpec& spec, int* failures) {
    const std::vector<std::vector<int> > span_ev = eventsPerSpan(r);
    const uint32_t first = (cycles > 1u) ? (cycles - 1u) * kCycleSamples : 0u;
    const int32_t  top_amp = spec.min + (int32_t)(spec.count - 1u) * spec.step;
    *failures = 0;

    std::fprintf(fh, "\n---- floor ladder: %u rungs of %u smp (%.1f ms), apexes %d..%d "
                     "counts by %d ----\n",
                 (unsigned)spec.count, (unsigned)spec.width, (double)spec.width * 0.1,
                 (int)spec.min, (int)top_amp, (int)spec.step);
    std::fprintf(fh, "  bounds in play: max >= %d counts, rise >= %u smp, width >= %u "
                     "smp, slope <= %.0f counts/smp\n",
                 (int)cfg.detector.min_max_value, (unsigned)cfg.detector.min_rise_samples,
                 (unsigned)cfg.detector.min_width_samples, (double)cfg.detector.max_slope);
    if (r.ref_sigma.size() > kQuietSamples) {
        // The reference the verdicts below are taken off - the *detector's* own, not the
        // model's estimate: a rung is judged against base + k_on * sigma.
        std::fprintf(fh, "  the detector's own reference after its quiet period: base %d, "
                         "sigma %.1f -> on +%.0f, off +%.0f counts\n",
                     (int)r.ref_base[kQuietSamples], (double)r.ref_sigma[kQuietSamples],
                     (double)cfg.detector.k_on * (double)r.ref_sigma[kQuietSamples],
                     (double)cfg.detector.k_off * (double)r.ref_sigma[kQuietSamples]);
    }
    std::fprintf(fh, "\n  rung   apex        the detector's answer\n");

    int rungs = 0, valid = 0, rejected = 0, silent = 0, split = 0, fused = 0;
    int lowest_valid = 0, lowest_amp = 0, top_valid = 0;
    for (std::size_t si = 0; si < r.spans.size(); si++) {
        const Span& s = r.spans[si];
        if (s.start < first || s.n_peaks < 1) { continue; }   // not this cycle's rungs
        const std::vector<int>& ev = span_ev[si];
        rungs++;
        const bool is_top = (rungs == (int)spec.count);
        std::fprintf(fh, "  #%-4d %5d counts  ", rungs, (int)s.peak_amp);
        if (ev.empty()) {
            silent++;
            std::fprintf(fh, "no event at all: the trace never crossed the "
                             "on-threshold\n");
            continue;
        }
        bool any_valid = false;
        for (std::size_t k = 0; k < ev.size(); k++) {
            if (r.events[(std::size_t)ev[k]].valid) { any_valid = true; }
        }
        const EventOut& e = r.events[(std::size_t)ev[0]];
        std::fprintf(fh, "e#%-3d w %u (dur %u) smp  %s %s  max=%d",
                     ev[0], (unsigned)e.width, (unsigned)e.judged,
                     any_valid ? "VALID    " : "rejected ", reasonName(e.reason),
                     (int)e.ch8_max);
        if (ev.size() > 1u) {
            split++;
            std::fprintf(fh, "  [%u events on one rung]", (unsigned)ev.size());
        }
        std::fprintf(fh, "\n");
        if (any_valid) {
            valid++;
            if (lowest_valid == 0) { lowest_valid = rungs; lowest_amp = s.peak_amp; }
            if (is_top) { top_valid = 1; }
        } else {
            rejected++;
        }
    }
    for (std::size_t ei = 0; ei < r.events.size(); ei++) {
        if (r.events[ei].n_peaks == 0) { fused++; }   // an event over no rung at all
    }

    std::fprintf(fh, "\n  %d rung(s) judged in cycle %u: %d VALID, %d rejected, %d never "
                     "reached at all\n", rungs, (unsigned)(cycles - 1u), valid, rejected,
                 silent);
    if (lowest_valid > 1) {
        std::fprintf(fh, "  the floor at %u smp (%.1f ms) is between %d and %d counts: "
                         "rung #%d (%d counts) is the lowest the detector took, and #%d "
                         "(%d counts) the tallest it left alone\n",
                     (unsigned)spec.width, (double)spec.width * 0.1,
                     spec.min + (int32_t)(lowest_valid - 2) * spec.step, lowest_amp,
                     lowest_valid, lowest_amp, lowest_valid - 1,
                     spec.min + (int32_t)(lowest_valid - 2) * spec.step);
    } else if (lowest_valid == 1) {
        std::fprintf(fh, "  the floor at %u smp (%.1f ms) is at or below the ladder's own "
                         "bottom rung (%d counts): every rung was taken, so walk further "
                         "down with --ladder-min / --ladder-step - the dither band is what "
                         "is being measured here, not the apex\n",
                     (unsigned)spec.width, (double)spec.width * 0.1, (int)spec.min);
    } else {
        std::fprintf(fh, "  the floor at %u smp (%.1f ms) is above the ladder's pinnacle "
                         "(%d counts): nothing was taken at any rung\n",
                     (unsigned)spec.width, (double)spec.width * 0.1, (int)top_amp);
    }
    std::fprintf(fh, "  nothing was invented where no rung was drawn: %d event(s) over no "
                     "rung at all\n\n", fused);

    if (!checkTo(fh, rungs == (int)spec.count,
                 "the ladder drew %d rung(s) where %u were asked for (width %u smp): a "
                 "rung wider than its own slot of the cycle cannot be drawn at all - see "
                 "SIGNALGEN_LADDER_* in config.h", rungs, (unsigned)spec.count,
                 (unsigned)spec.width)) {
        (*failures)++;
    }
    if (rungs == (int)spec.count) {
        // The ladder's claim - "an apex this far above the baseline, this wide, has
        // nowhere left to hide" - only holds where the shape bounds can let it through.
        // Narrower than the rise bound can ever accept (frontierApex() is 0 for
        // width <= 2*rise, i.e. the 4 ms corner of config.h's table) the ladder is a
        // legitimate probe that is *supposed* to come back empty, so it reports without
        // claiming; anywhere the envelope does reach the pinnacle, a miss is a failure.
        const int32_t need = sorter::frontierApex(
            (int32_t)lroundf((float)cfg.detector.k_on * (float)r.sigma_est),
            cfg.detector.min_rise_samples, spec.width);
        if (need > 0 && top_amp >= need) {
            if (!checkTo(fh, top_valid != 0,
                         "the ladder's pinnacle (%d counts at %u smp, %.1f ms) was not "
                         "reported: an apex the shape bounds accept, that far above the "
                         "baseline, has nowhere left to hide",
                         (int)top_amp, (unsigned)spec.width, (double)spec.width * 0.1)) {
                (*failures)++;
            }
        } else {
            std::fprintf(fh, "  (no claim on the pinnacle: %d counts at %u smp needs an "
                             "apex of %d counts to show a %u-sample edge at all - see "
                             "DET_MIN_RISE_SAMPLES, and config.h's floor table for this "
                             "corner)\n",
                         (int)top_amp, (unsigned)spec.width, (int)need,
                         (unsigned)cfg.detector.min_rise_samples);
        }
        if (!checkTo(fh, split + fused == 0,
                     "%d split rung(s) and %d event(s) covering no rung: the rungs are "
                     "not isolated, so the ladder is not measuring the floor it claims to",
                     split, fused)) {
            (*failures)++;
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    uint32_t cycles      = 4u;
    uint32_t seed        = kSeedDefault;
    uint32_t min_rise    = 0u;   // 0 = config.h (--min-rise overrides it)
    uint32_t min_width   = 0u;   // 0 = config.h (--min-width overrides it)
    // Analog path in front of the ADC.  1 = the generator's sample reaches the
    // converter untouched (what detector_audit.py and plot_signals.py model); 3 is
    // what a real board looks like - see the README's bench section: the DAC and the
    // AD7606's anti-alias filter do not settle inside one 100 us sample period, so a
    // 1-sample impulse comes back spread over 2-4 samples.  The default is the
    // realistic case, because that is where a 3-tap median stopped working.
    uint32_t frontend    = 3u;
    bool     detail      = false;
    bool     hw          = false;
    bool     report_only = false;
    bool     sweep       = false;
    bool     peaks       = false;
    bool     start_live  = false;   // feed the pulses through the quiet period too
    bool     stream_on   = false;   // print the board's own serial text (--stream)
    uint32_t stream_decim = 10u;
    bool     ladder_on   = false;   // --ladder: the floor ladder, not the random train
    LadderSpec lspec;               // ... at SIGNALGEN_LADDER_* unless a flag moved it

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            cycles = (uint32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--start") == 0 && i + 1 < argc) {
            // hold (default) = the detector's contract: the quiet period sees noise
            // only.  live = the board's reality: the model's pulses start at sample 0,
            // so the quiet period is contaminated by whatever is running then.
            const char* what = argv[++i];
            if (std::strcmp(what, "live") == 0) { start_live = true; }
            else if (std::strcmp(what, "hold") == 0) { start_live = false; }
            else { std::printf("--start takes hold or live\n"); return 2; }
        } else if (std::strcmp(argv[i], "--frontend") == 0 && i + 1 < argc) {
            frontend = (uint32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = (uint32_t)std::strtoul(argv[++i], nullptr, 0);
        } else if (std::strcmp(argv[i], "--min-rise") == 0 && i + 1 < argc) {
            min_rise = (uint32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--min-width") == 0 && i + 1 < argc) {
            min_width = (uint32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--detail") == 0) {
            detail = true;
        } else if (std::strcmp(argv[i], "--hw") == 0) {
            hw = true;
        } else if (std::strcmp(argv[i], "--report") == 0) {
            report_only = true;
        } else if (std::strcmp(argv[i], "--sweep") == 0) {
            sweep = true;
        } else if (std::strcmp(argv[i], "--peaks") == 0) {
            peaks = true;     // the per-span inventory: every peak, not just the bad ones
        } else if (std::strcmp(argv[i], "--stream") == 0) {
            stream_on = true;
        } else if (std::strcmp(argv[i], "--ladder") == 0) {
            ladder_on = true;
        } else if (std::strcmp(argv[i], "--ladder-width") == 0 && i + 1 < argc) {
            ladder_on    = true;
            lspec.width  = (uint32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--ladder-min") == 0 && i + 1 < argc) {
            ladder_on    = true;
            lspec.min    = (int32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--ladder-step") == 0 && i + 1 < argc) {
            ladder_on    = true;
            lspec.step   = (int32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--ladder-count") == 0 && i + 1 < argc) {
            ladder_on    = true;
            lspec.count  = (uint32_t)std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--stream-decim") == 0 && i + 1 < argc) {
            stream_decim = (uint32_t)std::atoi(argv[++i]);
            if (stream_decim < 1u) { stream_decim = 1u; }
        } else {
            std::printf("unknown option: %s\n", argv[i]);
            return 2;
        }
    }
    if (cycles < 2u) { cycles = 2u; }

    // --stream: the board's text on stdout, the run's verdict on stderr, nothing
    // else anywhere - so `--stream | live_plot.py --stdin` receives a stream and
    // only a stream.  This is the review path: what the viewer draws is the
    // firmware's own decision, not a mirror of it.
    if (stream_on) {
        std::setvbuf(stdout, nullptr, _IOLBF, 8192);   // a line at a time: a pipe
                                                      // would otherwise hold the
                                                      // first 4 kB (~1.5 s of stream)
        StreamOut so;
        so.fh    = stdout;
        so.decim = stream_decim;
        const EventAggregator::Config scfg  = benchConfig();
        const HwPath                  spath = hw ? kMismatchHw : kIdealHw;
        EventAggregator::Config       srun  = scfg;
        applyBoundOverrides(srun, min_rise, min_width);
        const RunResult streamed = runAudit(cycles, seed, spath, frontend, srun,
                                            !start_live, &so,
                                            ladder_on ? &lspec : nullptr);
        std::fflush(stdout);
        if (ladder_on) {
            // The stream carries the ladder, so the page draws the same staircase the
            // table on stderr judges: `--ladder --stream | live_plot.py --stdin` is the
            // picture, this is the measurement, and both come from one run.
            std::fprintf(stderr, "stream: floor ladder, %u cycle(s), %u sample(s) - the "
                                 "table follows on stderr, the stream on stdout\n",
                         cycles, cycles * kCycleSamples);
            int ladder_bad = 0;
            printLadder(stderr, streamed, srun, cycles, lspec, &ladder_bad);
            // 0 on purpose, exactly like the plain --stream path: stdout is the stream,
            // stderr carries the verdict, and the exit code is run_tests.ps1's business
            // (it does not run this combination).  A FAIL above is still printed.
            return 0;
        }
        Counters sc;
        classifyEvents(streamed, sc, false, srun);
        classifySpans(streamed, sc, false, srun);
        std::fprintf(stderr,
                     "stream: %u cycle(s), %u sample(s), %d event(s) (%d VALID), "
                     "%d missed, %d ghost, %d peak rejected, %d spike-set max=, "
                     "%d split, %d fused   [test/run_tests.ps1 owns the checks]\n",
                     cycles, cycles * kCycleSamples, sc.events, sc.events_valid,
                     sc.missed, sc.ghost_spike + sc.ghost_glitch, sc.rejected_peak,
                     sc.spike_max, sc.split_unjust, sc.fused_unjust);
        return 0;
    }

    CHECK(verifyClean(seed),
          "cleanCode() does not reproduce a noise-free twin model - the truth this "
          "harness compares against is wrong");

    // --ladder (without --stream): the same waveform, the same detector, and the table
    // the stream would have drawn - rung by rung.  Its scoring is its own, and lives in
    // printLadder: most of what the ladder draws is *supposed* to be undetectable (the
    // rungs below the floor ARE the measurement), so the bench contract above - "every
    // injected peak is reported" - does not apply to this pattern.
    if (ladder_on) {
        const HwPath path = hw ? kMismatchHw : kIdealHw;
        EventAggregator::Config lcfg = benchConfig();
        applyBoundOverrides(lcfg, min_rise, min_width);
        std::printf("bench loopback, floor ladder: %u cycles (%u samples), seed 0x%X, "
                    "%s, front end %u tap(s)\n",
                    cycles, cycles * kCycleSamples, seed,
                    hw ? "two modules with offset/noise" : "ideal pair", frontend);
        const RunResult lad = runAudit(cycles, seed, path, frontend, lcfg, !start_live,
                                      nullptr, &lspec);
        int ladder_bad = 0;
        printLadder(stdout, lad, lcfg, cycles, lspec, &ladder_bad);
        g_failures += ladder_bad;
        if (g_failures > 0 && report_only) {
            std::printf("%d check(s) failed (report mode: not fatal).\n", g_failures);
            return 0;
        }
        if (g_failures > 0) {
            std::printf("%d check(s) failed.\n", g_failures);
            return 1;
        }
        std::printf("Floor ladder: all checks passed.\n");
        return 0;
    }

    EventAggregator::Config cfg = benchConfig();
    applyBoundOverrides(cfg, min_rise, min_width);
    std::printf("bench loopback: %u cycles (%u samples), seed 0x%X, mask 0x%X, "
                "primary ch%d, %s, front end %u tap(s)\n",
                cycles, cycles * kCycleSamples, seed, (unsigned)kActiveMask,
                (int)kPrimaryCh, hw ? "two modules with offset/noise" : "ideal pair",
                frontend);
    if (min_rise != 0u || min_width != 0u) {
        std::printf("  bounds overridden (--min-rise/--min-width): rise >= %u smp, "
                    "width >= %u smp; config.h ships %u and %u\n",
                    (unsigned)cfg.detector.min_rise_samples,
                    (unsigned)cfg.detector.min_width_samples,
                    (unsigned)kMinRiseSamples, (unsigned)kMinWidthSamples);
    }
    std::printf("\n");

    const HwPath   path = hw ? kMismatchHw : kIdealHw;
    const RunResult all = runAudit(cycles, seed, path, frontend, cfg, !start_live);
    Counters       c;
    classifyEvents(all, c, detail, cfg);
    classifySpans(all, c, detail, cfg);
    printSummary(hw ? "bench loopback, two slightly different modules"
                    : "bench loopback, ideal pair", c, all, cfg);
    if (peaks) { printPeakInventory(all, cfg); }

    // The bench log's contract (README: "Bench test / Procedure"), asserted:
    //   * every injected pulse is reported - a merged pair as ONE event,
    //   * nothing is invented where the trace never rose (a glitch or an impulse),
    //   * the window neither closes early (the [truncated] flag) nor swallows the
    //     next event,
    //   * the width is the injected pulse's width, not a fraction of it.
    CHECK(c.missed == 0,
          "%d injected peak span(s) were never reported", c.missed);
    CHECK(c.small_spans >= ((int)cycles / 4) + 1,
          "the waveform presented only %d small-peak case(s) over %d cycles: the "
          "detector is not being judged anywhere near its own floor (see "
          "SIGNALGEN_FRONTIER_* in config.h)", c.small_spans, cycles);
    CHECK(c.split_unjust == 0,
          "%d span(s) split even though the trace never left the noise band: an "
          "event was cut in two by the dither", c.split_unjust);
    CHECK(c.fused_unjust == 0,
          "%d event(s) swallowed two spans although the trace dipped back into the "
          "noise band between them: two events reported as one", c.fused_unjust);
    CHECK(c.ghost_spike + c.ghost_glitch == 0,
          "%d false accept(s): VALID event(s) over no genuine pulse (spike-only %d, "
          "glitch-only %d)", c.ghost_spike + c.ghost_glitch, c.ghost_spike,
          c.ghost_glitch);
    CHECK(c.swallowed_rect == 0,
          "%d VALID event(s) measured a glitch inside their window: the spike sets "
          "the max= and stretches the width", c.swallowed_rect);
    CHECK(c.trunc_ch0 == 0 && c.trunc_ch8 == 0,
          "%d [truncated] flag(s) on a pair fed the same waveform (ch0 %d, ch%d %d): "
          "the window closed before the trace did",
          c.trunc_ch0 + c.trunc_ch8, c.trunc_ch0, (int)kPrimaryCh, c.trunc_ch8);
    CHECK(c.width_cut == 0,
          "%d event(s) came back less than half as wide as the trace above the "
          "on-threshold: the window closed before the signal did", c.width_cut);
    CHECK(c.rejected_peak == 0,
          "%d peak span(s) INSIDE the documented envelope (width >= %u smp, edge <= "
          "%.0f counts/smp) were reported as `rejected`: a real event the machine must "
          "report has been lost (a spike glued to the front of a peak, or a bound that "
          "has drifted out of step with the signal)",
          c.rejected_peak, (unsigned)cfg.detector.min_width_samples,
          (double)cfg.detector.max_slope);
    CHECK(c.below_floor_odd_reason == 0,
          "%d peak span(s) below the documented floor were rejected for something other "
          "than a width / rise / height bound",
          c.below_floor_odd_reason);
    CHECK(c.spike_max == 0,
          "a spike set the max= in %d VALID event(s) (worst +%d counts over the "
          "pulse the event covers)", c.spike_max, c.spike_max_worst);

    if (sweep) {
        std::printf("---- candidate tuning (cycles %u, seed 0x%X%s) ----\n", cycles, seed,
                    hw ? ", two modules" : "");
        // fe = how many samples the analog front end smears a step over.  rise /
        // width are the two bounds a *narrow* event runs into first: DET_MIN_RISE_SAMPLES
        // is the shape guard (a step is not a peak) and DET_MIN_WIDTH_SAMPLES is the
        // shortest event worth reporting, so they are what a 3 ms needle hits (see the
        // note in config.h's SIGNALGEN block).  The rows below therefore sweep them
        // against the front end: a shorter rise bound is only safe while the input
        // filter still removes the spikes that made the long one necessary.
        struct Cand { uint32_t fe; uint8_t taps; uint32_t deb; float koff;
                      uint32_t rise; uint32_t width; const char* name; };
        // Every row carries the shipped bounds (rise >= 20, width >= 35) unless its
        // name says otherwise: those are DET_MIN_RISE_SAMPLES / DET_MIN_WIDTH_SAMPLES,
        // so a "(shipped)" row is the configuration config.h actually ships.
        const Cand cands[18] = {
            {1u, 3u, 2u, 2.0f, 20u, 35u, "fe=1 taps=3 deb=2 koff=2.0 (old default)"},
            {2u, 3u, 2u, 2.0f, 20u, 35u, "fe=2 taps=3 deb=2 koff=2.0 (old default)"},
            {3u, 3u, 2u, 2.0f, 20u, 35u, "fe=3 taps=3 deb=2 koff=2.0 (old default)"},
            {4u, 3u, 2u, 2.0f, 20u, 35u, "fe=4 taps=3 deb=2 koff=2.0 (old default)"},
            {2u, 5u, 2u, 2.0f, 20u, 35u, "fe=2 taps=5 deb=2 koff=2.0"},
            {3u, 5u, 2u, 2.0f, 20u, 35u, "fe=3 taps=5 deb=2 koff=2.0"},
            {4u, 5u, 2u, 2.0f, 20u, 35u, "fe=4 taps=5 deb=2 koff=2.0"},
            {2u, 7u, 2u, 2.0f, 20u, 35u, "fe=2 taps=7 deb=2 koff=2.0 (shipped)"},
            {3u, 7u, 2u, 2.0f, 20u, 35u, "fe=3 taps=7 deb=2 koff=2.0 (shipped)"},
            {4u, 7u, 2u, 2.0f, 20u, 35u, "fe=4 taps=7 deb=2 koff=2.0 (shipped)"},
            {3u, 9u, 2u, 2.0f, 20u, 35u, "fe=3 taps=9 deb=2 koff=2.0"},
            {4u, 9u, 2u, 2.0f, 20u, 35u, "fe=4 taps=9 deb=2 koff=2.0"},
            // ... now the narrow-event bounds, at the realistic front end.
            {3u, 7u, 2u, 2.0f, 12u, 40u, "fe=3 taps=7 rise>=12 width>=40"},
            {3u, 7u, 2u, 2.0f, 10u, 30u, "fe=3 taps=7 rise>=10 width>=30"},
            {3u, 7u, 2u, 2.0f, 10u, 25u, "fe=3 taps=7 rise>=10 width>=25"},
            {3u, 7u, 2u, 2.0f,  8u, 25u, "fe=3 taps=7 rise>=8  width>=25"},
            {3u, 7u, 2u, 2.0f,  8u, 20u, "fe=3 taps=7 rise>=8  width>=20"},
            {3u, 7u, 2u, 2.0f,  6u, 20u, "fe=3 taps=7 rise>=6  width>=20"},
        };
        for (int k = 0; k < 18; k++) {
            EventAggregator::Config s = cfg;
            s.detector.impulse_filter_taps = cands[k].taps;
            s.detector.end_debounce        = cands[k].deb;
            s.detector.k_off               = cands[k].koff;
            s.detector.min_rise_samples    = cands[k].rise;
            s.detector.min_width_samples   = cands[k].width;
            const RunResult r = runAudit(cycles, seed, path, cands[k].fe, s);
            Counters        cs;
            classifyEvents(r, cs, false, s);
            classifySpans(r, cs, false, s);
            std::printf("  %-40s ev %3d/%3dVAL miss %2d  rejPeak %2d below %2d  "
                        "spikeMax %2d (worst +%d)  ghost %2d  trunc %2d  cut %2d  "
                        "splitU %2d/%2d  fuseU %2d\n",
                        cands[k].name, cs.events, cs.events_valid, cs.missed,
                        cs.rejected_peak, cs.rejected_below_floor, cs.spike_max,
                        cs.spike_max_worst, cs.ghost_spike + cs.ghost_glitch,
                        cs.trunc_ch0 + cs.trunc_ch8, cs.width_cut, cs.split_unjust,
                        cs.split_just, cs.fused_unjust);
        }
    }

    if (g_failures > 0 && report_only) {
        std::printf("%d check(s) failed (report mode: not fatal).\n", g_failures);
        return 0;
    }
    if (g_failures > 0) {
        std::printf("%d check(s) failed.\n", g_failures);
        return 1;
    }
    std::printf("Bench loopback: all checks passed.\n");
    return 0;
}
