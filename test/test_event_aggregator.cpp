// test/test_event_aggregator.cpp
//
// Host-side unit test for EventAggregator (no Arduino required).
// Build & run it with test/run_tests.ps1 (needs g++ or clang++).
//
// Scenarios (mirrors test/model_check_aggregator.py):
//   1. LINKED      : ch8 is the primary; ch0 sees the same three peaks at half
//                    height, ch1 sees nothing, ch2 is not in the channel mask.
//                    One event -> one record per MASKED channel, every width
//                    equals ch8's, ch0 is smaller than ch8, ch1 is absent and
//                    there is no ch2 record at all.
//   2. LINKED clip : ch0's peak arrives after ch8's window has closed, so its
//                    height is clipped by the window and its record is flagged
//                    TRUNCATED while still carrying the window's width.
//   3. PER_CHANNEL : ch0/ch1/ch8/ch9 fire with different widths; each channel
//                    reports its own width/height/area and the event is reported
//                    once the last channel of the group has closed.

#include "../event_aggregator.h"
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

const double NOISE = 5.0;      // sigma of the baseline noise on every channel
const double BASE  = 2000.0;

struct PeakSpec { int center; double width; double amp; };

// One channel of the synthetic signal: a noisy baseline plus Gaussian peaks.
struct ChannelSpec {
    uint8_t         ch;
    double          baseline;
    const PeakSpec* peaks;
    int             n_peaks;
};

int32_t chanValue(int i, const ChannelSpec& cs) {
    double x = cs.baseline + gauss() * NOISE;
    for (int p = 0; p < cs.n_peaks; p++) {
        const double d = (double)(i - cs.peaks[p].center);
        x += cs.peaks[p].amp *
             std::exp(-(d * d) / (2.0 * cs.peaks[p].width * cs.peaks[p].width));
    }
    return (int32_t)std::lround(x);
}

void makeSample(int i, const ChannelSpec* specs, int n_specs, int32_t* out) {
    for (int c = 0; c < SORTER_MAX_CHANNELS; c++) { out[c] = 0; }
    for (int s = 0; s < n_specs; s++) { out[specs[s].ch] = chanValue(i, specs[s]); }
}

EventDetector::Config detectorConfig() {
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

EventAggregator::Config aggConfig(MetricMode mode, uint32_t mask, uint8_t primary) {
    EventAggregator::Config c;
    c.detector     = detectorConfig();
    c.channel_mask = mask;
    c.mode         = mode;
    c.primary      = primary;
    return c;
}

// Drives the aggregator over `n` samples and keeps every completed event.
static const int MAX_EVENTS = 6;
struct EventLog {
    EventRecord rec[MAX_EVENTS];
    int         count;
};

void runAggregator(EventAggregator& agg, const ChannelSpec* specs, int n_specs,
                   int n, uint32_t dt_us, EventLog& log) {
    int32_t values[SORTER_MAX_CHANNELS];
    log.count = 0;
    for (int i = 0; i < n; i++) {
        makeSample(i, specs, n_specs, values);
        EventRecord rec;
        if (agg.update(values, (uint32_t)i * dt_us, (uint32_t)i, rec)) {
            if (log.count < MAX_EVENTS) { log.rec[log.count++] = rec; }
        }
    }
}

const ChannelMetrics* findChannel(const EventRecord& r, uint8_t ch) {
    for (uint8_t i = 0; i < r.bundle.n_channels; i++) {
        if (r.ch[i].channel == ch) { return &r.ch[i]; }
    }
    return nullptr;
}

bool near(double x, double lo, double hi) { return x >= lo && x <= hi; }

} // namespace

static int  g_failures = 0;
static char g_msg[256];

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::snprintf(g_msg, sizeof(g_msg), __VA_ARGS__);                   \
            std::printf("FAIL: %s\n", g_msg);                                   \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

// ---------------------------------------------------------------------------
// 1. LINKED: the primary channel times the event for everyone
// ---------------------------------------------------------------------------
static void testLinked() {
    const PeakSpec peaks8[] = { {6000, 200, 800.0}, {13000, 200, 800.0}, {20000, 200, 800.0} };
    const PeakSpec peaks0[] = { {6000, 200, 400.0}, {13000, 200, 400.0}, {20000, 200, 400.0} };
    const ChannelSpec specs[] = {
        {0, BASE, peaks0, 3},        // same peaks, half the height
        {1, BASE, nullptr, 0},       // pure noise: reported as absent
        {8, BASE, peaks8, 3},        // the primary: it times the event
        {2, 5000.0, nullptr, 0},     // not in the channel mask: must not appear
    };

    EventAggregator agg(aggConfig(METRIC_MODE_LINKED, 0x0103u, 8));
    CHECK(agg.primaryChannel() == 8, "LINKED: primary resolved to %u, expected 8",
          (unsigned)agg.primaryChannel());

    EventLog log;
    runAggregator(agg, specs, 4, 26000, 10, log);
    CHECK(log.count == 3, "LINKED: %d events, expected 3", log.count);
    if (log.count != 3) { return; }

    for (int e = 0; e < log.count; e++) {
        const EventRecord& r = log.rec[e];
        const EventBundle& b = r.bundle;
        CHECK(b.index == (uint32_t)e, "LINKED #%d: bundle index %u", e, (unsigned)b.index);
        CHECK(b.n_channels == 3, "LINKED #%d: %u records, expected 3", e,
              (unsigned)b.n_channels);
        CHECK(b.mode == (uint8_t)METRIC_MODE_LINKED, "LINKED #%d: mode %u", e, (unsigned)b.mode);
        CHECK(b.primary == 8, "LINKED #%d: primary %u, expected 8", e, (unsigned)b.primary);
        CHECK(b.valid == 1 && b.reason == REASON_OK,
              "LINKED #%d: valid=%u reason=%u", e, (unsigned)b.valid, (unsigned)b.reason);
        if (b.n_channels != 3) { continue; }

        CHECK(r.ch[0].channel == 0 && r.ch[1].channel == 1 && r.ch[2].channel == 8,
              "LINKED #%d: channels %u/%u/%u, expected 0/1/8", e,
              (unsigned)r.ch[0].channel, (unsigned)r.ch[1].channel, (unsigned)r.ch[2].channel);

        const ChannelMetrics* c0 = findChannel(r, 0);
        const ChannelMetrics* c1 = findChannel(r, 1);
        const ChannelMetrics* c8 = findChannel(r, 8);
        CHECK(c0 && c1 && c8, "LINKED #%d: a channel record is missing", e);
        if (!c0 || !c1 || !c8) { continue; }

        // One event, one width: every channel inherits the primary's width.
        CHECK(c0->width_samples == b.width_samples, "LINKED #%d ch0: width %u != window %u",
              e, (unsigned)c0->width_samples, (unsigned)b.width_samples);
        CHECK(c8->width_samples == b.width_samples, "LINKED #%d ch8: width %u != window %u",
              e, (unsigned)c8->width_samples, (unsigned)b.width_samples);
        CHECK(c1->width_samples == b.width_samples, "LINKED #%d ch1: width %u != window %u",
              e, (unsigned)c1->width_samples, (unsigned)b.width_samples);
        CHECK(b.width_us == c8->width_us, "LINKED #%d: window %uus != ch8 %uus",
              e, (unsigned)b.width_us, (unsigned)c8->width_us);
        CHECK(b.end_us == b.start_us + b.width_us, "LINKED #%d: window %u..%u",
              e, (unsigned)b.start_us, (unsigned)b.end_us);

        CHECK(c8->flags == CH_FLAG_PRESENT, "LINKED #%d ch8: flags 0x%02x",
              e, (unsigned)c8->flags);
        CHECK(c8->reason == REASON_OK, "LINKED #%d ch8: reason %u", e, (unsigned)c8->reason);
        CHECK(near(c8->max_value, 650, 950), "LINKED #%d ch8: height %d, expected ~800",
              e, (int)c8->max_value);

        CHECK(c0->flags == CH_FLAG_PRESENT, "LINKED #%d ch0: flags 0x%02x", e,
              (unsigned)c0->flags);
        CHECK(near(c0->max_value, 250, 550), "LINKED #%d ch0: height %d, expected ~400",
              e, (int)c0->max_value);
        CHECK(c0->max_value < c8->max_value, "LINKED #%d: ch0 (%d) not smaller than ch8 (%d)",
              e, (int)c0->max_value, (int)c8->max_value);
        CHECK(c0->area > 0 && (double)c8->area > 1.5 * (double)c0->area,
              "LINKED #%d: ch0 area %llu not clearly below ch8 area %llu",
              e, (unsigned long long)c0->area, (unsigned long long)c8->area);
        CHECK(near(c0->baseline, 1950, 2050), "LINKED #%d ch0: baseline %d", e,
              (int)c0->baseline);

        CHECK(c1->flags == 0, "LINKED #%d ch1: flags 0x%02x, expected absent", e,
              (unsigned)c1->flags);
        CHECK(c1->reason == REASON_MAX_TOO_SMALL, "LINKED #%d ch1: reason %u", e,
              (unsigned)c1->reason);
        CHECK(c1->max_value < 80, "LINKED #%d ch1: height %d, expected noise only",
              e, (int)c1->max_value);
    }

    std::printf("LINKED: %d events\n", log.count);
    for (int e = 0; e < log.count; e++) {
        const EventBundle& b = log.rec[e].bundle;
        std::printf("  #%u valid=%u window=%usmp/%uus", (unsigned)b.index, (unsigned)b.valid,
                    (unsigned)b.width_samples, (unsigned)b.width_us);
        for (uint8_t i = 0; i < b.n_channels; i++) {
            const ChannelMetrics& c = log.rec[e].ch[i];
            std::printf("  ch%u:%s h=%d w=%u", (unsigned)c.channel,
                        (c.flags & CH_FLAG_PRESENT) ? "P" : "-", (int)c.max_value,
                        (unsigned)c.width_samples);
        }
        std::printf("\n");
    }
}

// ---------------------------------------------------------------------------
// 2. LINKED: a channel that rises after the window closed is clipped
// ---------------------------------------------------------------------------
static void testLinkedClip() {
    // ch8 opens the window at ~6000 and closes it at ~6600; ch0 only rises at
    // 7000, so the window cuts it off while it is still climbing.
    const PeakSpec peaks8[] = { {6000, 200, 800.0} };
    const PeakSpec peaks0[] = { {7000, 200, 800.0} };
    const ChannelSpec specs[] = {
        {0, BASE, peaks0, 1},
        {1, BASE, nullptr, 0},
        {8, BASE, peaks8, 1},
    };

    EventAggregator agg(aggConfig(METRIC_MODE_LINKED, 0x0103u, 8));
    EventLog log;
    runAggregator(agg, specs, 3, 12000, 10, log);

    CHECK(log.count == 1, "clip: %d events, expected 1", log.count);
    if (log.count != 1) { return; }

    const EventRecord& r = log.rec[0];
    const EventBundle& b = r.bundle;
    const ChannelMetrics* c0 = findChannel(r, 0);
    const ChannelMetrics* c8 = findChannel(r, 8);
    CHECK(c0 && c8 && b.n_channels == 3, "clip: %u records, ch0/ch8 %s",
          (unsigned)b.n_channels, (c0 && c8) ? "present" : "missing");
    if (!c0 || !c8) { return; }

    CHECK(c0->flags == (CH_FLAG_PRESENT | CH_FLAG_TRUNCATED),
          "clip: ch0 flags 0x%02x, expected PRESENT|TRUNCATED", (unsigned)c0->flags);
    CHECK(c8->flags == CH_FLAG_PRESENT, "clip: ch8 flags 0x%02x", (unsigned)c8->flags);
    CHECK(c0->max_value > 0 && c0->max_value < 300,
          "clip: ch0 height %d, expected the window edge (~100)", (int)c0->max_value);
    CHECK(near(c8->max_value, 650, 950), "clip: ch8 height %d, expected ~800",
          (int)c8->max_value);
    CHECK(c0->max_value < c8->max_value, "clip: ch0 (%d) not clipped below ch8 (%d)",
          (int)c0->max_value, (int)c8->max_value);
    CHECK(c0->width_samples == b.width_samples, "clip: ch0 width %u != window %u",
          (unsigned)c0->width_samples, (unsigned)b.width_samples);

    std::printf("LINKED clip: #%u window=%usmp  ch8 h=%d  ch0 h=%d flags=0x%02x\n",
                (unsigned)b.index, (unsigned)b.width_samples, (int)c8->max_value,
                (int)c0->max_value, (unsigned)c0->flags);
}

// ---------------------------------------------------------------------------
// 3. PER_CHANNEL: every channel reports its own width
// ---------------------------------------------------------------------------
static void testPerChannel() {
    // Four peaks that overlap in time but have very different widths: ch0 broad,
    // ch1 narrow, ch8 medium, ch9 in between.
    const PeakSpec peaks0[] = { {6000, 200, 800.0}, {13000, 200, 800.0} };
    const PeakSpec peaks1[] = { {6100,  60, 600.0}, {13100,  60, 600.0} };
    const PeakSpec peaks8[] = { {6020, 150, 700.0}, {13020, 150, 700.0} };
    const PeakSpec peaks9[] = { {5970, 100, 500.0}, {12970, 100, 500.0} };
    const ChannelSpec specs[] = {
        {0, BASE, peaks0, 2},
        {1, BASE, peaks1, 2},
        {8, BASE, peaks8, 2},
        {9, BASE, peaks9, 2},
    };

    EventAggregator agg(aggConfig(METRIC_MODE_PER_CHANNEL, 0x0303u, 0));
    CHECK(agg.primaryChannel() == 0, "PER_CHANNEL: primary resolved to %u, expected 0",
          (unsigned)agg.primaryChannel());

    EventLog log;
    runAggregator(agg, specs, 4, 26000, 10, log);
    CHECK(log.count == 2, "PER_CHANNEL: %d events, expected 2", log.count);
    if (log.count != 2) { return; }

    for (int e = 0; e < log.count; e++) {
        const EventRecord& r = log.rec[e];
        const EventBundle& b = r.bundle;
        CHECK(b.mode == (uint8_t)METRIC_MODE_PER_CHANNEL, "PER_CHANNEL #%d: mode %u",
              e, (unsigned)b.mode);
        CHECK(b.primary == 0, "PER_CHANNEL #%d: primary %u, expected 0", e, (unsigned)b.primary);
        CHECK(b.valid == 1 && b.reason == REASON_OK, "PER_CHANNEL #%d: valid=%u reason=%u",
              e, (unsigned)b.valid, (unsigned)b.reason);
        CHECK(b.n_channels == 4, "PER_CHANNEL #%d: %u records, expected 4",
              e, (unsigned)b.n_channels);
        if (b.n_channels != 4) { continue; }

        CHECK(r.ch[0].channel == 0 && r.ch[1].channel == 1 &&
              r.ch[2].channel == 8 && r.ch[3].channel == 9,
              "PER_CHANNEL #%d: unexpected channel order", e);

        for (uint8_t i = 0; i < b.n_channels; i++) {
            const ChannelMetrics& c = r.ch[i];
            CHECK(c.flags == CH_FLAG_PRESENT, "PER_CHANNEL #%d ch%u: flags 0x%02x",
                  e, (unsigned)c.channel, (unsigned)c.flags);
            CHECK(c.reason == REASON_OK, "PER_CHANNEL #%d ch%u: reason %u",
                  e, (unsigned)c.channel, (unsigned)c.reason);
        }

        const ChannelMetrics* c0 = findChannel(r, 0);
        const ChannelMetrics* c1 = findChannel(r, 1);
        CHECK(c0 && c1, "PER_CHANNEL #%d: ch0/ch1 missing", e);
        if (!c0 || !c1) { continue; }

        // Each channel timed itself: the narrow one really is narrower, and the
        // bundle window spans the whole group.
        CHECK((double)c1->width_samples < 0.6 * (double)c0->width_samples,
              "PER_CHANNEL #%d: ch1 width %u not clearly below ch0 width %u",
              e, (unsigned)c1->width_samples, (unsigned)c0->width_samples);
        CHECK(c1->width_us >= 1 && c1->width_us < c0->width_us,
              "PER_CHANNEL #%d: ch1 width %uus vs ch0 %uus", e, (unsigned)c1->width_us,
              (unsigned)c0->width_us);
        CHECK(b.width_samples >= c0->width_samples,
              "PER_CHANNEL #%d: window %u narrower than ch0 %u",
              e, (unsigned)b.width_samples, (unsigned)c0->width_samples);
        CHECK(near(c1->max_value, 500, 700), "PER_CHANNEL #%d ch1: height %d, expected ~600",
              e, (int)c1->max_value);
        CHECK(c1->area > 0 && c1->area < c0->area,
              "PER_CHANNEL #%d: ch1 area %llu vs ch0 %llu", e,
              (unsigned long long)c1->area, (unsigned long long)c0->area);
    }

    std::printf("PER_CHANNEL: %d events\n", log.count);
    for (int e = 0; e < log.count; e++) {
        const EventBundle& b = log.rec[e].bundle;
        std::printf("  #%u valid=%u window=%usmp/%uus", (unsigned)b.index, (unsigned)b.valid,
                    (unsigned)b.width_samples, (unsigned)b.width_us);
        for (uint8_t i = 0; i < b.n_channels; i++) {
            const ChannelMetrics& c = log.rec[e].ch[i];
            std::printf("  ch%u:%usmp h=%d", (unsigned)c.channel, (unsigned)c.width_samples,
                        (int)c.max_value);
        }
        std::printf("\n");
    }
}

// ---------------------------------------------------------------------------
// 4. LINKED: the primary rails -> one notification bundle, then it recovers
// ---------------------------------------------------------------------------
static void testRail() {
    g_rng = 0x12345678u;
    EventAggregator agg(aggConfig(METRIC_MODE_LINKED, 0x0103u, 8));

    int events = 0, rails = 0, valid = 0;
    for (int i = 0; i < 14000; i++) {
        int32_t v[SORTER_MAX_CHANNELS];
        for (int c = 0; c < SORTER_MAX_CHANNELS; c++) { v[c] = 0; }
        v[0] = (int32_t)std::lround(2000.0 + gauss() * 5.0);
        v[1] = (int32_t)std::lround(2000.0 + gauss() * 5.0);

        double x8 = 2000.0 + gauss() * 5.0;
        if (i >= 4000 && i < 4200) { x8 = 32700.0; }            // bright burst: saturates
        if (i >= 8000 && i < 10000) {                           // normal peak after recovery
            const double d = (double)(i - 9000);
            x8 += 800.0 * std::exp(-(d * d) / (2.0 * 200.0 * 200.0));
        }
        v[8] = (int32_t)std::lround(x8);

        EventRecord rec;
        if (agg.update(v, (uint32_t)i * 10u, (uint32_t)i, rec)) {
            events++;
            if (rec.bundle.reason == REASON_RAILED) {
                rails++;
                CHECK(rec.bundle.valid == 0, "rail: valid=%u, expected 0",
                      (unsigned)rec.bundle.valid);
                CHECK(rec.bundle.n_channels == 3, "rail: %u records, expected 3",
                      (unsigned)rec.bundle.n_channels);
                const ChannelMetrics* c8 = findChannel(rec, 8);
                const ChannelMetrics* c0 = findChannel(rec, 0);
                CHECK(c8 && c8->flags == CH_FLAG_RAILED && c8->reason == REASON_RAILED,
                      "rail: ch8 not flagged RAILED (flags=0x%02x reason=%u)",
                      c8 ? (unsigned)c8->flags : 0u, c8 ? (unsigned)c8->reason : 0u);
                CHECK(c0 && c0->flags == 0 && c0->reason == REASON_MAX_TOO_SMALL,
                      "rail: ch0 should be absent in the rail bundle");
            } else if (rec.bundle.valid) {
                valid++;
                const ChannelMetrics* c8 = findChannel(rec, 8);
                CHECK(c8 && near(c8->max_value, 650, 950),
                      "rail: post-recovery height %d, expected ~800",
                      c8 ? (int)c8->max_value : -1);
            }
        }
    }
    CHECK(rails == 1, "rail: %d rail bundles, expected exactly 1", rails);
    CHECK(valid == 1, "rail: %d valid events after recovery, expected 1", valid);
    CHECK(events <= 4, "rail: %d events total - detector is chattering", events);
    std::printf("LINKED rail: %d events (%d rail, %d valid)\n", events, rails, valid);
}

// ---------------------------------------------------------------------------
// 5. An inverted channel (idles high, dips on an event) measured as a peak
// ---------------------------------------------------------------------------
static void testInvert() {
    g_rng = 0x12345678u;
    EventAggregator agg(aggConfig(METRIC_MODE_LINKED, (1u << 8), 8));

    int events = 0, valid = 0;
    for (int i = 0; i < 20000; i++) {
        int32_t v[SORTER_MAX_CHANNELS];
        for (int c = 0; c < SORTER_MAX_CHANNELS; c++) { v[c] = 0; }

        double x8 = 25000.0 + gauss() * 5.0;                    // idles HIGH ...
        for (int k = 0; k < 2; k++) {                           // ... dips on an event
            const int    center = (k == 0) ? 6000 : 13000;
            const double d = (double)(i - center);
            x8 -= 800.0 * std::exp(-(d * d) / (2.0 * 200.0 * 200.0));
        }
        v[8] = (int32_t)std::lround(x8);

        applyInputPolarity(v, (1u << 8));                       // the fix under test

        EventRecord rec;
        if (agg.update(v, (uint32_t)i * 10u, (uint32_t)i, rec)) {
            events++;
            if (rec.bundle.valid) {
                valid++;
                const ChannelMetrics* c8 = findChannel(rec, 8);
                CHECK(c8 && near(c8->max_value, 600, 1000),
                      "invert: ch8 height %d, expected ~800 (the dip depth)",
                      c8 ? (int)c8->max_value : -1);
            }
        }
    }
    CHECK(events == 2, "invert: %d events, expected 2", events);
    CHECK(valid == 2, "invert: %d valid events, expected 2", valid);
    std::printf("INVERT: %d events (dips detected as peaks)\n", events);
}

int main() {
    testLinked();
    testLinkedClip();
    testPerChannel();
    testRail();
    testInvert();

    if (g_failures > 0) {
        std::printf("\nFAILED: %d check(s)\n", g_failures);
        return 1;
    }
    std::printf("\nPASS: the aggregator grouped the per-channel detectors as expected "
                "in both modes.\n");
    return 0;
}
