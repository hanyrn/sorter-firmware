// sorter-firmware.ino
//
// Arduino GIGA R1 WiFi (STM32H747) firmware.
//
// The GIGA runs this SAME sketch on BOTH cores; the code path is selected at
// runtime with RPC.cpu_id().  Upload with the GIGA board selected.
//
//   Cortex-M4 (240 MHz) -- full acquisition + signal-processing pipeline:
//     * reads two AD7606 modules, each on its own 16-bit parallel bus, with
//       shared control lines (one instant for all 16 channels),
//     * runs one peak detector per channel (own baseline + std-dev threshold), so
//       every channel gets its own height / width / area,
//     * groups the per-channel results of one event into a bundle
//       (event_aggregator.h: linked to a primary channel, or per-channel - see
//       SORTER_METRIC_MODE) and rejects false peaks,
//     * forwards each completed event to the M7 over the RPC raw endpoint.
//
//   Bench mode (SORTER_SIGNAL_GEN = 1; see signal_gen.{h,cpp} and the README):
//     the same M4 loop additionally drives the two DAC outputs (A12/A13) with a
//     known synthetic waveform, so the detector can be validated on real
//     hardware through a DAC -> AD7606 loopback instead of a real signal.
//
//   Cortex-M7 (480 MHz) -- placeholder consumer of the event metrics. For now it
//     only prints them (one line per channel); the higher-level application logic
//     is added later.

#include <RPC.h>
using namespace rtos;

#include "config.h"
#include "peak_metrics.h"
#include "event_detector.h"
#include "event_aggregator.h"
#include "adc7606_parallel.h"
#include "metrics_sink.h"
#include "signal_gen.h"

// ===========================================================================
// M4: acquisition + detection pipeline
// ===========================================================================
static EventAggregator g_aggregator;
static EventRecord     g_event;      // staging record for the event being measured
static Adc7606Parallel g_adc;
static Thread          g_acqThread(osPriorityHigh, 4096, nullptr, "adc");
#if SORTER_SIGNAL_GEN
static sorter::SignalGen g_signalGen;   // bench test: drives the DACs (A12/A13)
#endif

// Pace the acquisition loop to SORTER_SAMPLE_RATE_HZ so the detector runs at the
// rate its bounds were written for (and, in bench mode, so the injected pattern
// is not smeared by the AD7606 analog front end). With SORTER_SAMPLE_RATE_HZ = 0
// the loop free-runs at whatever rate the parallel-bus reads allow.
static void waitNextSample(uint32_t& next_us) {
#if SORTER_SAMPLE_RATE_HZ > 0
    const uint32_t period_us = 1000000u / SORTER_SAMPLE_RATE_HZ;
    for (;;) {
        const int32_t remaining = (int32_t)(next_us - micros());
        if (remaining <= 0) {
            if (remaining < -(int32_t)period_us) {
                next_us = micros() + period_us;   // ran late: resync, never burst
            } else {
                next_us += period_us;             // on time: keep the time grid
            }
            return;
        }
        if (remaining > 1000) {
            delay((unsigned long)(remaining / 1000) - 1u);
        }
    }
#else
    (void)next_us;
#endif
}

static EventDetector::Config makeDetectorConfig() {
    EventDetector::Config c;
    c.baseline_alpha    = DET_BASELINE_ALPHA;
    c.noise_alpha       = DET_NOISE_ALPHA;
    c.k_on              = DET_K_ON;
    c.k_off             = DET_K_OFF;
    c.k_gate            = DET_K_GATE;
    c.sigma_floor       = DET_SIGMA_FLOOR;
    c.warmup_samples    = DET_WARMUP_SAMPLES;
    c.end_debounce      = DET_END_DEBOUNCE;
    c.min_max_value     = DET_MIN_MAX_VALUE;
    c.min_width_samples = DET_MIN_WIDTH_SAMPLES;
    c.max_width_samples = DET_MAX_WIDTH_SAMPLES;
    c.min_area          = DET_MIN_AREA;
    c.max_slope         = DET_MAX_SLOPE;
    return c;
}

// How the per-channel detectors are grouped into one event bundle.
static EventAggregator::Config makeAggregatorConfig() {
    EventAggregator::Config c;
    c.detector     = makeDetectorConfig();
    c.channel_mask = SORTER_ACTIVE_CHANNEL_MASK;
    c.mode         = SORTER_METRIC_MODE ? METRIC_MODE_LINKED : METRIC_MODE_PER_CHANNEL;
    c.primary      = (uint8_t)SORTER_PRIMARY_CHANNEL;
    c.rail_level    = DET_RAIL_LEVEL;
    c.rail_width    = DET_RAIL_WIDTH;
    c.rail_cooldown = DET_RAIL_COOLDOWN;
    return c;
}

// Runs forever on the M4: generate, convert, read, detect per channel, report.
static void acquisitionTask() {
    g_aggregator.configure(makeAggregatorConfig());
    g_adc.begin();
    g_adc.reset();
#if SORTER_SIGNAL_GEN
    g_signalGen.begin();     // bench test: bring the DACs up on the baseline
#endif

    int32_t  channels[AD7606_NUM_CHANNELS];
    uint32_t index          = 0;
    uint32_t next_sample_us = micros();

    for (;;) {
        waitNextSample(next_sample_us);

#if SORTER_SIGNAL_GEN
        // Sample-locked DAC update: the value written here is the one the
        // AD7606 samples (holding it for one whole sample period means the
        // analog front end has fully settled). This is a deterministic
        // one-sample delay, which no peak metric depends on.
        g_signalGen.tick();
#endif

        g_adc.readAll(channels);

        // Some channels idle HIGH and dip when an event occurs (see
        // SORTER_INVERT_CHANNEL_MASK): invert them so the one rising-edge peak
        // detector fits every channel. This runs before detection, so both the
        // per-channel detectors and the aggregator's window logic see the same
        // (inverted) value.
        applyInputPolarity(channels, SORTER_INVERT_CHANNEL_MASK);

        // Every channel is measured on its own; a completed event carries one
        // record per active channel and is handed to the M7 in a single frame.
        if (g_aggregator.update(channels, micros(), index, g_event)) {
            sorter::sendBundle(g_event.bundle, g_event.ch);
        }
        index++;
    }
}

// ===========================================================================
// M7: receive event bundles from the M4
// ===========================================================================
static Mail<EventRecord, 4> g_m7Mail;   // events are rare: 4 pending is plenty

static void onMetricsRaw(const uint8_t* buf, size_t len) {
    EventRecord rec;
    if (!sorter::parseBundle(buf, len, rec)) {
        return;
    }
    EventRecord* slot = g_m7Mail.try_alloc();
    if (slot != nullptr) {
        *slot = rec;
        g_m7Mail.put(slot);
    }
}

// Human-readable form of PeakRejectReason, so the M7 log can be read at a glance.
static const char* reasonName(uint8_t reason) {
    switch (reason) {
        case REASON_OK:               return "OK";
        case REASON_MAX_TOO_SMALL:    return "MAX_TOO_SMALL";
        case REASON_WIDTH_TOO_NARROW: return "WIDTH_TOO_NARROW";
        case REASON_WIDTH_TOO_WIDE:   return "WIDTH_TOO_WIDE";
        case REASON_AREA_TOO_SMALL:   return "AREA_TOO_SMALL";
        case REASON_SHORT_FOR_HEIGHT: return "SHORT_FOR_HEIGHT";
        case REASON_RAILED:           return "RAILED";
        default:                      return "UNKNOWN";
    }
}

// One line for the event (window + verdict), then one line per active channel.
static void printBundle(const EventRecord& rec) {
    const EventBundle& b = rec.bundle;

    Serial.print("Event #");    Serial.print(b.index);
    Serial.print(b.valid ? "  VALID     " : "  rejected  ");
    Serial.print("reason=");    Serial.print(b.reason);
    Serial.print(' ');          Serial.print(reasonName(b.reason));
    Serial.print("  window=");  Serial.print(b.start_us);
    Serial.print("..");         Serial.print(b.end_us);
    Serial.print("us/");        Serial.print(b.width_samples);
    Serial.print("smp  primary=ch"); Serial.print(b.primary);
    Serial.print("  mode=");
    Serial.println(b.mode == (uint8_t)METRIC_MODE_LINKED ? "linked" : "per-channel");

    if (b.reason == REASON_RAILED) {
        Serial.println("*** SIGNAL TOO BRIGHT: an input railed (saturated). Reduce the "
                       "source intensity; that channel's detector was reset. ***");
    }

    for (uint8_t i = 0; i < b.n_channels; i++) {
        const ChannelMetrics& c = rec.ch[i];
        Serial.print("  ch");       Serial.print(c.channel);
        if ((c.flags & CH_FLAG_RAILED) != 0u) {
            Serial.println("  RAILED (input saturated)");
            continue;
        }
        if ((c.flags & CH_FLAG_PRESENT) == 0u) {
            Serial.println("  absent");
            continue;
        }
        Serial.print("  max=");     Serial.print(c.max_value);
        Serial.print("  area=");    Serial.print((uint32_t)c.area);
        Serial.print("  width=");   Serial.print(c.width_us);
        Serial.print("us/");        Serial.print(c.width_samples);
        Serial.print("smp  base="); Serial.print(c.baseline);
        Serial.print("  ");
        if (metricsValid(c.reason)) {
            Serial.print("VALID");
        } else {
            Serial.print("rejected ");
            Serial.print(reasonName(c.reason));
        }
        if ((c.flags & CH_FLAG_TRUNCATED) != 0u) { Serial.print("  [truncated]"); }
        Serial.println();
    }
}

// ===========================================================================
// Arduino entry points (both cores call these)
// ===========================================================================
void setup() {
    Serial.begin(115200);
    RPC.begin();    // on the M7 this also boots the M4

    if (RPC.cpu_id() == CM4_CPUID) {
        g_acqThread.start(acquisitionTask);
    } else {
        RPC.attach(onMetricsRaw);   // M7: raw sink for M4 metric frames
        Serial.println("M7 ready: acquisition + detection run on the M4.");
#if SORTER_SIGNAL_GEN
        Serial.print("BENCH MODE: the M4 drives DAC A12/A13 with a known pattern at ");
        Serial.print(SORTER_SAMPLE_RATE_HZ);
        Serial.print(" Hz, measuring channel mask 0x");
        Serial.print((unsigned)SORTER_ACTIVE_CHANNEL_MASK, HEX);
        Serial.print(" in ");
        Serial.print(SORTER_METRIC_MODE ? "linked" : "per-channel");
        Serial.print(" mode (primary channel ");
        Serial.print(SORTER_PRIMARY_CHANNEL);
        Serial.println(").");
        Serial.println("Expected per 2 s pattern cycle: 3 VALID events (one 'ch' line per");
        Serial.println("measured channel) + 3 rejected events.");
#endif
    }
}

void loop() {
    if (RPC.cpu_id() == CM7_CPUID) {
        EventRecord* r;
        while ((r = g_m7Mail.try_get()) != nullptr) {
            printBundle(*r);
            g_m7Mail.free(r);
        }
        delay(1);
    } else {
        delay(1000);   // M4: the work happens in the acquisition thread
    }
}
