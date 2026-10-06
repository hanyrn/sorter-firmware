// sorter-firmware.ino
//
// Arduino GIGA R1 WiFi (STM32H747) firmware.
//
// The GIGA runs this SAME sketch on BOTH cores; the code path is selected at
// runtime with RPC.cpu_id().  Upload with the GIGA board selected.
//
//   Cortex-M4 (240 MHz) -- full acquisition + signal-processing pipeline:
//     * reads two AD7606 modules over a shared 16-bit parallel bus,
//     * combines the enabled channels into one sample per conversion
//       (SORTER_ACTIVE_CHANNEL_MASK; all 16 in the real application),
//     * detects peak events on an adaptive baseline + std-dev threshold,
//     * computes max value / area / width and rejects false peaks,
//     * forwards each closed event to the M7 over the RPC raw endpoint.
//
//   Bench mode (SORTER_SIGNAL_GEN = 1; see signal_gen.{h,cpp} and the README):
//     the same M4 loop additionally drives the two DAC outputs (A12/A13) with a
//     known synthetic waveform, so the detector can be validated on real
//     hardware through a DAC -> AD7606 loopback instead of a real signal.
//
//   Cortex-M7 (480 MHz) -- placeholder consumer of the peak metrics. For now it
//     only prints them; the higher-level application logic is added later.

#include <RPC.h>
using namespace rtos;

#include "config.h"
#include "peak_metrics.h"
#include "event_detector.h"
#include "adc7606_parallel.h"
#include "metrics_sink.h"
#include "signal_gen.h"

// ===========================================================================
// M4: acquisition + detection pipeline
// ===========================================================================
static EventDetector   g_detector;
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

// Runs forever on the M4: generate, convert, read, combine, detect, report.
static void acquisitionTask() {
    g_detector.configure(makeDetectorConfig());
    g_adc.begin();
    g_adc.reset();
#if SORTER_SIGNAL_GEN
    g_signalGen.begin();     // bench test: bring the DACs up on the baseline
#endif

    int32_t     channels[AD7606_NUM_CHANNELS];
    PeakMetrics metrics;
    uint32_t    index          = 0;
    uint32_t    next_sample_us = micros();

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

        int64_t  sum = 0;
        uint32_t n   = 0;
        for (uint8_t i = 0; i < AD7606_NUM_CHANNELS; i++) {
            if ((SORTER_ACTIVE_CHANNEL_MASK & (1uL << i)) != 0u) {
                sum += channels[i];
                n++;
            }
        }
        if (n == 0u) { n = 1u; }   // guard against an empty channel mask
#if SORTER_COMBINE_MEAN
        const int32_t combined = (int32_t)(sum / (int64_t)n);
#else
        const int32_t combined = (int32_t)sum;
#endif

        if (g_detector.update(combined, micros(), index, metrics)) {
            sorter::sendPeak(metrics);   // hand the closed event to the M7
        }
        index++;
    }
}

// ===========================================================================
// M7: receive peak metrics from the M4
// ===========================================================================
static Mail<PeakMetrics, 16> g_m7Mail;

static void onMetricsRaw(const uint8_t* buf, size_t len) {
    PeakMetrics m;
    if (!sorter::parsePeak(buf, len, m)) {
        return;
    }
    PeakMetrics* slot = g_m7Mail.try_alloc();
    if (slot != nullptr) {
        *slot = m;
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
        default:                      return "UNKNOWN";
    }
}

static void printMetrics(const PeakMetrics& m) {
    Serial.print("Peak #");    Serial.print(m.index);
    Serial.print(m.valid ? "  VALID     " : "  rejected  ");
    Serial.print("reason=");    Serial.print(m.reason);
    Serial.print(' ');          Serial.print(reasonName(m.reason));
    Serial.print("  max=");     Serial.print(m.max_value);
    Serial.print("  area=");    Serial.print((uint32_t)m.area);
    Serial.print("  width=");   Serial.print(m.width_us);
    Serial.print("us/");        Serial.print(m.width_samples);
    Serial.print("smp  base="); Serial.print(m.baseline);
    Serial.print("  sigma=");   Serial.println(m.noise);
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
        Serial.print(" Hz, combining channel mask 0x");
        Serial.print((unsigned)SORTER_ACTIVE_CHANNEL_MASK, HEX);
        Serial.println('.');
        Serial.println("Expected per 2 s pattern cycle: 3 VALID peaks + 3 rejected false peaks.");
#endif
    }
}

void loop() {
    if (RPC.cpu_id() == CM7_CPUID) {
        PeakMetrics* m;
        while ((m = g_m7Mail.try_get()) != nullptr) {
            printMetrics(*m);
            g_m7Mail.free(m);
        }
        delay(1);
    } else {
        delay(1000);   // M4: the work happens in the acquisition thread
    }
}
