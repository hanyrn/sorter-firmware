// sorter-firmware.ino
//
// Arduino GIGA R1 WiFi (STM32H747) firmware.
//
// The GIGA runs this SAME sketch on BOTH cores; the code path is selected at
// runtime with RPC.cpu_id().  Upload with the GIGA board selected.
//
//   Cortex-M4 (240 MHz) -- full acquisition + signal-processing pipeline:
//     * reads two AD7606 modules over a shared 16-bit parallel bus,
//     * combines all 16 channels into one sample per conversion,
//     * detects peak events on an adaptive baseline + std-dev threshold,
//     * computes max value / area / width and rejects false peaks,
//     * forwards each closed event to the M7 over the RPC raw endpoint.
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

// ===========================================================================
// M4: acquisition + detection pipeline
// ===========================================================================
static EventDetector   g_detector;
static Adc7606Parallel g_adc;
static Thread          g_acqThread(osPriorityHigh, 4096, nullptr, "adc");

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

// Runs forever on the M4: convert, read, combine, detect, report.
static void acquisitionTask() {
    g_detector.configure(makeDetectorConfig());
    g_adc.begin();
    g_adc.reset();

    int32_t     channels[AD7606_NUM_CHANNELS];
    PeakMetrics metrics;
    uint32_t    index = 0;

    for (;;) {
        g_adc.readAll(channels);

        int64_t sum = 0;
        for (uint8_t i = 0; i < AD7606_NUM_CHANNELS; i++) {
            sum += channels[i];
        }
#if SORTER_COMBINE_MEAN
        const int32_t combined = (int32_t)(sum / AD7606_NUM_CHANNELS);
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

static void printMetrics(const PeakMetrics& m) {
    Serial.print("Peak #");    Serial.print(m.index);
    Serial.print(m.valid ? "  VALID" : "  rejected");
    Serial.print("  reason=");  Serial.print(m.reason);
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
