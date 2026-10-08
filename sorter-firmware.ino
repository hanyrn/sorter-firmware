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
//
//   Live datastream (test/live_plot.py): besides the event log the M4 ships
//     decimated raw samples (stream_sink.h) to the M7, which can print them as
//     `S,<index>,<code>,<ch>:<value>...` lines.  That is the live plot window.
//     Streaming is OFF at boot - switch it on with `T` + Enter in the serial
//     monitor (or let live_plot.py send the command itself), so the human-readable
//     event log stays readable.

#include <RPC.h>
using namespace rtos;

#include "config.h"
#include "peak_metrics.h"
#include "event_detector.h"
#include "event_aggregator.h"
#include "adc7606_parallel.h"
#include "metrics_sink.h"
#include "signal_gen.h"
#include "stream_sink.h"

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

    // Which AD7606 channels the sample stream carries: the first
    // SORTER_STREAM_CHANNELS entries of SORTER_ACTIVE_CHANNEL_MASK, resolved once
    // instead of per sample (0xFF = the mask does not have that many channels,
    // see stream_sink.h).
    uint8_t stream_channels[SORTER_STREAM_CHANNELS];
    for (uint8_t k = 0; k < (uint8_t)SORTER_STREAM_CHANNELS; k++) {
        stream_channels[k] = sorter::streamChannelIndex(k);
    }

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

        // ---- sample stream for the live plot window -------------------------
        // One point every SORTER_STREAM_DECIM-th sample, carrying the generated
        // DAC code (the "expected" trace) plus the detector-input value of the
        // first SORTER_STREAM_CHANNELS active channels (post polarity, i.e.
        // exactly what the detectors see).  The M4 ships these frames
        // unconditionally and the M7 decides whether to print them, so the
        // readable event log is untouched until streaming is switched on.
        if ((index % SORTER_STREAM_DECIM) == 0u) {
            int32_t values[SORTER_STREAM_CHANNELS];
            for (uint8_t k = 0; k < (uint8_t)SORTER_STREAM_CHANNELS; k++) {
                const uint8_t ch = stream_channels[k];
                values[k] = (ch < (uint8_t)AD7606_NUM_CHANNELS) ? channels[ch] : 0;
            }
#if SORTER_SIGNAL_GEN
            sorter::streamSample(index, g_signalGen.code(), values);
#else
            sorter::streamSample(index, 0, values);   // no generator -> gen == 0
#endif
        }

        // Every channel is measured on its own; a completed event carries one
        // record per active channel and is handed to the M7 in a single frame.
        if (g_aggregator.update(channels, micros(), index, g_event)) {
            sorter::sendBundle(g_event.bundle, g_event.ch);
        }
        index++;
    }
}

// ===========================================================================
// M7: receive event bundles and sample frames from the M4
// ===========================================================================
static Mail<EventRecord, 4> g_m7Mail;      // events are rare: 4 pending is plenty
static Mail<sorter::StreamFrame, 4> g_streamMail;  // ~400 bytes each, ~16 frames/s

// Raw sink for everything the M4 sends: one endpoint, two frame kinds, told apart
// by the magic word at the front of the frame.
static void onM4RawFrame(const uint8_t* buf, size_t len) {
    uint16_t magic = 0xFFFFu;
    if (len >= sizeof(magic)) {
        memcpy(&magic, buf, sizeof(magic));
    }

    if (magic == sorter::STREAM_FRAME_MAGIC) {
        sorter::StreamFrame* slot = g_streamMail.try_alloc();
        if (slot == nullptr) {
            return;                        // M7 is behind: drop this frame
        }
        if (!sorter::parseStreamFrame(buf, len, *slot)) {
            g_streamMail.free(slot);
            return;
        }
        g_streamMail.put(slot);
        return;
    }

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
// M7: sample stream -> USB serial (the live plot window)
// ===========================================================================
// The M4 sends the frames all the time; the M7 stays quiet until the host asks for
// them, so the readable event log is unaffected when nobody is plotting.
//
//   T      toggle the stream          T1 / T0   on / off
//   ?      list the commands
static bool     g_streamOn    = false;
static uint32_t g_streamLines = 0;      // points printed since streaming came on

// Metadata line: the host learns the decimation (=> the sample rate) and which
// channels the columns carry from here instead of hard-coding config.h.
static void printStreamInfo() {
    Serial.print("STREAM ");
    Serial.print(g_streamOn ? "on" : "off");
    Serial.print(": decim=");
    Serial.print((unsigned)SORTER_STREAM_DECIM);
    Serial.print(" chans=");
    uint8_t printed = 0;
    for (uint8_t k = 0; k < (uint8_t)SORTER_STREAM_CHANNELS; k++) {
        const uint8_t ch = sorter::streamChannelIndex(k);
        if (ch == 0xFFu) { continue; }
        if (printed++ > 0u) { Serial.print(','); }
        Serial.print(ch);
    }
    Serial.print(" wf=");           // 1 = the gen column is a real generated code
#if SORTER_SIGNAL_GEN
    Serial.print('1');
#else
    Serial.print('0');
#endif
    Serial.print(" points=");
    Serial.print((unsigned)SORTER_STREAM_POINTS);
    Serial.println();
    g_streamLines = 0;
}

// One line per point:   S,<sample index>,<gen code>,<ch>:<value>[,...]
// Deliberately terse - at the default decimation this is 1000 lines/s.
static void printStreamFrame(const sorter::StreamFrame& f) {
    for (uint16_t i = 0; i < f.header.count; i++) {
        const sorter::StreamPoint& p = f.point[i];
        Serial.print('S');
        Serial.print(',');
        Serial.print(f.header.first_index + (uint32_t)i * (uint32_t)f.header.decim);
        Serial.print(',');
        Serial.print(p.gen);
        for (uint8_t k = 0; k < f.header.n_channels; k++) {
            Serial.print(',');
            Serial.print(f.header.channels[k]);
            Serial.print(':');
            Serial.print(p.adc[k]);
        }
        Serial.println();
        g_streamLines++;
    }
}

static void handleStreamCommand(const char* cmd, uint8_t len) {
    const char c = cmd[0];
    if (c == 't' || c == 'T') {
        g_streamOn = (len >= 2) ? (cmd[1] == '1') : !g_streamOn;
        printStreamInfo();
    } else if (c == '?') {
        Serial.println("Commands: T = toggle the sample stream, T1/T0 = on/off, ? = this list.");
    }
}

// The board's only input channel: line-terminated commands on USB serial.  Line
// based on purpose, so a stray keystroke in the monitor cannot switch the stream
// on accidentally.
static void pollStreamCommand() {
    static char    cmd[16];
    static uint8_t n = 0;

    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) { break; }
        if (c == '\r' || c == '\n') {
            if (n > 0u) {
                cmd[n] = '\0';
                handleStreamCommand(cmd, n);
                n = 0;
            }
            continue;
        }
        if (c == ' ' || c == '\t') { continue; }
        if (n < (uint8_t)(sizeof(cmd) - 1u)) {
            cmd[n++] = (char)c;
        } else {
            n = 0;                          // overlong garbage: drop the line
        }
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
        RPC.attach(onM4RawFrame);   // M7: raw sink for M4 metric + sample frames
        Serial.println("M7 ready: acquisition + detection run on the M4.");
#if SORTER_SIGNAL_GEN
        Serial.print("BENCH MODE: the M4 drives DAC A12/A13 with a synthetic waveform at ");
        Serial.print(SORTER_SAMPLE_RATE_HZ);
        Serial.print(" Hz, measuring channel mask 0x");
        Serial.print((unsigned)SORTER_ACTIVE_CHANNEL_MASK, HEX);
        Serial.print(" in ");
        Serial.print(SORTER_METRIC_MODE ? "linked" : "per-channel");
        Serial.print(" mode (primary channel ");
        Serial.print(SORTER_PRIMARY_CHANNEL);
        Serial.println(").");
        Serial.println("Waveform: a NEW random pulse train every pattern cycle - Gaussian");
        Serial.println("peak heights/widths, uniform gaps, and a share of rectangular");
        Serial.println("glitches that must be rejected (SIGNALGEN_* in config.h).");
#endif
#if SORTER_M7_HEARTBEAT_MS
        Serial.print("Heartbeat: a liveness line every ");
        Serial.print(SORTER_M7_HEARTBEAT_MS);
        Serial.println(" ms whenever no event has just been printed.");
#endif
        Serial.println("Sample stream (live plot): type T + Enter to start/stop it, ? for help.");
    }
}

void loop() {
    if (RPC.cpu_id() == CM7_CPUID) {
        // Print every event the M4 sent, remembering the moment of the last
        // printout so an idle heartbeat can fill the quiet gaps.
        static uint32_t lastOutputMs = 0;
        static uint32_t eventCount   = 0;

        EventRecord* r;
        while ((r = g_m7Mail.try_get()) != nullptr) {
            printBundle(*r);
            g_m7Mail.free(r);
            lastOutputMs = millis();
            eventCount++;
        }

        // Commands from the host (T = toggle the sample stream, T1/T0 = on/off).
        pollStreamCommand();

        // Sample stream -> serial, only while it is switched on.  With streaming
        // off the queued frames are simply dropped, so switching it on later starts
        // from fresh samples instead of replaying stale ones.  The per-pass budget
        // bounds the pass, so commands and the heartbeat stay responsive.
        uint8_t budget = 8u;
        sorter::StreamFrame* f;
        while (budget > 0u && (f = g_streamMail.try_get()) != nullptr) {
            if (g_streamOn) {
                printStreamFrame(*f);
                lastOutputMs = millis();   // streaming counts as output: no heartbeat
                budget--;
            }
            g_streamMail.free(f);
        }

        // Liveness heartbeat: if nothing has been printed for a while, say so, so
        // the board is visibly alive even with no events (no sensors / no DAC ->
        // AD7606 loopback).  Suppressed while events flow, so it never interleaves
        // with an event's block.  SORTER_M7_HEARTBEAT_MS = 0 disables it.
#if SORTER_M7_HEARTBEAT_MS
        if (millis() - lastOutputMs >= SORTER_M7_HEARTBEAT_MS) {
            Serial.print("M7 heartbeat: ");
            Serial.print(millis() / 1000u);
            Serial.print(" s up, ");
            Serial.print(eventCount);
            Serial.println(" event(s) so far.");
            lastOutputMs = millis();
        }
#endif
        delay(1);
    } else {
        delay(1000);   // M4: the work happens in the acquisition thread
    }
}
