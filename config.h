// config.h
//
// Board wiring and tuning constants for the M4 acquisition/detection firmware.
// Included by the .ino and by adc7606_parallel.cpp. Not used by the host test.

#pragma once

#include <Arduino.h>

// ===========================================================================
// Bench test: DAC -> AD7606 loopback
// ===========================================================================
// SORTER_SIGNAL_GEN = 1 makes the M4 drive the GIGA's two 12-bit DAC outputs so
// a *known* synthetic waveform can be fed straight into the AD7606 inputs,
// turning the whole chain (parallel bus -> per-channel detectors -> event
// aggregation -> RPC -> M7 printout) into a hardware-in-the-loop check of the
// peak detector.
//
//   DAC_0 / DAC = A12 = PA_4 = DAC1_OUT1  ->  AD7606 module 0 input
//   DAC_1       = A13 = PA_5 = DAC1_OUT2  ->  AD7606 module 1 input
//
// Set this to 0 for the real application: the DACs are then left untouched and
// every active channel is measured on its own (see SORTER_ACTIVE_CHANNEL_MASK).
#define SORTER_SIGNAL_GEN     1

// Fixed acquisition rate. 0 = free-run at the fastest rate the loop sustains.
// 10 kSPS is the rate the detector bounds are written for (and the rate used by
// test/model_check.py); it also guarantees the AD7606 analog input settles
// between samples, so the injected waveform is not smeared by the module's
// anti-aliasing filter.
#define SORTER_SAMPLE_RATE_HZ 10000u

// Which channels carry a signal worth measuring:  bit i = channel i
// (0..7 = module 0, 8..15 = module 1). Every listed channel keeps its own
// baseline/noise tracker and its own per-event metrics (see SORTER_METRIC_MODE);
// a channel outside the mask is neither evaluated nor reported, so an unused
// (floating) input can never inject a phantom event - and tracking fewer channels
// leaves more headroom per sample in the acquisition loop.
// In the real application only the 9 wired channels are used: 0..8, i.e. 0x01FF.
// The other 7 inputs (9..15) are not connected to any sensor, so they are not
// tracked at all.
// In the bench loopback only the DAC-driven inputs (0 and 8) are live.
#if SORTER_SIGNAL_GEN
  #define SORTER_ACTIVE_CHANNEL_MASK  ((1u << 0) | (1u << 8))
#else
  #define SORTER_ACTIVE_CHANNEL_MASK  0x000001FFu   // channels 0..8 (9 wired inputs)
#endif

// ===========================================================================
// How one event is assembled from the per-channel metrics
// ===========================================================================
// The detector runs once per channel (each channel gets its own baseline, noise
// floor and threshold), so height / width / area are measured per channel.  How
// the per-channel results are tied together into one event is selectable:
//
//   SORTER_METRIC_MODE = 1 (linked, the default)
//       SORTER_PRIMARY_CHANNEL times the event: the shared window opens when
//       that channel rises above its on-threshold and closes when it falls back
//       below.  Every other channel's height/area is measured INSIDE that window
//       only, and all channels of the event report the primary's width (one
//       uniform event width).  Use a channel that is present for every event
//       (the 9th here) as the primary.
//
//   SORTER_METRIC_MODE = 0 (per-channel)
//       Every channel is treated separately: its own start, its own end and its
//       own width, so the widths legitimately differ from channel to channel.
//       The event is reported once ALL channels that entered it have closed
//       again (the detector's own max_width_samples guard closes a channel that
//       never returns, so an event can never hang).
//
// In both modes the M4 sends one bundle per event: the window plus one record
// per active channel.  A channel that never rose is reported as absent.
#define SORTER_METRIC_MODE     1
#define SORTER_PRIMARY_CHANNEL 8

// Per-channel input polarity: bit i = 1 means channel i is measured inverted.  The
// primary channel (the 9th, index 8) idles HIGH and DIPS when an event occurs -
// the opposite of the other eight channels - so it is inverted here and then
// handled by the very same rising-edge peak logic (its height/area then read as
// the dip depth).  Clear a bit for any channel that rises like the majority.
//
// In the bench loopback BOTH DACs output the same (positive) waveform, so no
// channel is inverted there - otherwise the injected peaks would be flipped.
#if SORTER_SIGNAL_GEN
  #define SORTER_INVERT_CHANNEL_MASK  0u
#else
  #define SORTER_INVERT_CHANNEL_MASK  (1u << SORTER_PRIMARY_CHANNEL)
#endif

// ===========================================================================
// M7 serial output
// ===========================================================================
// The M7 prints one block per event bundle (printBundle() in the .ino).  When no
// event has been printed for this long, it emits a short liveness heartbeat
// instead, so the board can be seen to be alive -- and the M4 -> M7 link shown to
// be up -- even when nothing is being detected (no sensors attached, or no
// DAC -> AD7606 loopback wired in bench mode).  The heartbeat is suppressed
// while events are flowing, so it never interleaves with an event's output.
// Set to 0 to disable the heartbeat.
#define SORTER_M7_HEARTBEAT_MS  2000u

// ===========================================================================
// Sample streaming (M4 -> M7 -> USB serial) -- the live plot window
// ===========================================================================
// Besides the per-event blocks, the M4 can ship raw waveform samples to the M7,
// which prints them as `S,<index>,<code>[,<ch>:<value>...]` lines.  That is what
// test/live_plot.py draws live, and it is the only way to *see* the synthetic
// signal (and, once the modules are wired, the measured one) without a scope.
//
// Streaming is deliberately quiet by default: the M4 fills the frames
// continuously, but the M7 prints them only once streaming has been switched on
// from the host - `T` in the serial monitor, or live_plot.py, which sends it on
// startup.  So the ordinary monitor output stays readable.
//
// Cost: SORTER_SAMPLE_RATE_HZ / SORTER_STREAM_DECIM points per second, each
// carrying 2 + 2*SORTER_STREAM_CHANNELS bytes over the M4 -> M7 link; at the
// defaults that is 1000 points/s (8 kB/s) in 16 frames/s.  One frame must stay
// below the 512-byte RPMSG payload buffer - stream_sink.h asserts that.
#define SORTER_STREAM_POINTS    64u   // points per frame
#define SORTER_STREAM_DECIM     10u   // ship every Nth sample (10 = 1 kSPS at 10 kSPS)
#define SORTER_STREAM_CHANNELS  2u    // measured channels carried, in mask order


// ===========================================================================
// AD7606 parallel buses -- Arduino GIGA R1 WiFi digital pins
// ===========================================================================
//
// TWO independent 16-bit data buses, ONE set of shared control lines:
//
//   * every module keeps its OWN DB0..DB15, so both 16-bit words are on the
//     GIGA at the same time and the modules can never fight over a bus (no
//     three-state timing, no per-module CS sequencing);
//   * CONVST / RESET / RD / CS are *tied together* and driven by one GIGA pin
//     each, so both modules convert, reset and shift out their data on exactly
//     the same edges.  Channel n of module 0 and channel n of module 1 are
//     therefore latched in the same RD cycle -> the 16 channels are one
//     snapshot of the same instant;
//   * BUSY stays per module: BUSY is a push-pull *output* on the module side,
//     so BUSY0/BUSY1 must never be tied to each other (that shorts drivers).
//
// Data lines (MCU inputs, 2 x 16 = 32 lines): the GIGA digital block
// D22..D53 -- one 16-line run per module (one 16-way header / ribbon each):
//   module 0  DB0..DB15  -> D22 D23 D24 D25 D26 D27 D28 D29 D30 D31 D32 D33
//                           D34 D35 D36 D37
//   module 1  DB0..DB15  -> D38 D39 D40 D41 D42 D43 D44 D45 D46 D47 D48 D49
//                           D50 D51 D52 D53
//
// Control lines (shared, MCU outputs unless noted): the analog pins A0..A5, so
// the whole control set sits on the analog header instead of the digital block.
//   CONVST  A0 = PC4    (rising edge starts the conversion on both modules)
//   RESET   A1 = PC5
//   RD      A2 = PB0    (one strobe advances BOTH modules to the next channel)
//   CS      A3 = PB1    (shared: asserted for the whole 8-strobe burst)
//   BUSY0   A4 = PC3    (input, module 0 end-of-conversion)
//   BUSY1   A5 = PC2    (input, module 1 end-of-conversion)
//   OS0..2  tie to GND  (no oversampling = maximum sample rate)
//
// The remaining module pins are static straps and are NOT wired to the MCU:
// RANGE selects +/-5 V (tie GND) or +/-10 V (tie 3.3 V) on BOTH modules (see
// SIGNALGEN_ADC_PER_DAC below), CONVST A and CONVST B are tied together per
// module, FRSTDATA is an unused output, VDRIVE must be 3.3 V and PAR/SER must be
// low.  See README, "Module silkscreen -> AD7606 pin".
//
// 38 GIGA pins total: 32 data lines on the digital block D22..D53 plus the six
// control/BUSY lines on A0..A5.  All of them are plain GPIOs (no peripheral
// defaults; A0..A5 are ADC1 inputs that also work as digital I/O).  A6/A7 stay
// free, and A12/A13 stay reserved for the bench-test DACs.  Any other free pin
// works too - the read path only needs the (port, bit) of each line, see
// AD7606_DB_LINES below.
// ---------------------------------------------------------------------------

#define AD7606_NUM_MODULES    2
#define AD7606_CH_PER_MODULE  8
#define AD7606_NUM_CHANNELS    (AD7606_NUM_MODULES * AD7606_CH_PER_MODULE)

// DB0..DB15 per module, and the total number of data lines on the GIGA.
#define AD7606_DB_PER_MODULE  16
#define AD7606_NUM_DB_LINES   (AD7606_NUM_MODULES * AD7606_DB_PER_MODULE)

// One data line: 'pin' is the Arduino number (used once for pinMode() in
// begin()); 'port'/'bit' are what the acquisition loop reads (port->IDR).
struct Ad7606DbLine {
    uint8_t       pin;
    GPIO_TypeDef* port;
    uint8_t       bit;
};

// Bit i of the word read from a module is its DB_i line.
// Entries 0..15 = module 0 DB0..DB15, entries 16..31 = module 1 DB0..DB15.
static const Ad7606DbLine AD7606_DB_LINES[AD7606_NUM_DB_LINES] = {
    // ---- module 0 : GIGA D22..D37 ---------------------------------------
    { D22, GPIOJ, 12 }, { D23, GPIOG, 13 }, { D24, GPIOG, 12 }, { D25, GPIOJ,  0 },
    { D26, GPIOJ, 14 }, { D27, GPIOJ,  1 }, { D28, GPIOJ, 15 }, { D29, GPIOJ,  2 },
    { D30, GPIOK,  3 }, { D31, GPIOJ,  3 }, { D32, GPIOK,  4 }, { D33, GPIOJ,  4 },
    { D34, GPIOK,  5 }, { D35, GPIOJ,  5 }, { D36, GPIOK,  6 }, { D37, GPIOJ,  6 },
    // ---- module 1 : GIGA D38..D53 ---------------------------------------
    { D38, GPIOJ,  7 }, { D39, GPIOI, 14 }, { D40, GPIOE,  6 }, { D41, GPIOK,  7 },
    { D42, GPIOI, 15 }, { D43, GPIOI, 10 }, { D44, GPIOG, 10 }, { D45, GPIOI, 13 },
    { D46, GPIOH, 15 }, { D47, GPIOB,  2 }, { D48, GPIOK,  0 }, { D49, GPIOE,  4 },
    { D50, GPIOI, 11 }, { D51, GPIOE,  5 }, { D52, GPIOK,  2 }, { D53, GPIOG,  7 },
};

// Control pins as (port, bit) for fast register access.
#define AD7606_CONVST_PORT    GPIOC
#define AD7606_CONVST_BIT     4
#define AD7606_RESET_PORT     GPIOC
#define AD7606_RESET_BIT      5
#define AD7606_RD_PORT        GPIOB
#define AD7606_RD_BIT         0
#define AD7606_CS_PORT        GPIOB
#define AD7606_CS_BIT         1
#define AD7606_BUSY0_PORT     GPIOC
#define AD7606_BUSY0_BIT      3
#define AD7606_BUSY1_PORT     GPIOC
#define AD7606_BUSY1_BIT      2

// AD7606 RESET is active-HIGH (holds the part in reset). Set to 0 if your module
// uses an active-low /RESET.
#define AD7606_RESET_ACTIVE_HIGH   1

// Timing (nanoseconds / microseconds). Values are deliberately conservative.
#define AD7606_CONVST_PULSE_NS     60    // CONVST low pulse width
#define AD7606_TACC_NS             60    // data access time after RD goes low
#define AD7606_TRD_NS              30    // RD high time between reads
#define AD7606_BUSY_TIMEOUT_US     50    // give up waiting for BUSY (safety)

// Control pins as Arduino numbers (used only for pinMode() in begin(); the hot
// path uses the (port, bit) definitions above, and every data line carries its
// own Arduino number inside AD7606_DB_LINES).
static const uint8_t AD7606_CONVST_PIN = A0;
static const uint8_t AD7606_RESET_PIN  = A1;
static const uint8_t AD7606_RD_PIN     = A2;
static const uint8_t AD7606_CS_PIN     = A3;
static const uint8_t AD7606_BUSY0_PIN  = A4;
static const uint8_t AD7606_BUSY1_PIN  = A5;

// ===========================================================================
// Event detector defaults  (tune these to your signal / detector bandwidth)
// ===========================================================================
// The first group conditions the input and fixes the timing of one detection; the
// rest are the bounds a detected event has to satisfy to count as a genuine peak
// (see event_detector.h for the ladder and for the state machine behind it).
//
// Every sample-counted knob below is tied to SORTER_SAMPLE_RATE_HZ: at 10 kSPS one
// sample is 100 us, so 10 samples = 1 ms, 50 = 5 ms, 3000 = 0.3 s.  Change the rate
// and all of them have to be re-expressed in samples.
//
// The other unit note: every *count* bound below is written for the RANGE strap this
// build is made for - +/-5 V (RANGE tied to GND on both modules, 6553.6 counts/V, see
// SIGNALGEN_ADC_PER_DAC at the bottom of this file).  The same physical bounds on a
// +/-10 V build read half these numbers, which is why the generator's injected heights
// are written at this strap too - the bench and the firmware have to agree on what a
// count is worth, or a healthy peak comes back `rejected`.
#define DET_BASELINE_ALPHA     0.001f
#define DET_NOISE_ALPHA        0.001f
#define DET_K_ON               5.0f
#define DET_K_OFF              2.0f
#define DET_K_GATE             3.0f
#define DET_SIGMA_FLOOR        1

// ---- the quiet period, and the timing of one detection --------------------
// The quiet period is how the reference (baseline + noise) is established: detection
// is disarmed while it runs, the baseline/noise are seeded from it, and from then on
// they are only maintained - never re-seeded, never stepped.  3000 samples = 0.3 s.
// DET_QUIET_SEED_POINTS of its samples seed the reference.  They are spread over the
// whole period, and no single sample and no mean decides: the baseline is their median
// and sigma is their median absolute deviation x 1.4826 (see
// EventDetector::primeFromSeed).  That pair is robust to up to 50 % contamination,
// which is what covers a spike, a dropout or one peak inside the period - but NOT the
// case where a pulse covers most of the period.  That case is real at power-up: the
// bench generator has been drawing pulses since its own sample 0 (signal_gen.cpp does
// not hold the DACs for the quiet period), so the board's first quiet period is mostly
// pulse.  The median then locks onto the pulse: measured with
// `test/test_bench_detect.exe --peaks --start live`, the reference seeds to base 11316
// with sigma 706 counts, i.e. a +3528-count trigger instead of the +104..118 a
// noise-seeded run arms with - so the first pattern cycles lose real peaks (3 of the 90
// peak spans of an 8-cycle run are never reported, and those that do come back measure
// ~113 samples narrower than the span that was drawn), until the gated EMA walks the
// reference back to the noise over several seconds.  How many are lost scales with how
// busy the waveform is: at ~20 events per pattern cycle the 3000-sample period is never
// idle on this bench, which is exactly the production shape here - anything crossing
// the sensor during the first 0.3 s after power-up.  Keep DET_QUIET_SAMPLES long enough
// for the source to be idle for all of it.  Afterwards the reference is gated
// (k_gate * sigma, capped at DET_GATE_MAX counts), which is what keeps it from taking
// a step either way - and once
// a window is open the *noise* estimate stops being refreshed at all: the gate is wider
// than the band a peak opens on (k_gate > k_off), so a faint peak's own flank is inside
// it and would otherwise feed its rise into sigma, lifting the trigger with it (measured:
// sigma 27.0 -> 38.4 counts across one 120-count rung at 600 samples, i.e. the trigger
// from +136 to +192, above that rung's own apex).  The mean keeps tracking - it is what
// follows the slow wander - while sigma is held (see EventDetector::updateBaseline).
#define DET_QUIET_SAMPLES      3000
#define DET_QUIET_SEED_POINTS  128
#define DET_GATE_MAX           320
// End of an event: this many consecutive samples back below the baseline band.
// 2 samples = 0.2 ms at 10 kSPS.
#define DET_END_DEBOUNCE       2

// ---- input conditioning ---------------------------------------------------
// Length of the running median in front of every detector (and in front of the
// shared event window in LINKED mode): 1 = off, 3 = drop a 1-sample noise spike,
// 5 = drop up to 2, 7 = drop up to 3, 9 = drop up to 4.  A spike is taller than
// any real peak and is not part of the signal; without this filter it can start an
// event, end one early, and - the visible symptom - be reported as the event's
// maximum.  Only odd values up to 9 are used.
//
// 7 is the default because the spike has to be caught AFTER the analog path: an
// impulse that the generator writes as a single sample comes back from the AD7606
// spread over two to four samples (the DAC and the converter's anti-alias filter
// do not settle inside the 100 us sample period).  A median of 2n+1 taps removes
// a run of at most n samples, so the rule is
//
//     taps >= 2 * (front-end span in samples) + 1
//
// i.e. 3 for a 1-sample spike, 5 for 2, 7 for 3 (the default), 9 for 4 - 9 is the
// ceiling, so a front end wider than four samples cannot be covered.  Measured
// with `test/test_bench_detect.exe --cycles 12 --sweep` (at the shipped bounds: rise
// >= 20, width >= 35 samples; 135 events, 219 peak spans over the *harder* waveform the
// SIGNALGEN block describes), the tap count decides two things: whether a spike that
// lands on a real peak's apex costs that event its shape guard, and - at the widest
// front end - whether the spike is caught at all.  The table is in-envelope peaks
// rejected / false accepts (`--sweep` prints "rejPeak" and "ghost"); the shipped
// combination is fe=3 with 7 taps, the one row that is clean in both columns:
//
//            fe=1      fe=2      fe=3      fe=4
//     taps=3  7 / 0     3 / 0     3 / 1     2 / 1
//     taps=5    -       2 / 0     3 / 1     3 / 1
//     taps=7    -       2 / 0     0 / 0     4 / 1
//     taps=9    -         -       1 / 0     1 / 0     (126 VALID events, both columns)
//
// (fe=1 with 3 taps is the one row that *loses* a real peak outright: miss 1, 120 VALID
// of 134 events - taps=3 is not enough median for even a 1-sample front end here.  The
// other rows reject one to four healthy peaks that the 35-sample floor counts as inside
// the envelope, which is what the table is for: that cost is the tap count's, not the
// width bound's.)
//
// So 7 taps is right for a 3-sample front end (0 rejected, 0 ghost) and 9 is the
// closest a 4-sample one comes to the same result (1 / 0) - fe=4 with only 7 taps is
// the row that pays for the missing median (4 rejected, 1 ghost).  Where the median
// *cannot* reach the spike it clips its height out
// of the measurement (the path may only move DET_MAX_SLOPE per sample) - but only up
// to the point where the impulse stops looking like an event: four samples above the
// on-threshold outlive the rise test, and then the spike is reported as a peak of its
// own, which is the `1` in the fe=4 ghost column and exactly what the tap count exists
// to prevent.  Note that the 6-cycle figures this note used to quote (24 pulses, 1
// rejected) were small enough to hide both effects.
//
// The cost is delay (taps/2 samples): every timestamp moves by that much (0.3 ms
// at 10 kSPS with 7) and no width changes, because both ends of an event move
// together.  A real peak is hundreds of samples wide, so the longer median costs its
// apex a count or two; anything up to 3 samples wide - a spike, a narrow glitch -
// disappears, which is exactly what should happen to it.
#define DET_IMPULSE_FILTER_TAPS  7

// ---- metric bounds --------------------------------------------------------
// DET_MIN_WIDTH_SAMPLES is the shortest event worth reporting: events take 3.5 ms to
// 500 ms, so anything shorter than 3.5 ms (35 samples at 10 kSPS) is reported as
// WIDTH_TOO_NARROW instead of as a peak.
//
// It is compared against the event's DURATION, never against the span the measurement
// happened to see.  An event's window opens where the trace leaves the noise band (-K_OFF
// sigma, the lower of the two levels) and closes where it returns into it, so both of its
// ends sit on the same level - but a *faint* pulse spends only part of its length above
// that level: the trace reaches the band a little way up the flanks, so the span
// under-reads the pulse.  Measured with `test/test_bench_detect.exe --cycles 12`, the
// window comes back 117 samples (1.2 ms) shorter than the pulse the generator drew, on
// average (|max| 435); a floor compared straight against the span therefore rejected real
// events as "too narrow" - and the bench's own audit could not see it, because its width
// test asked the same span the rejection was judged from.  (The audit now judges from the
// width the *model* drew - rejectionJustified() in test/test_bench_detect.cpp - and fails
// the run if a pulse drawn at or over this floor is rejected for width.)
//
// So the detector inverts the pulse's envelope to get the length back - see
// EventDetector::envelopeDuration: a raised cosine of duration W whose apex A is read
// against a level T spends W * acos(2T/A - 1) / pi above T, and inverting that at the
// level the window's ends both sit on (the same K_OFF sigma the span was measured between,
// which is what makes the two ends of the relation cancel) turns the span back into the
// pulse's length.  That corrected number is the channel's `width=` on the link, the number
// every verdict here is judged with, and what live_plot.py draws its floor band against.
// Measured over the same 12 cycles it lands within 12 samples (0.12 ms) of the drawn width
// on average (|max| 165), and no event drawn at or over this floor was judged under it.
// It is an estimate: a flat-topped event of the same span reads up to twice as long, and
// an edge DET_MAX_SLOPE had to spin out leaves the span itself long, so it can overshoot -
// which is the safe direction for a floor (a length read too long cannot lose an event).
// It also carries a sample or two of the crossings' own noise, which is why the bench
// allows the measurement its own resolution (kWidthResolve there) before calling a
// rejection a defect.
//
// The generator draws against this floor: SIGNALGEN_WIDTH_SPAN_PCT % of its peaks are
// needles down to SIGNALGEN_WIDTH_MIN (3.0 ms), i.e. deliberately below it, and those
// still come back as `rejected` records - never silently dropped.  A 3 ms event that has
// to be *reported* moves this bound, and it does not move alone: DET_MIN_RISE_SAMPLES and
// DET_MAX_SLOPE below are the other two, and lowering the rise bound is not free (see the
// note under the rise test).  Note what the three bounds are *not*: a pulse drawn inside
// all three can still be rejected on its *measured* leading edge, because a faint pulse
// reaches the noise band high up its flank and the edge the detector measures is then
// short even when the event is long (that is the DET_MIN_RISE_SAMPLES note, and the
// bench's shape clause says the same).  The bench's 4.6 ms / 2144-count needle is exactly
// that case: drawn 46 samples wide - over this floor by a wide margin, and its window
// comes back 44 - and still reported as `rejected SHORT_FOR_HEIGHT`, because its apex
// arrives 15 samples (1.5 ms) after the window opened, and a 2144-count rise in 1.5 ms is
// a step's edge, not a pulse's.
// DET_MAX_WIDTH_SAMPLES is NOT a filter but a fault guard: a signal that never comes
// back to the baseline (a stuck channel) is closed there so detection cannot hang.
// 20000 samples = 2 s, 4x the longest event expected.
#define DET_MIN_MAX_VALUE      160
#define DET_MIN_WIDTH_SAMPLES  35u
#define DET_MAX_WIDTH_SAMPLES  20000u
#define DET_MIN_AREA           0ull

// ---- the rise test, and the slope the signal may move at ------------------
// A real rise lasts: DET_MIN_RISE_SAMPLES is the number of samples a candidate has to
// survive before the excursion counts as an event at all, and DET_DIP_TOLERANCE is how
// many CONSECUTIVE samples it may dip back into the baseline band while it does.
// 20 samples = 2 ms: something that happens in the 1-2 ms range is not an event (a
// lone spike, a dropout, a glitch), and a candidate that does not survive the test is
// suppressed - no event, no rejected record, nothing on the link.
// The same number is the shape bound in classifyPeak(): the apex has to arrive at
// least that late, so a step cannot be reported as a peak.  Keep it at or below the
// shortest leading edge a *real* object produces, or real events will be rejected.
//
// It is also what lets the *bounded path* catch up with a fast edge before the end
// test may fire - see the note on DET_MAX_SLOPE below.  That is not obvious from
// here and it is expensive to discover: lowering this bound to reach a 3 ms needle is
// NOT a knob turn.  Measured over 12 cycles with `test/test_bench_detect.exe --sweep`
// (the rows at the end of the table), everything is clean at rise >= 12, while
// rise >= 10 turns 42 windows into `[truncated]` - the shared window closing in the
// middle of a rising edge, i.e. the exact defect the audit exists to catch.  The
// candidate is promoted before the slope-bounded path has climbed past the
// off-threshold, so the end test (which reads that path) fires two samples later,
// on the flank.  A shorter rise bound therefore needs the promotion/end-test
// interaction dealt with as well, not just the number changed.
#define DET_MIN_RISE_SAMPLES   20u
#define DET_DIP_TOLERANCE      3u
// ... and no value is measured - and no descent is read as "back at the baseline" -
// if the signal moved faster than this.  200 counts/sample = 2e6 counts/s at 10 kSPS:
// 5x the fastest edge a real event can have (1000 counts in 5 ms), and far below what
// one impulse does (~1500 counts in a single sample).  Set it too low and a real apex
// gets clipped (clippedSamples() counts those samples); too high and a spike is
// tracked again.
//
// This bound is a statement about the *physics* of the application (how fast the
// signal is allowed to move), so it is the one a narrow event hits first and hardest:
// a 3 ms needle that rises 6000 counts reaches its apex 15 samples after its
// crossing, i.e. 400 counts/sample - twice this - and comes back
// `rejected SHORT_FOR_HEIGHT` however healthy the peak is.  The bench waveform draws
// those needles on purpose now (`SIGNALGEN_WIDTH_MIN` 3.0 ms), and the measurement
// backs the arithmetic: the tallest needle in a 6-cycle run is *drawn* 6586 counts
// over 46 samples (4.6 ms) and the detector still rejects it - the 7-tap median blunts
// the apex to max 5492 counts and the crossing-to-apex edge is ~15 samples (the raw
// dump for that span is what `--peaks` prints), i.e. 366 counts/sample, nearly twice
// this bound.  It is the height/edge test that fires, not the width floor: the same
// needle's width - the pulse's own duration now, see the note above - is over the
// 35-sample floor, so what stops it is this bound.  The audit finds it outside the
// documented envelope (the *drawn* edge is what DET_MAX_SLOPE is written against), not as
// a lost peak.  Read against the waveform, the bound is a cap on the amplitude a given
// width may carry: a raised cosine's steepest edge is pi * A / W, so A <= 200 * W / pi,
// i.e. ~64 counts per sample of width - a 3.5 ms (35-sample) event is acceptable up to
// ~2200 counts, and the 4.6 ms / 6586-count needle above is 2.3x over it.  Whether that
// is the right physics for the application is a question for the application, not for
// this bound: raising it (or drawing the generator's needles to it) is the knob.  Raising
// it is what "we must catch a 3 ms event" ultimately asks for - and the headroom is the
// impulse, not the peak: the input median filter has already removed the spikes by then,
// and what is left to reject is a step at ~1500 counts in ONE sample.
#define DET_MAX_SLOPE          200.0f

// Saturation ("rail") guard.  A bright source drives an AD7606 input past its
// range, where the reading clips at full scale: nothing can be measured there, so
// the channel is taken out of detection and its detector re-baselined.  A channel
// is declared railed after DET_RAIL_WIDTH consecutive samples at or beyond
// +/-DET_RAIL_LEVEL counts, then held out for DET_RAIL_COOLDOWN samples.  The M7
// is told once per rail episode (SIGNAL TOO BRIGHT) so the operator can act.
#define DET_RAIL_LEVEL         32000   // |sample| >= this = saturated: 4.88 V of the
                                       // ADC's 5 V full scale (9.77 V at +/-10 V)
#define DET_RAIL_WIDTH         8       // consecutive saturated samples to declare a rail
#define DET_RAIL_COOLDOWN      1000    // samples a railed channel is held out of detection

// ===========================================================================
// Synthetic signal generator (bench test only, SORTER_SIGNAL_GEN = 1)
// ===========================================================================
// The waveform itself - a train of randomly parameterised pulses on a DC baseline
// with dither and impulse noise - lives in signal_model.h (Arduino-free,
// unit-tested, mirrored in Python by test/signal_model.py).  These values map that
// model onto the board.
//
// A NEW train is drawn at the start of every pattern cycle, so the detector keeps
// meeting a fresh combination of peak heights, peak widths and gaps instead of
// the same fixed pattern for ever.  The train is deliberately a hard one:
//
//   gap between events    uniform in [GAP_MIN, GAP_MAX] samples (GAP_MIN is short,
//                         so events are allowed to sit close together)
//   overlapping events    MERGE_PCT % of the events are placed *inside* the previous
//                         one instead of after it, so that the two add up into one
//                         two-humped event spanning MERGED_SPAN_MIN..MAX nominal
//                         widths: the event-width distribution then shows a second,
//                         wider mode (pairs are never chained into longer events)
//   peak width            Gaussian(WIDTH_MEAN, WIDTH_SD)   clipped to MIN..MAX,
//                         except WIDTH_SPAN_PCT % of the peaks, which are drawn
//                         log-uniform over [WIDTH_MIN, WIDTH_MEAN] instead.  That
//                         slice IS the narrow end of the range: WIDTH_MIN is 30
//                         samples = 3.0 ms at 10 kSPS, the shortest event the
//                         machine can be asked to measure, and a Gaussian alone
//                         never reaches down there (its tails are far too thin).
//                         It stops at WIDTH_MEAN so a single pulse is still never
//                         as wide as a pair of overlapping ones.
//   peak height           Gaussian(HEIGHT_MEAN, HEIGHT_SD) clipped to MIN..MAX,
//                         except HEIGHT_SPAN_PCT % of the peaks, which are drawn
//                         log-uniform over the whole MIN..MAX span: from an apex
//                         that only just clears K_ON*sigma up to one that drives
//                         the DAC into its clamp (a saturated, flat-topped peak)
//   small-peak slice      FRONTIER_PCT % of the peaks take their apex from the
//                         smallest one the detector's rise test can accept at the
//                         width they were drawn with, times FRONTIER_MARGIN_MIN..MAX,
//                         when that width lands in FRONTIER_WIDTH_MIN..MAX.  Height
//                         and width are drawn independently above, so a small apex
//                         is rare and usually lands where being small does not
//                         matter - the corner the bounds actually decide would never
//                         be visited (see the SIGNALGEN_FRONTIER_* block below for
//                         what that corner measures, width by width)
//   glitch width          uniform in 1..GLITCH_WIDTH_MAX samples
//   glitch height         Gaussian(GLITCH_HEIGHT_MEAN, GLITCH_HEIGHT_SD)
//   glitch share          GLITCH_PCT % of the pulses
//   dither                white noise, +/-NOISE_CODE DAC codes (NOISE_ALPHA < 1
//                         low-passes it)
//   baseline wander       the DC level the pulses sit on is itself moving: a sine
//                         of BASELINE_WAVE_CODE DAC codes peak over
//                         BASELINE_WAVE_SAMPLES samples (30 s at 10 kSPS).  It
//                         runs off a *global* sample counter, so it is continuous
//                         across the pattern-cycle boundary and across a capture.
//                         This is the baseline tracker's own test: the drift is
//                         ~300x slower than the tracker, so it has to be followed
//                         silently - no event, no shift in any reported height -
//                         while a tracker that could not follow it would report one
//                         enormous event
//   impulses              SPIKE_PCT % of the samples get a single-sample impulse of
//                         uniform height in +/-SPIKE_MAX DAC codes: the noise
//                         spikes that look exactly like peaks
//
// The glitches (rectangular, 1..6 samples) and the impulses (1 sample) are the
// deliberate false peaks, and they exercise two different parts of the ladder:
// an impulse is removed by DET_IMPULSE_FILTER_TAPS before it can do any damage,
// while a rectangular glitch is caught by the shape guards (DET_MIN_RISE_SAMPLES /
// DET_MAX_SLOPE), together with DET_MIN_MAX_VALUE / DET_MIN_WIDTH_SAMPLES.  A
// randomized train must therefore still show the detector rejecting what it must
// reject, and never reporting a spike as a peak's maximum.
//
// The narrow slice drives the same ladder from the other end: a 3 ms needle is only
// ~15 samples from its crossing to its apex, so DET_MIN_RISE_SAMPLES and
// DET_MIN_WIDTH_SAMPLES decide whether it is a peak at all.  Those two bounds were
// written for "events take 3.5 ms to 500 ms" (see the DET_* block above), and a
// narrower event is exactly what hits them first: keep the requirement and the
// bounds in step, or the bench reports healthy peaks as rejected ones.  They are not
// read the same way, though: the width is the pulse's *duration* (that note above), while
// the rise is the edge the *trace* shows - so a *faint* needle, whose crossing sits high
// up its flank, can still be refused as SHORT_FOR_HEIGHT while the width it was drawn at
// is well over the floor.  That is the shape guard doing its documented job, and the
// bench's rejectionJustified() now names the bound a rejection came from so the two
// cannot be confused again.
//
// Keep these equal to the matching defaults in SignalModel::Config (signal_model.h)
// - the host unit test builds the model from those defaults.
#define SIGNALGEN_CYCLE_SAMPLES  20000u  // pattern length (2.0 s at 10 kSPS)
#define SIGNALGEN_BASELINE_CODE  2048    // DC level in DAC codes (~1.65 V)
#define SIGNALGEN_NOISE_CODE     16      // dither amplitude in DAC codes (~13 mV)
#define SIGNALGEN_NOISE_ALPHA    1.0f    // dither low-pass weight (1.0 = white)
#define SIGNALGEN_BASELINE_WAVE_CODE    150      // slow wander of that DC level:
                                                 // +/- this many DAC codes
                                                 // (~+/-792 AD counts at +/-5 V:
                                                 // 9x the dither, 3x the K_ON
                                                 // trigger - and still not a peak)
#define SIGNALGEN_BASELINE_WAVE_SAMPLES 300000u  // ... one full sine over this many
                                                 // samples: 30 s at 10 kSPS
#define SIGNALGEN_SPIKE_PCT      0.05f   // % of samples carrying an impulse
#define SIGNALGEN_SPIKE_MAX      400     // impulse height, uniform in +/- DAC codes
#define SIGNALGEN_SEED           0x2F6E2B1u

#define SIGNALGEN_MAX_PULSES       48u     // never more pulses than this per cycle
#define SIGNALGEN_CYCLE_MARGIN     600u    // keep pulses clear of the cycle boundary
#define SIGNALGEN_GAP_MIN          30u     // quiet time between events (samples):
                                           // 0.3 ms, barely more than
                                           // DET_END_DEBOUNCE, so the next excursion
                                           // is already rising while the previous
                                           // window closes
#define SIGNALGEN_GAP_MAX          900u    // ... and the longest quiet stretch is 90 ms:
                                           // a cycle is packed with events end to end
                                           // instead of holding a quiet second or two
                                           // (a cycle holds ~20 events, not ~4)
#define SIGNALGEN_MERGE_PCT        80u     // % of pulses that try to fuse with the
                                           // previous one (pairs only, no chains;
                                           // a request - see signal_model.h)
#define SIGNALGEN_MERGED_SPAN_MIN  1.50f   // a merged event of two pulses spans this
#define SIGNALGEN_MERGED_SPAN_MAX  2.00f   // ..that times WIDTH_MEAN (uniform)
#define SIGNALGEN_MERGE_MIN_OVERLAP 0.12f  // the two must overlap by at least this
                                           // fraction of the first pulse's width,
                                           // or they stay two separate events
#define SIGNALGEN_WIDTH_MEAN       1200.0f // peak width (samples, 120 ms at 10 kSPS)
#define SIGNALGEN_WIDTH_SD         260.0f
#define SIGNALGEN_WIDTH_MIN        30u     // 3.0 ms: the narrowest event that still
                                           // has to be measured - its leading edge
                                           // is only ~15 samples long
#define SIGNALGEN_WIDTH_MAX        3000u
#define SIGNALGEN_WIDTH_SPAN_PCT   40u     // % of the peaks drawn log-uniform over
                                           // [WIDTH_MIN, WIDTH_MEAN] instead of the
                                           // Gaussian - the needles, in every cycle
#define SIGNALGEN_HEIGHT_MEAN      1800.0f  // nominal peak height (AD7606 counts)
#define SIGNALGEN_HEIGHT_SD        600.0f
#define SIGNALGEN_HEIGHT_SPAN_PCT  30u     // % of peaks drawn log-uniform, over the
                                           // whole MIN..MAX span, instead
#define SIGNALGEN_HEIGHT_MIN       320     // counts: ~1.3 * K_ON * sigma, i.e. an apex
                                           // that only just clears the noise band
#define SIGNALGEN_HEIGHT_MAX       40000   // counts: past the DAC clamp, so the top of
                                           // the span saturates the source
// ---- the small-peak slice: the corner the bounds decide -------------------
// The height draw above reaches down to HEIGHT_MIN, but it is drawn *independently
// of the width*, so a small apex is rare and usually lands on a pulse where being
// small does not matter.  The corner the bounds really decide - a small apex on a
// pulse wide enough for DET_MIN_RISE_SAMPLES to be the thing that decides - is
// therefore almost never visited, and a change to that bound would not show up in
// the bench at all.  This slice draws it on purpose: FRONTIER_PCT % of the peaks
// take their apex from the smallest one the detector's rise test can accept *at the
// width they were drawn with*, times FRONTIER_MARGIN_MIN..MAX (see
// signal_model.h::frontierApex).  The width itself is left exactly as drawn, so the
// width / merge / bimodality statistics of the train do not move.
//
//     apex_min(w) = FRONTIER_TRIGGER / cos^2(FRONTIER_RISE * pi / w)
//
// which over the band the slice uses (200..800 samples) is 200..266 counts, so the
// slice puts ~500..1100-count peaks - the *small* end of the train - into most cycles,
// and the run fails on any of them that comes back `rejected` (the bench's own MISSED /
// envelope checks).  It applies over the FRONTIER_WIDTH_MIN..MAX band only, which is
// the range over which the *documented* envelope is the whole story: narrower, the
// slope guard (DET_MAX_SLOPE) owns the corner and the same apex needs a far taller
// pulse; wider, it is the run's own *reference* that sets the floor: the trigger is
// base + k_on * sigma, so the run's sigma decides - together with DET_MIN_MAX_VALUE,
// since the excursion has to *reach* 160 counts to be reported at all, and at this noise
// level a drawn apex has to be about 120-140 counts for its reported max to get there.
// Measured with --ladder, 4 cycles, shipped front end: 1545 samples takes 140 counts and
// up (sigma 23.8 -> on +118) and leaves 60 and 100 "never crossed the on-threshold", while
// the same width asked for as four rungs 424..544 (sigma 20.8 -> on +104) is VALID at
// every one - window 1180..1228 samples, duration 1413..1493 of the 1545 drawn.
// At 907 samples it is the same story: four rungs 670..790 all VALID (windows 730..786,
// durations 850..912), and the 60..320 walk-up puts the floor between 120 and 140 counts.
// What a faint rung must not do is decide that sigma for itself: an excursion may not
// feed the noise estimate - EventDetector::updateBaseline() freezes it while a window is
// open - because a rung whose own flank is inside the reference gate otherwise lifts its
// trigger as it rises.  Measured at 600 samples: sigma walked 27.0 -> 38.4 counts across
// one 120-count rung, i.e. the trigger from +136 to +192 counts, above that rung's own apex,
// so it was never crossed.  Those regimes are
// real and worth knowing (see the notes under DET_MIN_RISE_SAMPLES / DET_END_DEBOUNCE
// above); the slice stays inside the band so that the run is about the detector's
// floor and not about the corner cases around it.
//
// Measured against a swept single pulse on the bench's own detector stack (+/-5 V,
// front end 3, the dither and the slow wander - walked with `--ladder`, 4 cycles: sigma
// 20.8..23.8 counts after the quiet period, so the trigger the run judges with is
// +104..118), the smallest *drawn* apex that comes back VALID is
//
//     width (samples)   30-50   60    80   100   150-1200  2400
//     smallest VALID     never   280   200   120   140-200   160
//     apex (counts)      (3-5 ms: no edge long enough at any height - the rise test)
//     what sets it       rise   rise+slope  slope  bound  bound/tracker  tracker
//
// so the slice sits ~3-6x above that floor: the smallest thing in the train, and far
// enough inside the envelope that a run is about the detector rather than the draw.
// The numbers do not fall smoothly with width, and that is the rise test's own
// resolution: the edge it measures is read on a filtered, slope-bounded path, so it
// steps (measured: 15 samples to 21 between two neighbouring amplitudes at 80 samples
// wide, and 1 to 22 at 100) - which is why 8 ms needs a *taller* pulse than 6 ms and
// why "the floor" is a staircase rather than a curve.  `test_bench_detect --peaks`
// prints both numbers for every span, so any of these can be re-measured directly - and
// `test_bench_detect --ladder` (or `live_plot.py --sim --ladder`) walks one column of
// that table as a *visible* staircase: see SIGNALGEN_LADDER_* below.
// (FRONTIER_TRIGGER is the round 200 rather than the 104..118 that was measured: the
// envelope is apex_min(w) = TRIGGER / cos^2(...), so this is the *apex* the slice has
// to clear, not the threshold it is compared against - 200 keeps the slice's peaks
// (500..1100 counts) at the bottom of the train, about twice the trigger the run
// judges with, and above DET_MIN_MAX_VALUE so that they can be reported at all.)
// The 2400-sample column is the other end of the story: a pulse that
// slow spends longer than the baseline tracker's own time constant
// (1/DET_BASELINE_ALPHA = 1000 samples) inside the reference gate, so the baseline
// follows it up and the floor rises again - a wide pulse is not automatically easy.
// (Those figures are counts above the level the trace was on, and they were re-measured
// after the reference stopped taking its sigma from an open window - see below and
// EventDetector::updateBaseline: with the old rule a faint apex fed its own rise into
// the estimate and the floors came out ~2-3x higher.)
#define SIGNALGEN_FRONTIER_PCT        50u     // % of peaks that take the small-peak
                                              // style *when their width lands in the
                                              // band below* - about a quarter of the
                                              // draws do, so ~1 peak in 8, i.e. one
                                              // or two small peaks in every cycle
#define SIGNALGEN_FRONTIER_WIDTH_MIN  200u    // the band the slice applies in: 20 ms
#define SIGNALGEN_FRONTIER_WIDTH_MAX  800u    // .. 80 ms.  Outside it the envelope
                                              // below is not the whole story:
                                              // narrower, DET_MAX_SLOPE owns the
                                              // corner; wider, the run's own
                                              // reference sets the floor - it is
                                              // k_on * sigma, plus DET_MIN_MAX_VALUE
                                              // on the reported max (measured: 1545
                                              // samples takes 140 counts and up,
                                              // 60 and 100 come back "never crossed
                                              // the on-threshold") - the slice stays
                                              // inside what the documented envelope
                                              // covers
#define SIGNALGEN_FRONTIER_TRIGGER    200     // counts: the on-threshold the apex is
                                              // judged against (measured +104..118 on
                                              // the filtered stream, +244 on the raw)
#define SIGNALGEN_FRONTIER_RISE       20u     // DET_MIN_RISE_SAMPLES: samples of
                                              // leading edge the apex has to show
#define SIGNALGEN_FRONTIER_MARGIN_MIN 2.5f    // apex = apex_min(width) x this..that
#define SIGNALGEN_FRONTIER_MARGIN_MAX 5.0f    // (uniform)
// ---- the floor ladder: *seeing* how close to the baseline a peak can be -----
// The table above is a measurement, and the random train cannot show it.  Its gaps are
// 30..900 samples (3..90 ms) against 120 ms-wide pulses and 80 % of its pulses fuse
// with a neighbour, so a small apex is normally a *shoulder* on a 1800-count slope, and
// on a plot whose vertical scale is set by the train itself (it reaches the DAC clamp:
// ~20328 counts at +/-5 V) a 600-count apex is two or three pixels - less than the
// +/-2112-count impulse spikes sharing the trace.  Measured over 8 cycles: 19 spans
// carry a pulse <= 1000 counts, and 42..1400 samples separate them from neighbours that
// are 1000..6330 counts tall.  So "how close to the baseline can an apex be and still
// be detected" cannot be *seen* on the live plot at all.
//
// SIGNALGEN_LADDER = 1 answers it.  The pattern becomes a staircase: LADDER_COUNT
// raised-cosine pulses of SIGNALGEN_LADDER_WIDTH samples, one centred in each slot of
// the cycle, with a whole (slot - width) samples of flat baseline between them (a
// raised cosine is exactly zero outside its own width), apexes stepping by
// SIGNALGEN_LADDER_STEP from SIGNALGEN_LADDER_MIN.  Nothing else is drawn - no random
// pulses, no glitches, and no impulses - so:
//
//   * the dither (+/-16 DAC codes = +/-84 counts) and the slow wander are all the
//     detector has to separate a rung from, and its on-threshold (+104..118 counts
//     measured, see the table above) sits in the *middle* of the ladder, which is what
//     makes the rungs below it the interesting ones;
//   * the live plot's own auto scale settles near the ladder instead of on the tallest
//     false peak, so the bottom rungs are a few pixels of the 168-count noise band and
//     the top ones fill a third of the pane - and the firmware's own event lines say
//     which rungs it took.  The lowest rung it accepts *is* the floor at that width.
//
//     test\test_bench_detect.exe --ladder                 the table, rung by rung
//     test\test_bench_detect.exe --ladder --peaks         ... with every span's detail
//     py -3 test\live_plot.py --sim --ladder              the same, drawn live
//     test\test_bench_detect.exe --ladder --stream | py -3 test\live_plot.py --stdin
//
// Off by default, because the random train is the bench: it is what keeps the baseline
// tracker, the bounds and the merge/split rules honest across many shapes at once.  The
// ladder is the probe you point at one question.  The floor is a staircase in *width*
// too (see the table above), and the ladder fixes only the width it draws - walk the
// other dimension with `--ladder-width 200 / 600 / 1200`.
//
// Measured with it (12 rungs 40..920 counts at 600 samples, 4 cycles per run): the first
// rung the detector takes is 120 counts - a borderline apex whose reported max (168) only
// just reaches DET_MIN_MAX_VALUE, so it comes and goes with the dither on it, which is
// the honest answer at this width rather than a single number.  The *same* 60 ms rung
// placement, refined with --ladder-step 20 from 40, puts the edge between 120 and 140
// counts (120 silent).  So "the floor at 60 ms" is a *band* of 120-140 counts and the spread
// is a borderline apex being decided by the dither on it - the same reason the table above
// is a staircase and not a curve.  The table's own figure for that column (70-100) sits
// just above the band.
//
// That bottom rung is not decided by the *trigger*, which is worth knowing before
// lowering DET_K_ON to "reach" it: re-measured with DET_K_ON = 4 (on +82 instead of the
// +104 above), the 12-rung ladder answers *identically* - the same rungs, the same
// windows, the same reported maxes, the same 3 of 6 seeds taking the 120-count rung, and
// the bench's false accepts still 0 - because the trace does cross the lower trigger
// either way.  What decides it is how much of the drawn apex is left once the run's own
// reference is subtracted (the tracker partly follows a 600-sample rung), which is why
// the edge moves with the seed and not with the trigger.
//
// Walking the width with the shipped staircase (--ladder-width, 12 rungs 40..920, step 80)
// brackets the table above from the step side: 40 samples (4 ms) takes *nothing* - 10 of
// the 12 rungs rejected, and the run says "no claim on the pinnacle" instead of failing,
// because no apex can show a 20-sample edge there; 60 (6 ms) between 360 and 440 counts;
// 80 (8 ms) between 200 and 280; 100..200 (10..20 ms) between 120 and 200; 300..1200
// (30..120 ms) between 40 and 120, i.e. rung #2 (120 counts) taken.  A width the ladder
// cannot draw at all (2400 samples into a 1666-sample slot) fails with that reason rather
// than reporting a floor it did not measure.
//
// The ladder answers in --ladder-step jumps (80 counts) and its bottom rung flips a step
// with the slot geometry, so read the *band* and not the number: the same 600-sample rung
// is taken at the shipped 12 rungs per cycle and left alone at --ladder-count 6.  A slot
// long against the baseline tracker's time constant (1/DET_BASELINE_ALPHA = 1000 samples)
// is what a *wide* rung needs, not a lower floor: at 12 rungs/cycle a 1200-sample rung
// leaves 466 samples of baseline and still reads between 40 and 120 counts (11 of 12
// VALID), while a 2400-sample rung cannot be drawn there at all and draws at
// --ladder-count 4 (5000-sample slots), reading between 120 and 200 - the table's own
// 240 ms cell.  So --ladder-count buys a wide rung the room to be drawn, and the table
// stays the finer measurement of the floor (one raised cosine, fresh quiet period).
#define SIGNALGEN_LADDER       0       // 0 = the random train, 1 = the floor ladder
#define SIGNALGEN_LADDER_WIDTH 600u    // samples (60 ms): the width every rung has,
                                       // inside the band the table above covers
#define SIGNALGEN_LADDER_MIN   40      // counts: the bottom rung's apex - well inside
                                       // the +/-84-count dither band, so the staircase
                                       // starts where a peak cannot be told from noise
#define SIGNALGEN_LADDER_STEP  80      // counts between neighbouring rungs
#define SIGNALGEN_LADDER_COUNT 12u     // rungs: 40, 120, ... 920 counts.  12 x (1666-smp
                                       // slot) = one cycle, and every rung fits with
                                       // (slot - width) = 1066 samples of baseline
                                       // around it
#define SIGNALGEN_GLITCH_PCT       40u     // % of pulses that are glitches
#define SIGNALGEN_GLITCH_WIDTH_MAX 6u      // glitch width uniform in 1..this
#define SIGNALGEN_GLITCH_HEIGHT_MEAN 3600.0f
#define SIGNALGEN_GLITCH_HEIGHT_SD   2200.0f
#define SIGNALGEN_GLITCH_HEIGHT_MIN  240
#define SIGNALGEN_GLITCH_HEIGHT_MAX  40000

// The DAC output buffer is only linear from roughly 0.2 V to VDDA-0.2 V, so keep
// the generated code inside this window (250 .. 3850 of 0 .. 4095).  This is also
// where a saturated peak flattens out: the tallest a pulse can ever measure is
// (MAX_CODE - BASELINE_CODE) * ADC_PER_DAC ~= 9515 counts at +/-5 V, whatever
// SIGNALGEN_HEIGHT_MAX asks for - the rest of the excursion is clipped away.
#define SIGNALGEN_MIN_CODE       250
#define SIGNALGEN_MAX_CODE       3850

// AD7606 counts per DAC count. Calibrate this against your module's RANGE
// jumper; the printed max= values scale with it:
//   DAC            : 4096 codes over ~3.3 V (VREF+)         = 1241 codes/V
//   AD7606 +/-10 V : 32768 counts / 10 V = 3276.8 counts/V  -> 2.64 counts/count
//   AD7606 +/-5 V  : 32768 counts /  5 V = 6553.6 counts/V  -> 5.28 counts/count
// A unipolar sensor (0..3.3 V / 0..5 V) also uses the +/-5 V setting: the code
// simply stays in the positive half (0..+32767 for 0..5 V).
//
// THIS BUILD IS THE +/-5 V STRAP (RANGE tied to GND on both modules): it is what a
// 0..5 V sensor wants, and it doubles the counts per volt.  The strap is a package
// deal - the count bounds and injected heights around it are written for it, and
// every tool that reads the bench has to be told the same thing (live_plot.py /
// plot_signals.py / analyze_capture.py --range 5).  The ABSOLUTE count constants
// are written at this strap and scale by the same factor when the range changes:
// this one, DET_MIN_MAX_VALUE (160), DET_MIN_AREA, DET_MAX_SLOPE (200
// counts/sample), DET_GATE_MAX (320) and the SIGNALGEN heights and ladder rungs
// above (they are counts above the baseline, so they move with the strap too).
// The ones measured in samples or relative to sigma (DET_MIN_WIDTH_SAMPLES,
// DET_MIN_RISE_SAMPLES, DET_K_ON/DET_K_OFF/DET_K_GATE, DET_SIGMA_FLOOR) do not -
// which is what lands the same *physical* pulse and the same bound on the same
// volts on either strap.
#define SIGNALGEN_ADC_PER_DAC    5.28f

// Busy-wait helper used for sub-microsecond AD7606 timing margins.
static inline void tinyDelayNs(uint32_t ns) {
    uint32_t n = (ns < 4u) ? 1u : (ns / 4u);   // ~4 ns per loop on M4 @ 240 MHz
    while (n--) {
        __NOP();
    }
}
