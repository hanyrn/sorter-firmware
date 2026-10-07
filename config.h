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
#define DET_BASELINE_ALPHA     0.001f
#define DET_NOISE_ALPHA        0.001f
#define DET_K_ON               5.0f
#define DET_K_OFF              2.0f
#define DET_K_GATE             3.0f
#define DET_SIGMA_FLOOR        1
#define DET_WARMUP_SAMPLES     3000
#define DET_END_DEBOUNCE       2
#define DET_MIN_MAX_VALUE      80
#define DET_MIN_WIDTH_SAMPLES  3u
#define DET_MAX_WIDTH_SAMPLES  200000u
#define DET_MIN_AREA           0ull
#define DET_MAX_SLOPE          100.0f

// Saturation ("rail") guard.  A bright source drives an AD7606 input past its
// range, where the reading clips at full scale: nothing can be measured there, so
// the channel is taken out of detection and its detector re-baselined.  A channel
// is declared railed after DET_RAIL_WIDTH consecutive samples at or beyond
// +/-DET_RAIL_LEVEL counts, then held out for DET_RAIL_COOLDOWN samples.  The M7
// is told once per rail episode (SIGNAL TOO BRIGHT) so the operator can act.
#define DET_RAIL_LEVEL         32000   // |sample| >= this = saturated (counts, +/-10 V)
#define DET_RAIL_WIDTH         8       // consecutive saturated samples to declare a rail
#define DET_RAIL_COOLDOWN      1000    // samples a railed channel is held out of detection

// ===========================================================================
// Synthetic signal generator (bench test only, SORTER_SIGNAL_GEN = 1)
// ===========================================================================
// The pattern itself (pulse timings/shapes) lives in signal_gen.cpp (kPattern);
// these values scale it into DAC codes.
#define SIGNALGEN_CYCLE_SAMPLES  20000u  // pattern length (2.0 s at 10 kSPS)
#define SIGNALGEN_BASELINE_CODE  2048    // DC level in DAC codes (~1.65 V)
#define SIGNALGEN_NOISE_CODE     6       // dither amplitude in DAC codes (~4.8 mV)
#define SIGNALGEN_NOISE_ALPHA    1.0f    // dither low-pass weight (1.0 = white)
#define SIGNALGEN_SEED           0x2F6E2B1u

// The DAC output buffer is only linear from roughly 0.2 V to VDDA-0.2 V, so keep
// the generated code inside this window (250 .. 3850 of 0 .. 4095).
#define SIGNALGEN_MIN_CODE       250
#define SIGNALGEN_MAX_CODE       3850

// AD7606 counts per DAC count. Calibrate this against your module's RANGE
// jumper; the printed max= values scale with it:
//   DAC            : 4096 codes over ~3.3 V (VREF+)         = 1241 codes/V
//   AD7606 +/-10 V : 32768 counts / 10 V = 3276.8 counts/V  -> 2.64 counts/count
//   AD7606 +/-5 V  : 32768 counts /  5 V = 6553.6 counts/V  -> 5.28 counts/count
// A unipolar sensor (0..3.3 V / 0..5 V) also uses the +/-5 V setting: the code
// simply stays in the positive half (0..+32767 for 0..5 V).  The ABSOLUTE
// detector bounds (DET_MIN_MAX_VALUE, DET_MIN_AREA, DET_MAX_SLOPE) scale by the
// same factor when the range changes; the sigma-relative ones do not.
#define SIGNALGEN_ADC_PER_DAC    2.64f

// Busy-wait helper used for sub-microsecond AD7606 timing margins.
static inline void tinyDelayNs(uint32_t ns) {
    uint32_t n = (ns < 4u) ? 1u : (ns / 4u);   // ~4 ns per loop on M4 @ 240 MHz
    while (n--) {
        __NOP();
    }
}
