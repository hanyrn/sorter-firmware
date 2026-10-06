// config.h
//
// Board wiring and tuning constants for the M4 acquisition/detection firmware.
// Included by the .ino and by adc7606_parallel.cpp. Not used by the host test.

#pragma once

#include <Arduino.h>

// ===========================================================================
// Feature flags
// ===========================================================================

// How the 16 channels are combined into a single sample:
//   1 = mean   (sum / 16), keeps the value in the AD7606 native +/-32768 range
//   0 = sum    (raw accumulation, more resolution, larger values)
#define SORTER_COMBINE_MEAN   1

// ===========================================================================
// Bench test: DAC -> AD7606 loopback
// ===========================================================================
// SORTER_SIGNAL_GEN = 1 makes the M4 drive the GIGA's two 12-bit DAC outputs so
// a *known* synthetic waveform can be fed straight into the AD7606 inputs,
// turning the whole chain (parallel bus -> combine -> detector -> RPC -> M7
// printout) into a hardware-in-the-loop check of the peak detector.
//
//   DAC_0 / DAC = A12 = PA_4 = DAC1_OUT1  ->  AD7606 module 0 input
//   DAC_1       = A13 = PA_5 = DAC1_OUT2  ->  AD7606 module 1 input
//
// Set this to 0 for the real application: the DACs are then left untouched and
// all 16 channels feed the combined sample (see SORTER_ACTIVE_CHANNEL_MASK).
#define SORTER_SIGNAL_GEN     1

// Fixed acquisition rate. 0 = free-run at the fastest rate the loop sustains.
// 10 kSPS is the rate the detector bounds are written for (and the rate used by
// test/model_check.py); it also guarantees the AD7606 analog input settles
// between samples, so the injected waveform is not smeared by the module's
// anti-aliasing filter.
#define SORTER_SAMPLE_RATE_HZ 10000u

// Which channels are combined into the single sample per conversion:
//   bit i = channel i  (0..7 = module 0, 8..15 = module 1)
// In the real application every channel carries a signal, so combine all 16.
// In the bench loopback only the DAC-driven inputs are live, so combining just
// those keeps the amplitude intact and keeps the floating inputs from adding
// noise to the combined sample.
#if SORTER_SIGNAL_GEN
  #define SORTER_ACTIVE_CHANNEL_MASK  ((1u << 0) | (1u << 8))
#else
  #define SORTER_ACTIVE_CHANNEL_MASK  0x0000FFFFu
#endif

// ===========================================================================
// AD7606 parallel bus -- Arduino GIGA R1 WiFi digital pins
// ===========================================================================
//
// Two AD7606 modules share ONE 16-bit data bus; each module drives the bus only
// while its own CS is asserted.  Both modules are started simultaneously with a
// shared CONVST so that all 16 detectors sample the same instant.
//
// Data bus (MCU inputs):
//   DB0..DB7   -> PJ0..PJ7  (D25 D27 D29 D31 D33 D35 D37 D38)
//   DB8..DB15  -> PK0..PK7  (D48 D10 D52 D30 D32 D34 D36 D41)
// => the whole 16-bit word is captured with two register reads (see readBus()).
//
// Control lines (MCU outputs unless noted):
//   CONVST  D22 = PJ12   (shared, rising edge starts conversion on both modules)
//   RESET   D23 = PG13   (shared)
//   RD      D24 = PG12   (shared; module selected via CS)
//   CS0     D26 = PJ14   (module 0 select)
//   CS1     D28 = PJ15   (module 1 select)
//   BUSY0   D39 = PI14   (input, module 0 end-of-conversion)
//   BUSY1   D42 = PI15   (input, module 1 end-of-conversion)
//   OS0..2  tie to GND   (no oversampling = maximum sample rate)
// ---------------------------------------------------------------------------

#define AD7606_NUM_MODULES    2
#define AD7606_CH_PER_MODULE  8
#define AD7606_NUM_CHANNELS    (AD7606_NUM_MODULES * AD7606_CH_PER_MODULE)

// Data bus as two contiguous 8-bit port fields.
#define AD7606_DB_LOW_PORT    GPIOJ
#define AD7606_DB_LOW_MASK    0x00FFu   // PJ0..PJ7
#define AD7606_DB_HIGH_PORT   GPIOK
#define AD7606_DB_HIGH_MASK   0x00FFu   // PK0..PK7

// Control pins as (port, bit) for fast register access.
#define AD7606_CONVST_PORT    GPIOJ
#define AD7606_CONVST_BIT     12
#define AD7606_RESET_PORT     GPIOG
#define AD7606_RESET_BIT      13
#define AD7606_RD_PORT        GPIOG
#define AD7606_RD_BIT         12
#define AD7606_CS0_PORT       GPIOJ
#define AD7606_CS0_BIT        14
#define AD7606_CS1_PORT       GPIOJ
#define AD7606_CS1_BIT        15
#define AD7606_BUSY0_PORT     GPIOI
#define AD7606_BUSY0_BIT      14
#define AD7606_BUSY1_PORT     GPIOI
#define AD7606_BUSY1_BIT      15

// AD7606 RESET is active-HIGH (holds the part in reset). Set to 0 if your module
// uses an active-low /RESET.
#define AD7606_RESET_ACTIVE_HIGH   1

// Timing (nanoseconds / microseconds). Values are deliberately conservative.
#define AD7606_CONVST_PULSE_NS     60    // CONVST low pulse width
#define AD7606_TACC_NS             60    // data access time after RD goes low
#define AD7606_TRD_NS              30    // RD high time between reads
#define AD7606_BUSY_TIMEOUT_US     50    // give up waiting for BUSY (safety)

// Arduino digital pins (used only for pinMode()/setup; the hot path uses the
// port/bit definitions above).
static const uint8_t AD7606_DATA_PINS[AD7606_NUM_CHANNELS] = {
    D25, D27, D29, D31, D33, D35, D37, D38,   // DB0..DB7   -> PJ0..PJ7
    D48, D10, D52, D30, D32, D34, D36, D41,   // DB8..DB15  -> PK0..PK7
};
static const uint8_t AD7606_CONVST_PIN = D22;
static const uint8_t AD7606_RESET_PIN  = D23;
static const uint8_t AD7606_RD_PIN     = D24;
static const uint8_t AD7606_CS0_PIN    = D26;
static const uint8_t AD7606_CS1_PIN    = D28;
static const uint8_t AD7606_BUSY0_PIN  = D39;
static const uint8_t AD7606_BUSY1_PIN  = D42;

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
#define SIGNALGEN_ADC_PER_DAC    2.64f

// Busy-wait helper used for sub-microsecond AD7606 timing margins.
static inline void tinyDelayNs(uint32_t ns) {
    uint32_t n = (ns < 4u) ? 1u : (ns / 4u);   // ~4 ns per loop on M4 @ 240 MHz
    while (n--) {
        __NOP();
    }
}
