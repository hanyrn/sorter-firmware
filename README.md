# sorter-firmware

Arduino **GIGA R1 WiFi** (STM32H747, dual-core) firmware that reads two **AD7606**
modules over their **parallel bus** (no SPI/serial protocol), combines the
enabled ADC channels into one signal (all 16 in the real application; see
`SORTER_ACTIVE_CHANNEL_MASK`), detects peak events on an adaptive baseline +
standard deviation threshold, and reports per-event metrics (max value, area,
width) to the Cortex-M7.

The whole acquisition + signal-processing pipeline runs on the **Cortex-M4**
(240 MHz). The **Cortex-M7** (480 MHz) simply receives the metrics for now; the
higher-level application logic is added later.

## Core split (why it looks the way it does)

The GIGA runs **one sketch on both cores**. The code path is chosen at runtime
with `RPC.cpu_id()` (`CM4_CPUID` / `CM7_CPUID`):

| Core | Responsibility |
|------|----------------|
| **M4** | Drive the AD7606 bus, read + combine 16 channels, run the event detector, push each closed event to the M7 |
| **M7** | Receive event frames over the RPC raw endpoint and hand them to the application (currently printed to Serial) |

Inter-core transport uses the built-in `RPC` library (OpenAMP/rpmsg). Metrics
travel as one fixed-size `sorter::PeakFrame` over the **raw** endpoint, because
the RPC function dispatcher cannot carry this many fields as call arguments.

## Source layout

| File | Purpose |
|------|---------|
| `sorter-firmware.ino` | Entry points (`setup`/`loop`); M4 pipeline thread + M7 sink |
| `config.h` | Pin map + detector tuning constants (Arduino side only) |
| `adc7606_parallel.{h,cpp}` | Register-level parallel-bus driver for the two modules |
| `event_detector.h` | Hardware-independent event detector (unit-testable) |
| `peak_metrics.h` | `PeakMetrics` / reason codes shared by both cores |
| `metrics_sink.h` | M4→M7 metric frame encode/parse over the RPC raw endpoint |
| `signal_gen.{h,cpp}` | Bench-test synthetic signal generator (drives the two DACs) |
| `test/model_check.py` | Python mirror of the detector + synthetic validation |
| `test/test_event_detector.cpp` | C++ unit test for the detector (host) |
| `test/run_tests.ps1` | Builds & runs the C++ unit test |

## Wiring (suggested pin map)

Two AD7606 modules share **one** 16-bit data bus. Each module drives the bus only
while its own `CS` is asserted, so they can share the data lines. A single
`CONVST` starts both modules simultaneously so all 16 detectors sample the same
instant.

**Data bus DB0–DB15 (MCU inputs)** — packed so the whole word is latched with two
register reads (`GPIOJ` low byte, `GPIOK` high byte):

| ADC line | GIGA pin | STM32 |
|---|---|---|
| DB0..DB7 | D25 D27 D29 D31 D33 D35 D37 D38 | PJ0..PJ7 |
| DB8..DB15 | D48 D10 D52 D30 D32 D34 D36 D41 | PK0..PK7 |

**Control lines**

| Signal | GIGA pin | STM32 | Notes |
|---|---|---|---|
| CONVST (A+B tied) | D22 | PJ12 | shared, starts both modules |
| RESET | D23 | PG13 | shared |
| RD | D24 | PG12 | shared; module chosen via CS |
| CS module 0 | D26 | PJ14 | |
| CS module 1 | D28 | PJ15 | |
| BUSY module 0 | D39 | PI14 | input |
| BUSY module 1 | D42 | PI15 | input |
| OS0/1/2 | — | — | tie LOW (no oversampling = max rate) |

### Hardware notes (important)

* Tie each AD7606 **VDRIVE** to **3.3 V** (or use level shifters) so the parallel
  bus is GIGA-safe. With VDRIVE = 3.3 V the module's V_IH is met by the GIGA's
  3.3 V drivers.
* Tie `OS0..OS2` low for no oversampling (maximum sample rate). Set them
  (3 GPIOs) if you later want hardware oversampling.
* `RESET` polarity is configurable: `AD7606_RESET_ACTIVE_HIGH` in `config.h`
  (default `1`). Verify against your module if a reset does not take effect.

## Detector algorithm (`event_detector.h`)

For every combined sample `x` (the mean of the enabled channels):

1. **Idle / warm-up**: keep a running estimate of the baseline (EMA mean) and the
   noise level (`sigma`, EMA of the variance). The reference is refreshed **only
   while the signal is inside the noise band** (`|x - baseline| <= k_gate*sigma`);
   this prevents a slowly rising peak from dragging the baseline/sigma along with
   it, and it is also what "freeze the baseline during an event" means in
   practice.
2. **Event start**: when `x > baseline + k_on*sigma`, latch a frozen reference
   (`baseline`, `sigma`) and begin an event.
3. **During the event**: the reference stays frozen. Track the maximum value, the
   area (running sum of `x - baseline`) and the width (samples/time).
4. **Event end**: when `x < baseline + k_off*sigma` for `end_debounce`
   consecutive samples (hysteresis, `k_off < k_on`), close the event.
5. **Validation**: accept only if it passes every bound; otherwise mark it
   rejected with a reason (`PeakRejectReason`). The key "false peak" test is the
   **shape** check: `max_value / width_samples <= max_slope`, i.e. a spike that is
   *too short for its height* is rejected.

Tuning lives in `config.h` (`DET_*` macros). The two EMA weights are deliberately
slow (`0.001`) — the tracker must be much slower than an event. If you change the
sample rate or the expected event duration, revisit `baseline_alpha`/`noise_alpha`,
`warmup_samples`, and the `DET_MIN_*` / `DET_MAX_SLOPE` bounds.

## Build & upload

Using the Arduino CLI (the GIGA core is `arduino:mbed_giga`):

```powershell
arduino-cli core install arduino:mbed_giga

# 1) main (M7) core
arduino-cli compile --fqbn arduino:mbed_giga:giga .
arduino-cli upload  --fqbn arduino:mbed_giga:giga -p <PORT> .

# 2) M4 co-processor
arduino-cli compile --fqbn "arduino:mbed_giga:giga:target_core=cm4" .
arduino-cli upload  --fqbn "arduino:mbed_giga:giga:target_core=cm4" -p <PORT> .
```

In VS Code this folder is an Arduino sketch (see `.vscode/arduino.json`); set the
board to **Arduino GIGA R1 WiFi** and build/upload once with *Target core* =
**Main Core**, then again with *Target core* = **M4 Co-processor**.

## Testing

The event detector has **no hardware dependency**, so it can be validated
without the board:

* **Python model check (no compiler needed)**
  ```powershell
  py -3 test\model_check.py
  ```
  Mirrors the C++ detector and runs a synthetic signal (baseline + noise, three
  broad peaks, two glitches); it should print `PASS`.

* **C++ unit test (needs g++/clang++)**
  ```powershell
  pwsh test\run_tests.ps1
  ```

Both check the same scenario: the three broad peaks must be reported as VALID
(max ~= 800, width ~= 1100 samples) and the two glitches rejected.

## Bench test: DAC → AD7606 loopback (hardware-in-the-loop)

The GIGA's two 12-bit DAC outputs can generate a *known* waveform and feed it
straight back into the AD7606 inputs, turning the complete chain (parallel bus →
combine → detector → RPC → M7 printout) into a hardware-in-the-loop test of the
peak detector. It is enabled by `SORTER_SIGNAL_GEN 1` in `config.h` (the current
default): the M4 then drives the DACs once per conversion — **sample-locked to the
ADC** — with the pattern in `signal_gen.cpp`, and combines only the driven
channels.

### Wiring

| GIGA | AD7606 | Notes |
|---|---|---|
| **A12** (`DAC_0`, PA_4, DAC1_OUT1) | module 0 **V1** | driven channel of module 0 |
| **A13** (`DAC_1`, PA_5, DAC1_OUT2) | module 1 **V1** | driven channel of module 1 |
| **GND** | **AGND** of both modules | mandatory common ground |
| — | all other Vx inputs | tie to AGND if convenient |

`SORTER_ACTIVE_CHANNEL_MASK` defaults to `(1u << 0) | (1u << 8)`, i.e. exactly
those two inputs; the other 14 are ignored, so they can neither dilute the
amplitude nor inject noise. `A12`/`A13` are the two DAC pins of the analog
header — check the board silkscreen before wiring.

### Levels

The DAC output is 0 … ~3.3 V single-ended, and the waveform rides on a **1.65 V DC
baseline**, so peaks are positive excursions — exactly what the detector's
"baseline + k·σ" model expects. No level shifting is needed.

| | per volt | at the 1.65 V baseline |
|---|---|---|
| DAC (4096 codes over 3.3 V) | 1241 codes/V | code 2048 |
| AD7606 ±10 V, 16-bit | 3276.8 counts/V | ≈ 5407 counts |
| AD7606 ±5 V, 16-bit | 6553.6 counts/V | ≈ 10813 counts |

* Prefer the **±5 V** RANGE jumper: it doubles the counts per volt.
* `SIGNALGEN_ADC_PER_DAC` (2.64 for ±10 V, 5.28 for ±5 V) converts injected
  AD7606 counts into DAC codes. If the printed `max=` values are off by a constant
  factor, correct this number rather than the pattern.

### Procedure

1. **Check the DAC first.** After flashing, measure **A12–GND** with a multimeter:
   it must read ≈ 1.65 V (the baseline). If it reads 0 V the DAC is not being
   driven, so fix that before suspecting the ADC.
2. **Open the Serial monitor at 115200 baud.** The M7 prints a `BENCH MODE`
   banner at boot.
3. **Check the loopback.** Every metric line carries `base=`; with the wiring in
   place it must read ≈ **5407** (±10 V) or ≈ **10813** (±5 V). A `base=` near 0
   means the DAC output is not reaching the ADC input.
4. **Watch one 2 s pattern cycle** (20 000 samples at 10 kSPS):

   | injected pulse | expected log line |
   |---|---|
   | broad peak, 800 counts, 1100 samples | `VALID`, `max≈800`, `width` a little under 1100 samples |
   | broad peak, 1200 counts, 900 samples | `VALID`, `max≈1200` |
   | broad peak, 500 counts, 1400 samples | `VALID` |
   | 4-sample spike, 900 counts | `rejected SHORT_FOR_HEIGHT` |
   | 2-sample spike, 4000 counts | `rejected WIDTH_TOO_NARROW` |
   | 20-sample bump, 60 counts | `rejected MAX_TOO_SMALL` |

   The measured width is a bit shorter than the programmed pulse because the
   raised-cosine tails fall below the detection threshold.

### Tuning with the bench test

* **`DET_K_ON` / `DET_K_OFF` / `DET_SIGMA_FLOOR`** — read `sigma=` off the log; the
  injected dither gives a real, non-zero noise floor to gate on.
* **`DET_MIN_MAX_VALUE`, `DET_MAX_SLOPE`, `DET_MIN_WIDTH_SAMPLES`** — the spikes
  exist to show false-peak rejection in action; if a spike comes back `VALID`, the
  bound that should have caught it is too loose.
* **`DET_BASELINE_ALPHA` / `DET_NOISE_ALPHA` / `DET_WARMUP_SAMPLES`** are per-sample
  and therefore depend on the sample rate: at 10 kSPS, `0.001` ≈ a 1 s time
  constant and `warmup_samples = 3000` ≈ 0.3 s.
* **Sample rate** — `SORTER_SAMPLE_RATE_HZ` (10 kSPS) sets the pacing. Do not raise
  it much without re-checking the narrow spikes: the AD7606 analog front end needs
  a few µs to settle, so faster sampling smears the 2–4 sample spikes until they no
  longer look like spikes.
* The pattern (timings, amplitudes, shapes) is the `kPattern` table at the top of
  `signal_gen.cpp`. Both DAC channels always output the same waveform.

### Turning the bench test off

Set `SORTER_SIGNAL_GEN 0` in `config.h`: the DACs are then never touched, the loop
free-runs again, and all 16 channels are combined
(`SORTER_ACTIVE_CHANNEL_MASK` = `0xFFFF`) — the production configuration.

## Notes / next steps

* The M4->M7 link currently only prints the metrics on the M7. The application
  logic that consumes `PeakMetrics` is the next thing to add on the M7.
* `AD7606_NUM_MODULES`/combining assumes both modules sense the same event; the
  combined sample is the **mean** of the channels selected by
  `SORTER_ACTIVE_CHANNEL_MASK` (`SORTER_COMBINE_MEAN`).
* The loop is paced to `SORTER_SAMPLE_RATE_HZ` (10 kSPS by default) so the
  detector runs at the rate its bounds were written for. Set it to `0` to
  free-run: with no oversampling the AD7606 needs ~3.5 us per conversion plus 16
  short bus reads, so the loop can reach on the order of 100 ksamples/s.
* Hardware-in-the-loop check of the whole chain: see
  "Bench test: DAC → AD7606 loopback" above.
