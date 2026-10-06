# sorter-firmware

Arduino **GIGA R1 WiFi** (STM32H747, dual-core) firmware that reads two **AD7606**
modules over their **parallel bus** (no SPI/serial protocol), combines all 16
channels into one signal, detects peak events on an adaptive baseline + standard
deviation threshold, and reports per-event metrics (max value, area, width) to
the Cortex-M7.

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

For every combined sample `x` (the mean of the 16 channels):

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

## Notes / next steps

* The M4->M7 link currently only prints the metrics on the M7. The application
  logic that consumes `PeakMetrics` is the next thing to add on the M7.
* `AD7606_NUM_MODULES`/combining assumes both modules sense the same event; the
  combined sample is the **mean** of all 16 channels (`SORTER_COMBINE_MEAN`).
* Throughput: with no oversampling the AD7606 needs ~3.5 us per conversion plus
  16 short bus reads, so the loop runs on the order of 100 ksamples/s.
