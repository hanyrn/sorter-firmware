# sorter-firmware

Arduino **GIGA R1 WiFi** (STM32H747, dual-core) firmware that reads two **AD7606**
modules over their **parallel buses** (one 16-bit bus per module, shared control
lines; no SPI/serial protocol), combines the
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
| **M4** | Drive the two AD7606 buses (shared control lines), read + combine 16 channels, run the event detector, push each closed event to the M7 |
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
| `test/verify_adc_pinmap.ps1` | Checks the AD7606 pin map (config.h vs README vs GIGA core) |

## Wiring (suggested pin map)

The two AD7606 modules use **separate 16-bit data buses** and **shared control
lines**: `CONVST`, `RESET`, `RD` and `CS` are tied together, so both modules
convert, reset and shift out their data on exactly the same edges. Each module
keeps its **own** `DB0..DB15` (2 × 16 = **32 data lines**), which is exactly what
makes the shared strobes safe: two independent buses can never contend, so no
per-module `CS` sequencing and no three-state timing is involved.

Because one `CONVST` starts both conversions and one `RD` strobe advances both
modules, channel *n* of module 0 and channel *n* of module 1 are latched in the
**same `RD` cycle** — the 16 channels are one snapshot of the same instant.

**Data buses (MCU inputs, 2 × DB0..DB15)**

| Module | ADC lines | GIGA pins | STM32 |
|---|---|---|---|
| 0 | DB0..DB7  | D22 D23 D24 D25 D26 D27 D28 D29 | PJ12 PG13 PG12 PJ0 PJ14 PJ1 PJ15 PJ2 |
| 0 | DB8..DB15 | D30 D31 D32 D33 D34 D35 D36 D37 | PK3 PJ3 PK4 PJ4 PK5 PJ5 PK6 PJ6 |
| 1 | DB0..DB7  | D38 D39 D40 D41 D42 D43 D44 D45 | PJ7 PI14 PE6 PK7 PI15 PI10 PG10 PI13 |
| 1 | DB8..DB15 | D46 D47 D48 D49 D50 D51 D52 D53 | PH15 PB2 PK0 PE4 PI11 PE5 PK2 PG7 |

The 32 lines are the GIGA digital block `D22..D53` in order: module 0 takes
`D22..D37` and module 1 takes `D38..D53`, so each module can use a single 16-way
header / ribbon cable.

**Control lines** — one GIGA pin each, wired to the *same* signal on both modules:

| Signal | GIGA pin | STM32 | Notes |
|---|---|---|---|
| CONVST (A+B tied on each module) | A0 | PC4 | shared, starts both conversions |
| RESET | A1 | PC5 | shared |
| RD | A2 | PB0 | shared; one strobe advances both modules |
| CS | A3 | PB1 | shared; asserted for the whole 8-strobe burst |
| BUSY module 0 | A4 | PC3 | input |
| BUSY module 1 | A5 | PC2 | input — **never tie to BUSY0** |
| OS0/1/2 | — | — | tie LOW (no oversampling = max rate) |

> **Changing the map.** `AD7606_DB_LINES[]` in `config.h` is the single place that
> defines all 32 data lines — one `{Arduino pin, port, bit}` entry per line — plus
> the six `AD7606_*_PIN` control pins (`CONVST`, `RESET`, `RD`, `CS`, `BUSY0`,
> `BUSY1`, on `A0..A5`). The read path does not require the data
> lines to be contiguous, so **any** free GIGA pin works; just keep the
> `{port, bit}` entry in sync with the Arduino pin number.

### The analog inputs are *not* part of these tables

The modules' **analog** inputs stay in the analog domain: they are never wired to
the GIGA. Each AD7606 is an 8-channel part, so the pair carries **16 analog
channels** in total (`V1..V8` on module 0 and `V1..V8` on module 1), each with its
AGND return:

| Side | Lines | Connects to |
|---|---|---|
| Analog | 16 × `Vx` (+ AGND returns) | the detectors / signal sources |
| Digital | 2 × 16 `DB0..DB15` + 4 shared control + 2 `BUSY` | the GIGA (**38 pins**) |

So count the GIGA side, not the ADC side: **38 MCU pins** — 32 data lines plus
`CONVST`, `RESET`, `RD`, `CS`, `BUSY0`, `BUSY1`. The 32 data lines carry **16
channels at 16 bits**: 8 channels per module (`V1..V8`), each with its own 16-bit
word, and all of them latched in the same `RD` cycles. `Adc7606Parallel::readAll()`
therefore opens one shared `CS` burst, clocks 8 `RD` pulses and fills both halves
of `channels[]` per pulse (channels `0..7` = module 0, `8..15` = module 1).

Consequences for a real build:

* Never tie the two `BUSY` lines together — they are push-pull outputs on the
  module side, so tying them shorts two drivers. Keep `BUSY0`/`BUSY1` separate;
  `readAll()` waits for both.
* Tie **CONVST A and CONVST B** together on each module, otherwise only `V1..V4`
  convert.
* `CS0`/`CS1` on the modules may be tied together and driven by A3 (as in the
  table). Tying them permanently low also works: a new conversion re-arms the
  read pointer at `V1`, so the firmware's `CS` pulse is then simply unused.
* Every module needs its **own** 16 data lines. Carrier boards with (always
  enabled) bus buffers are fine here — nothing is three-stated on the module side,
  unlike on a single shared 16-line bus.
* `CS` is assumed **active-low** (assert = drive low). If your modules invert it
  (e.g. via a `74HC138`), invert the logic in `Adc7606Parallel::readAll()`.
* The **control lines use `A0..A5`** (D76..D81): six plain GPIOs on the analog
  header, which keeps the digital block `D22..D53` free for the 32 data lines and
  makes `CONVST`/`RESET`/`RD`/`CS`/`BUSY0`/`BUSY1` quick to wire. `A6`/`A7` are
  left free (the GIGA's own ADC is unused because all 16 analog channels come
  from the AD7606s) and `A12`/`A13` stay reserved for the bench-test DACs.
* None of the 38 used pins (`D22..D53` + `A0..A5`) has an on-board peripheral
  default on the GIGA (LEDs, USB, QSPI flash, SDRAM and the radio module are all
  on other pins), and `A0..A7` are ADC1 inputs that also work as digital I/O, so
  the whole map can be driven/read as plain GPIO without fighting the mbed core.

### Hardware notes (important)

* Tie each AD7606 **VDRIVE** to **3.3 V** (or use level shifters) so the parallel
  bus is GIGA-safe. With VDRIVE = 3.3 V the module's V_IH is met by the GIGA's
  3.3 V drivers.
* Tie `OS0..OS2` low for no oversampling (maximum sample rate). Set them
  (3 GPIOs) if you later want hardware oversampling.
* Check for a **`PAR/SER`** (or `BYTE SEL`) pin: it must be tied **LOW** for the
  parallel bus this driver uses. Most breakouts already strap it on-board;
  serial-only boards will not work with `adc7606_parallel.cpp`.
* `RESET` polarity is configurable: `AD7606_RESET_ACTIVE_HIGH` in `config.h`
  (default `1`). Verify against your module if a reset does not take effect.
* **Throughput:** `readWord()` collects its 16 bits with 16 individual register
  bit reads (~0.7 µs per word on the M4), i.e. ~1.4 µs per `RD` strobe for both
  modules and ~11 µs for the whole 8-strobe burst — far below the 100 µs period
  of 10 kSPS. If you ever need >50 kSPS, move the 32 lines onto four contiguous
  8-bit port fields (one `GPIOx->IDR` byte load per field) and
  read one byte per field per strobe.

* **Port fields in this map:** module 0 spans several ports (`GPIOJ`, `GPIOG`,
  `GPIOK`), so a word is still 16 bit-reads. The only complete 8-bit field is
  `PJ0..PJ7` (D25, D27, D29, D31, D33, D35, D37 = module 0 DB3..DB15 odd bits,
  plus D38 = module 1 DB0). To go faster, remap each module's `DB0..DB15` onto
  two 8-bit port fields and load `GPIOx->IDR` once per field per `RD` strobe.

### Module silkscreen → AD7606 pin

Cheap "AD7606 8-channel" breakouts abbreviate the mode/config pins. None of these
lines is wired to the GIGA in this project — they are **static straps**, so a
different labelling does not change `config.h`:

| Module label | AD7606 pin | Type | What it does | Strap for this project |
|---|---|---|---|---|
| `RAGE` (RANGE, silkscreen clipped) | `RANGE` | input | selects the analog range of **all 8 channels at once**: low = ±5 V, high = ±10 V | tie to GND for ±5 V (preferred: doubles the counts/V) or to 3.3 V for ±10 V — **both modules must match**, then set `SIGNALGEN_ADC_PER_DAC` (5.28 / 2.64) |
| `CVA` | `CONVST A` | input, rising edge | starts the conversion of `V1..V4` | tie `CVA` and `CVB` together on each module and drive the pair from the shared `CONVST` line |
| `CVB` | `CONVST B` | input, rising edge | starts the conversion of `V5..V8` | same node as `CVA`; left floating, only half of each module's channels convert |
| `FRST` | `FRSTDATA` | **output** | goes high when the first result (`V1`) is on `DB0..DB15` and low again after the first `RD` | unused — leave open or scope it. Never drive it, and never tie the two modules' `FRST` pins together (two push-pull drivers) |
| `VO` | `VDRIVE` (many boards print `VIO`) | power input | digital-interface supply (2.3 … 5.25 V): sets the logic thresholds `V_IH`/`V_OL` of the parallel bus | tie to **3.3 V**, not 5 V — at VDRIVE = 5 V the required `V_IH` is ≈ 3.5 V and the GIGA's 3.3 V drivers are no longer guaranteed high |

> If the `VO` label is ambiguous on your board, measure it before wiring: with the
> module powered and nothing attached, an **input** pin sits near 0 V while a
> regulator **output** reads ≈ 3.3 V or 5 V (do not back-feed an output — feed
> `VDRIVE` from the GIGA's 3.3 V instead). `FRST` needs no measurement: it is an
> output on every AD7606 (check the datasheet for your exact part number).

### Analog ranges: 0–3.3 V / 0–5 V (single-supply) sensors

The AD7606 has **no unipolar range**: `RANGE` selects ±5 V (low) or ±10 V (high),
and the output is two's complement with **0 V = code 0**. A sensor that only goes
0 … +5 V therefore can never fill more than the *positive half* of the codes
(15 bits), whatever you do with the strap — that is a property of the part, not of
the wiring. In other words the penalty for using a bipolar converter on a
unipolar signal is exactly **one bit** on the ±5 V range (32768 codes for 0 … 5 V,
152.6 µV/LSB, versus 76.3 µV/LSB for a hypothetical 0 … 5 V unipolar part) and
**two bits** on ±10 V (16384 codes, 305 µV/LSB) — which is why the ±5 V strap is
the right choice for 0 … 5 V sensors.

| Sensor span | `RANGE` strap | Codes produced | counts/V | LSB |
|---|---|---|---|---|
| 0 … 5 V | **GND** (±5 V) | 0 … +32767 | 6553.6 | 152.6 µV |
| 0 … 3.3 V | **GND** (±5 V) | 0 … +21627 | 6553.6 | 152.6 µV |
| 0 … 10 V | 3.3 V (±10 V) | 0 … +32767 | 3276.8 | 305 µV |

Rule of thumb: pick the bipolar range whose magnitude matches the sensor's
maximum (0–5 V → ±5 V, 0–10 V → ±10 V). The unused sign bit costs nothing real:
the part's own noise floor on the ±5 V range is already several LSB rms.

Wiring a single-supply sensor:

* **Tie the sensor ground to the module's `AGND`** (and the GIGA's GND). A
  single-supply sensor's "0 V" is only 0 V relative to that node — a floating
  ground is the number one cause of a bogus offset on unipolar signals.
* Keep the source impedance low (≲ a few kΩ) and add a series resistor (~1 kΩ)
  plus a small cap (1–10 nF) to AGND per used channel: the 8 inputs are sampled
  simultaneously and then multiplexed, so the sample capacitors need a
  low-impedance source to settle within the 100 µs sample period.
* The analog inputs are 1 MΩ and internally protected to ±16.5 V, so a 5 V sensor
  cannot damage the ADC even if it overshoots (the code simply clips at +32767).
  The 3.3 V ceiling applies to the **digital** side (`VDRIVE`), not to the analog
  input, so 0–5 V sensors need no level shifting here.
* Unused `Vx` inputs: tie them to AGND so they read ~0 counts.

Software: nothing range-specific is baked into the driver (`readAll()` returns
signed counts and the detector only looks at deviations from its adaptive
baseline), but the **absolute** count bounds scale ×2 when moving from ±10 V to
±5 V at the same signal voltage — `DET_MIN_MAX_VALUE`, `DET_MIN_AREA`,
`DET_MAX_SLOPE` and the bench `SIGNALGEN_ADC_PER_DAC` (2.64 → 5.28). The
σ-relative ones (`DET_K_ON`/`DET_K_OFF`/`DET_K_GATE`, `DET_SIGMA_FLOOR`) are
scale-free and need no change.

If you truly need a 0–5 V signal to occupy the whole 16-bit span: a single-supply
gain stage (0–3.3 V × ~1.52 → 0–5 V, rail-to-rail op-amp on the existing 5 V
rail) fills the positive half and gives the best resolution for a 3.3 V sensor
(≈100 µV/LSB) without a negative supply. Using all 65536 codes would require the
input to swing to −5 V, hence an op-amp with a negative rail — not worth it for a
theoretical 1 LSB. A genuinely unipolar ADC is different silicon (e.g. the
software-configurable AD7606B/AD7606C variants, or TI's ADS8688 family, which is
not compatible with this driver).

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

* **Pin-map check (needs the `arduino:mbed_giga` core installed)**
  ```powershell
  pwsh test\verify_adc_pinmap.ps1
  ```
  Compares the 32 data lines and the 6 control pins in `config.h` with the GIGA
  `variant.cpp`, checks that `README.md` lists the same pins in the same order,
  and fails if a pin is used twice or carries an on-board peripheral function.

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
  free-run: with no oversampling the AD7606 needs ~3.5 us per conversion plus
  ~11 us of bit-banged bus reads, so the loop free-runs at on the order of
  50 ksamples/s — 10 kSPS leaves a wide margin.
* Hardware-in-the-loop check of the whole chain: see
  "Bench test: DAC → AD7606 loopback" above.
