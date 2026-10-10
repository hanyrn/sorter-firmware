# sorter-firmware

Arduino **GIGA R1 WiFi** (STM32H747, dual-core) firmware that reads two **AD7606**
modules over their **parallel buses** (one 16-bit bus per module, shared control
lines; no SPI/serial protocol), runs one peak detector per enabled ADC channel
(each with its own adaptive baseline + standard-deviation threshold; see
`SORTER_ACTIVE_CHANNEL_MASK`), groups the per-channel results of one event into a
single bundle — linked to a primary channel or timed per channel, see
`SORTER_METRIC_MODE` — and reports the per-event metrics (height, width, area per
channel) to the Cortex-M7.

The whole acquisition + signal-processing pipeline runs on the **Cortex-M4**
(240 MHz). The **Cortex-M7** (480 MHz) simply receives the metrics for now; the
higher-level application logic is added later.

## Core split (why it looks the way it does)

The GIGA runs **one sketch on both cores**. The code path is chosen at runtime
with `RPC.cpu_id()` (`CM4_CPUID` / `CM7_CPUID`):

| Core | Responsibility |
|------|----------------|
| **M4** | Drive the two AD7606 buses (shared control lines), read the 16 channels, run one event detector per channel, group the per-channel results into one event bundle, push each closed event to the M7 |
| **M7** | Receive event bundles over the RPC raw endpoint and hand them to the application (currently printed to Serial) |

Inter-core transport uses the built-in `RPC` library (OpenAMP/rpmsg). Each event
travels as one `sorter::BundleFrame` over the **raw** endpoint: the event window
plus one `ChannelMetrics` record per active channel (see *Metrics & inter-core
frame* below), because the RPC function dispatcher cannot carry this many fields
as call arguments.

### Flash split

Both cores are flashed through the bootloader with `dfu-util`, and the *flash
split* is what makes the two images agree on the layout. `flash.ps1` passes
`-Split 75_25` (1.5 MB M7 + 0.5 MB M4) to both passes, which sets
`CM4_BINARY_START=0x08180000` for the M7 code and `upload.address_m4=0x08180000`
for the M4 upload — the M4 image is placed exactly where the M7 boot code looks
for it when `RPC.begin()` starts the co-processor. Without an explicit split the
address is empty and the M4 upload is rejected, which is why `flash.ps1` always
does the **M7 pass first, then the M4 pass**, and why `-Split` exists.

## Source layout

| File | Purpose |
|------|---------|
| `sorter-firmware.ino` | Entry points (`setup`/`loop`); M4 pipeline thread + M7 sink |
| `config.h` | Pin map + detector/aggregator tuning constants (Arduino side only) |
| `adc7606_parallel.{h,cpp}` | Register-level parallel-bus driver for the two modules |
| `event_detector.h` | Hardware-independent single-channel event detector (unit-testable) |
| `event_aggregator.h` | Groups the per-channel detectors into one event (LINKED / PER_CHANNEL) |
| `peak_metrics.h` | `ChannelMetrics` / `EventBundle` / reason codes shared by both cores |
| `metrics_sink.h` | M4→M7 event-bundle frame encode/parse over the RPC raw endpoint |
| `signal_gen.{h,cpp}` | Bench-test synthetic signal generator (drives the two DACs) |
| `flash.ps1` | Builds + uploads both GIGA cores (M7 first, then M4) over `arduino-cli` |
| `monitor.ps1` | Opens a serial terminal on the board via `arduino-cli monitor` |
| `test/model_check.py` | Python mirror of the detector + synthetic validation (**out of date**: still the pre-rework rule set - see the file header) |
| `test/model_check_aggregator.py` | Python mirror of the aggregator (LINKED / PER_CHANNEL) (**out of date**, same reason) |
| `test/detector_audit.py` | Ground-truth audit of the detector against the generated waveform (**out of date**; `test/test_bench_detect.cpp` does this against the real C++ now) |
| `test/signal_model.py` | Python mirror of `signal_model.h` (bench waveform: pulses, merged pairs, glitches, impulses) |
| `test/plot_signals.py` | Plots the AD7606 input waveform + detector verdicts (SVG/HTML, no deps) |
| `test/live_plot.py` | Live viewer for the serial sample stream (`T` command) in the browser; `--sim` draws the bench harness's own firmware verdicts instead of a Python mirror, and `--sim-ladder` swaps the random train for the floor ladder - the isolated near-baseline staircase the pane's own scale can show |
| `test/stream_check.py` | Contract check for the harness → viewer text protocol: runs `test_bench_detect.exe --stream` (and again with `--ladder`, where the report must stay off the stream), feeds its stdout through `live_plot.py`'s own parser and fails on any line, count or window the viewer stopped understanding |
| `test/test_event_detector.cpp` | C++ unit test for the detector (host) |
| `test/test_event_aggregator.cpp` | C++ unit test for the aggregator (host) |
| `test/test_signal_model.cpp` | C++ unit test for `signal_model.h` (host) |
| `test/test_bench_detect.cpp` | Bench-loopback harness: the real detector + aggregator over the real `signal_model` waveform, scored against the model's own pulse train (widths, grouping, spike-set `max=`). `--peaks` prints the per-span inventory - what the model drew, what the firmware answered, and the reference it judged it against - `--ladder` runs the floor ladder instead (isolated pulses of one width, apexes stepping up from below the floor, printed rung by rung: the run that makes the detection floor visible and measurable), and `--start live` replays the board's own power-up, where the pulses run through the quiet period |
| `test/run_tests.ps1` | Builds & runs the C++ unit tests |
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
]]]]]]]]]]]]6ed 
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
σ-relative ones (`DET_K_ON`/`DET_K_OFF`/`DET_K_GATE`, `DET_SIGMA_FLOOR`) and the
ones counted in samples (`DET_MIN_WIDTH_SAMPLES`, `DET_MIN_RISE_SAMPLES`,
`DET_IMPULSE_FILTER_TAPS`) are scale-free and need no change.

If you truly need a 0–5 V signal to occupy the whole 16-bit span: a single-supply
gain stage (0–3.3 V × ~1.52 → 0–5 V, rail-to-rail op-amp on the existing 5 V
rail) fills the positive half and gives the best resolution for a 3.3 V sensor
(≈100 µV/LSB) without a negative supply. Using all 65536 codes would require the
input to swing to −5 V, hence an op-amp with a negative rail — not worth it for a
theoretical 1 LSB. A genuinely unipolar ADC is different silicon (e.g. the
software-configurable AD7606B/AD7606C variants, or TI's ADS8688 family, which is
not compatible with this driver).

## Detector algorithm (`event_detector.h`)

For every sample `x` of one channel (the detector runs once per channel):

0. **Input conditioning**: every sample passes through a running median of
   `DET_IMPULSE_FILTER_TAPS` samples (`ImpulseFilter`) before anything else. A
   noise impulse is taller than any real peak but only a few samples wide, so it
   must not be able to start an event, end one early, or - the symptom that shows
   up in the `max=` column - become the event's maximum. Everything the detector
   reports (the baseline/noise statistics, the thresholds, the width, the area and
   the maximum) is measured on that filtered stream, so `max_value` is the maximum
   of the underlying signal instead of the highest single sample the noise
   produced. The price is a fixed delay of `taps/2` samples (0.3 ms at 10 kSPS with
   the default 7), which shifts every timestamp by that much but no width, because
   both ends of an event move together.

   The default is 7, not 3, because the spike has to be caught **after** the analog
   path. The generator (and an EMI pickup) writes an impulse as a single sample, but
   what the AD7606 hands back is that impulse spread over two to four samples: the
   DAC and the converter's anti-alias filter do not settle inside the 100 µs sample
   period. A 3-tap median lets such a spike straight through, and it then does
   exactly the damage this stage exists to prevent. Measured on the model waveform
   with a 2-4 sample front end, over 6 cycles (24 real pulses): taps=3 reported 49
   events, one with a `max=` up to +315 counts above anything the noise-free trace
   contains, and taps=5 still leaked a 3-sample spike. taps=7 removes them all at 1,
   2, 3 and 4 front-end taps, and taps=9 measures the same.
1. **Quiet period**: detection is disarmed and the reference is *established* from
   what the signal does here: `DET_QUIET_SAMPLES` samples (0.3 s at 10 kSPS), of
   which `DET_QUIET_SEED_POINTS` are taken spread over the whole period. The mean is
   their median and `sigma` is their MAD (`median |x - median|` x 1.4826), so a
   single outlier - or a pulse that lands inside the quiet period - cannot decide
   either estimate (a median/MAD pair tolerates up to 50% contamination; the
   `(p84 - p16) / 2` it replaced broke down at 16%, and a sigma that comes out too
   large puts the trigger *above* every real peak, i.e. the detector goes blind).
   The source has to be idle for the period: that is the operator's side of the
   bargain, and it is why the input should settle before the board starts.
2. **Idle**: the reference is only ever *maintained* now - a slow EMA of the mean and
   the variance, refreshed **only while the signal is inside the noise band**
   (`|x - baseline| <= k_gate*sigma`, the gate capped in absolute counts), and the
   *variance is not refreshed at all while a window is open* (from the sample the trace
   leaves the band until it is back inside it). The mean keeps tracking through an
   excursion - it is what follows a slow wander - but `sigma` does not, because the gate
   is wider than the band (`k_gate` > `k_off`): a faint peak's own lower flank sits inside
   the gate and would feed its own rise into the noise estimate, lifting the trigger
   (`base + k_on*sigma`) as the peak climbs until the peak stays under it. Measured at 600
   samples: `sigma` 13.5 -> 19.2 counts across one 60-count rung, i.e. the trigger from
   +68 to +96 counts, above that rung's own apex, so the rung was never crossed. It is
   never re-seeded and never takes a step either way, which is what "stable baseline"
   means in practice: a rising peak cannot drag it along, and one outlier cannot move it.
3. **Window and rise candidate**: two things happen here, on two different samples. The
   *window* opens the moment the trace leaves the noise band (`x > baseline +
   k_off*sigma`), and that sample is remembered: it is where the event starts, the span
   the width is measured over, and the span the aggregator times every other channel
   inside. A trace that comes back into the band for more than `DET_DIP_TOLERANCE`
   consecutive samples before it reaches the trigger takes the window with it - it was a
   band crossing, not an event. The *candidate* starts where the trace crosses the
   trigger (`x > baseline + k_on*sigma`): there the reference is latched (`baseline`,
   `sigma` frozen for the rest of the event) and the rise test begins - a real rise
   lasts, so the candidate has to survive `DET_MIN_RISE_SAMPLES` (2 ms) samples counted
   from the window's opening edge, and while it does it may dip back into the band for up
   to `DET_DIP_TOLERANCE` (3) **consecutive** samples. A candidate that does not survive
   is a noise event - a spike, a blip, a dropout - and it is **suppressed**: nothing is
   reported for it at all, not even a rejected line.
4. **Event**: the excursion is measured (height, area, leading edge) until the
   signal has settled back into the band (`x < baseline + k_off*sigma` for
   `end_debounce` consecutive samples, hysteresis `k_off < k_on`), and the event
   **ends on the first sample of that run** - the window is the excursion, not the
   excursion plus the debounce. Everything is final at that sample: one event, one
   report. Both ends of the window sit on the same level (`k_off*sigma`), which is also
   the level the *duration* is read back at: the verdict, the `width=` on the link and
   the number the operator reads are derived from the excursion by inverting the pulse's
   own envelope (`EventDetector::envelopeDuration`) - a faint pulse spends part of its
   length inside the band, so the excursion under-reads it (117 samples / 1.2 ms on
   average over the bench's 12-cycle run) while the floor `DET_MIN_WIDTH_SAMPLES` is a
   statement about the length. Two details of the measurement:
   * it runs on a **slope-bounded path**, not on the raw sample: the path follows
     the filtered sample but may move at most `DET_MAX_SLOPE` counts per sample, so a
     lone impulse can neither become the reported height nor end an event early, and
     the same bound judges the descent;
   * a sample where the *signal* jumped more than `DET_MAX_SLOPE` ahead of the path
     (while the path was already inside the excursion) is flagged: the reported
     height stays bounded, but an excursion that jumped is not a peak - that is what
     tells a burst of impulses apart from a pulse.
5. **Validation**: accept only if it passes every bound; otherwise report it as
   rejected with a reason (`PeakRejectReason`). The bounds are amplitude, width and
   area (`DET_MIN_MAX_VALUE` - checked once more *before* reporting, so a dither
   excursion that never reached it is suppressed rather than logged;
   `DET_MIN_WIDTH_SAMPLES` 3.5 ms; `DET_MAX_WIDTH_SAMPLES` 2 s as the stuck-signal
   guard; `DET_MIN_AREA`), followed by the **shape** guards, which judge the
   *leading edge* - the samples from the event start to its maximum: the edge must
   last at least `DET_MIN_RISE_SAMPLES` samples (a step, i.e. a rectangular glitch or
   whatever a spike leaves behind, is already at its top when it starts, while a real
   peak climbs over hundreds of samples) and must not have jumped (`DET_MAX_SLOPE`
   counts per sample).

Tuning lives in `config.h` (`DET_*` macros), and every sample-counted value there
carries its equivalent in milliseconds at the rate the loop runs at (10 kSPS). The two
EMA weights are deliberately slow (`0.001` ≈ a 1 s time constant); `DET_QUIET_SAMPLES`
is the quiet period that establishes the reference. If you change the sample rate or
the expected event duration, revisit all of them together - in particular
`DET_MIN_RISE_SAMPLES` (the rise test, 2 ms in the shipped config) has to stay below
the shortest *real* leading edge the sensor produces, or real events are rejected as
too fast.

## Event aggregation (`event_aggregator.h`)

The detector above runs **once per channel**, so one physical event yields one set
of metrics per channel. `EventAggregator` owns one `EventDetector` per channel,
feeds them all from the same sample set, and turns the per-channel results into a
single **event bundle** (`EventBundle` + one `ChannelMetrics` per active channel).
How the channels are tied together is chosen in `config.h`:

| `SORTER_METRIC_MODE` | Behaviour |
|---|---|
| `1` (linked, default) | The **primary channel** (`SORTER_PRIMARY_CHANNEL`) times the event: the sample its trace leaves its own noise band opens the shared window and the sample it comes back into the band closes it. Every other channel's height/area is measured **inside that window only** (a channel that is still well inside its own excursion when the window closes - a second peak rising inside it - is clipped and flagged `TRUNCATED`), and all channels report the primary's width — so one event has exactly one width. |
| `0` (per-channel) | Every channel is timed on its own (own start/end/width, so widths may differ). The event is reported once **all** channels that entered it have closed; the detector's `max_width_samples` guard closes a channel that never returns, so an event can never hang. |

In both modes the bundle carries the event window (start/end/width), the channel
that resolved the verdict (`primary`) and the mode, plus one record per channel in
`SORTER_ACTIVE_CHANNEL_MASK`. A channel that never rose is reported as absent
(`flags == 0`), so it is not mistaken for a zero-height peak.

A **suppressed** rise candidate (see the rise test above) is suppressed for the
aggregator too: in linked mode the shared window the primary's trace opened is
dropped with it, and in per-channel mode the channel leaves the open group - so a
blip can neither be reported nor hold a group open, and a group in which every
candidate was suppressed is not reported at all. The same holds for a window that never
became an event at all: a band breakout the trace dipped back out of leaves the
detector's window closed, and the aggregator's window (or the channel's place in the
group) goes with it, so the next breakout opens a fresh one.

Pick a primary (`SORTER_PRIMARY_CHANNEL`) that appears in **every** event; with the
bench loopback that is channel 8 (the module-1 DAC input). Use `0` for per-channel
mode (the primary is irrelevant there).

## Saturation ("rail") guard and channel polarity

Two input-condition concerns are handled in `EventAggregator::update()` **before**
the per-channel metrics are assembled:

* **Saturation (rail).** A source that is *too bright* drives an AD7606 input past
  its range, where the reading clips at full scale and nothing can be measured.
  Whenever `|x| >= DET_RAIL_LEVEL` for `DET_RAIL_WIDTH` consecutive samples, the
  channel is declared **railed**: it is taken out of detection (its detector is fed
  no more samples) and the M4 sends one **rail bundle** for the episode —
  `bundle.reason = REASON_RAILED`, `bundle.valid = 0`, and the railed channel(s)
  carry `CH_FLAG_RAILED`. The M7 prints `*** SIGNAL TOO BRIGHT ***`, the operator's
  cue to reduce the source intensity. The channel stays held while the input remains
  bright; once the signal returns to range the stale reference is discarded and the
  detector starts a **fresh quiet period** (`DET_QUIET_SAMPLES`) instead of trusting
  the one sample that follows the episode — the level is re-established, not guessed,
  so detection on that channel is blind for that period. The notification is emitted
  once per rail episode (re-armed only after every channel has recovered), so a
  stuck-bright source cannot flood the link. Tune with `DET_RAIL_LEVEL` /
  `DET_RAIL_WIDTH` / `DET_RAIL_COOLDOWN`.

* **Channel polarity.** Some channels idle HIGH and *dip* when an event occurs — in
  the real application the primary (9th) channel, index 8, is the complement of the
  other eight. `SORTER_INVERT_CHANNEL_MASK` (bit i = channel i) negates those
  channels right after the ADC read (see `applyInputPolarity()`), so the same
  rising-edge detector fits every channel; the inverted channel's reported height
  and area then read as the **dip depth**. Set a bit for any channel that behaves
  this way, clear it for the rest. (In the bench loopback both DACs share one
  positive waveform, so the mask is `0` there.)

## Metrics & inter-core frame (`peak_metrics.h`, `metrics_sink.h`)

`metrics_sink.h` ships one closed event per raw RPC frame from the M4 to the M7:

```
uint16 magic | uint8 version | uint8 pad
EventBundle   bundle                    (fixed header)
ChannelMetrics ch[bundle.n_channels]    (one record per active channel)
```

`bundle.n_channels` fixes the payload length, so a 9-channel event fits comfortably
and even the worst case (all 16 channels) stays inside the 512-byte RPMsg buffer.
The M7 side validates magic/version/length in `parseBundle()` and rejects anything
it does not understand, then hands the record to the application (for now it is
printed to Serial; see `printBundle()` in `sorter-firmware.ino`).

## Build & upload

Using the Arduino CLI (the GIGA core is `arduino:mbed_giga`):

```powershell
arduino-cli core install arduino:mbed_giga

# 1) main (M7) core
arduino-cli compile --fqbn "arduino:mbed_giga:giga:split=75_25" .
arduino-cli upload  --fqbn "arduino:mbed_giga:giga:split=75_25" -p <PORT> .

# 2) M4 co-processor
arduino-cli compile --fqbn "arduino:mbed_giga:giga:target_core=cm4,split=75_25" .
arduino-cli upload  --fqbn "arduino:mbed_giga:giga:target_core=cm4,split=75_25" -p <PORT> .
```

In VS Code this folder is an Arduino sketch (see `.vscode/arduino.json`); set the
board to **Arduino GIGA R1 WiFi** and build/upload once with *Target core* =
**Main Core**, then again with *Target core* = **M4 Co-processor**. Set *Flash
split* to **1.5MB M7 + 0.5MB M4** for both (or just run `flash.ps1`, which passes
`-Split 75_25`) — without a split the M4 upload has no address.

### First-time setup (fresh board)

Plug the USB-C cable into the **programming port** (the one next to the DC
jack): the board enumerates as `USB Serial Device (COMx)` and `arduino-cli`
identifies it as `Arduino Giga R1` (`arduino:mbed_giga:giga`). The second USB-C
port is the *native* port used by `USBHost`/`USBDevice` sketches and does **not**
appear as a COM port.

Flashing on Windows additionally needs the Arduino **WinUSB driver** for the
bootloader DFU interface (`USB\VID_2341&PID_0366&MI_00`), because `dfu-util`
reaches that interface through libusb. The Arduino IDE installs it automatically;
with `arduino-cli` alone it has to be installed once, as administrator:

```powershell
$core = "$env:LOCALAPPDATA\Arduino15\packages\arduino\hardware\mbed_giga\4.6.0"
Start-Process "$core\post_install.bat" -Verb RunAs    # dpinst + drivers\giga.inf
```

Without it the upload fails with `No DFU capable USB device available`.

1. **Core** (once per machine):
   ```powershell
   arduino-cli core install arduino:mbed_giga
   ```
2. **Find the port** with the board plugged in:
   ```powershell
   arduino-cli board list      # -> COMx  Arduino Giga R1  arduino:mbed_giga:giga
   ```
3. **Flash both cores** with the helper script — it locates `arduino-cli` on
   `PATH`, or the copy bundled with the *Arduino Maker Workshop* VS Code
   extension when the CLI is not on `PATH`. Both passes state the **flash split**
   explicitly (`-Split`, default `75_25` = 1.5 MB M7 + 0.5 MB M4): `arduino-cli`
   does not apply the `boards.txt` menu defaults, and without a split the M4
   upload address stays empty, so `dfu-util` rejects the file with
   *"Only DfuSe file version 1.1a is supported"*:
   ```powershell
   .\flash.ps1                 # M7 then M4, port auto-detected, split 75_25
   .\flash.ps1 -SkipUpload     # compile check only, no board needed
   .\flash.ps1 -Monitor        # ... and open the 115200 serial monitor
   .\flash.ps1 -Port COM7      # if auto-detection picks the wrong port
   .\flash.ps1 -Split 50_50    # 1 MB M7 + 1 MB M4 instead
   ```
4. **Verify** at **115200 baud**: the `BENCH MODE` banner at boot, then metric
   lines whose `base=` reads ~5407 (±10 V) or ~10813 (±5 V) once the
   DAC → AD7606 loopback in the next section is wired.

   Open a serial terminal with the helper script (`Ctrl+C` quits; only one
   program can hold the port, so close any other monitor first):
   ```powershell
   .\monitor.ps1                 # auto-detects the board, 115200 baud
   .\monitor.ps1 -Baud 9600 -Port COM7
   .\flash.ps1 -Monitor          # flash, then watch the banner
   ```

   In VS Code that is just the integrated terminal (`Ctrl` + backtick). For a GUI
   monitor, install Microsoft's *Serial Monitor* extension
   (`code --install-extension ms-vscode.vscode-serial-monitor`): it adds a
   **Serial Monitor** view to the Panel (`Ctrl+J`, also reachable via
   *Terminal → New Terminal*), where you select `COM6` and the baud rate.

Notes:

* If an upload fails with `No DFU capable USB device available` — or the 1200-bps
  touch is refused with `Access to the port 'COMx' is denied` — the board never
  entered the bootloader because another program is holding the port: close the
  VS Code *Serial Monitor* panel, any `.\monitor.ps1`, the Arduino IDE, PuTTY,
  ... and retry. On a machine that has never flashed the board with the Arduino
  IDE, install the DFU driver first (see *First-time setup*).
* If an upload fails and the port disappears, the board is running a sketch rather
  than the bootloader: **double-tap RESET** and upload again, selecting the
  bootloader port that appears (usually a new `COMx`).
* Flashing with no AD7606 modules attached still works — the BUSY wait has a
  timeout — but you are then only testing the toolchain and the serial link, since
  the floating data lines make the detector report noise.
* The checked-in default is the **bench build** (`SORTER_SIGNAL_GEN 1`, DACs drive
  channels 0/8). Set it to `0` for the real application, where the 9 wired channels
  (0..8) are measured on their own.

## Testing

The detector and the aggregator have **no hardware dependency**, so both can be
validated without the board:

* **Python model checks (no compiler needed)**
  ```powershell
  py -3 test\model_check.py             # detector
  py -3 test\model_check_aggregator.py  # aggregator (LINKED + PER_CHANNEL)
  py -3 test\detector_audit.py          # detector vs the generated waveform, scored
  ```
  `model_check.py` mirrors the C++ detector and runs a synthetic signal (baseline +
  noise, three broad peaks, a flat glitch and a 1-sample one). `model_check_aggregator.py`
  mirrors the C++ aggregator and checks both modes: in LINKED every channel inherits the
  primary's window, in PER_CHANNEL each channel keeps its own width. Both print
  `PASS`. `detector_audit.py` goes further and *scores* the detector against the
  bench waveform the model itself draws: it reports the false accepts (`GHOST`), the
  missed peaks, and how far the reported `max=` drifts from the noise-free apex -
  which is how the input filter and the shape guards were tuned. Run it with
  `[cycles]` (1 cycle = 2.0 s) and read the summary table at the end.

* **Signal plot (no hardware, no third-party packages)**
  ```powershell
  py -3 test\plot_signals.py                        # DAC loopback -> test\signals.html
  py -3 test\plot_signals.py --source model --range 5
  ```
  `plot_signals.py` graphs the voltage the AD7606 will digitize - in the bench
  loopback that is the DAC output itself - and shades the detector's own verdicts
  (VALID / rejected + reason) using the same `model_check.py` mirror. It writes a
  self-contained HTML/SVG (no Python packages required) plus a text summary, and
  opens the file in your default browser when it finishes (`--no-open` to skip).
  The default `--source dac` mirrors the bench waveform (`signal_model.h`, i.e.
  `test/signal_model.py`); `--source model` plots `model_check.py`'s own detector
  cross-check signal instead. `--range` must match the AD7606 **RANGE strap**, and it
  picks the same counts-per-DAC-code factor the firmware has to be built with
  (`SIGNALGEN_ADC_PER_DAC`: 2.64 at ±10 V, 5.28 at ±5 V), so a ±5 V run reports the
  programmed peak heights instead of twice them.

* **Live sample stream (the board, or `--sim` for the firmware without one)**
  ```powershell
  py -3 test\live_plot.py --sim              # no board: the bench harness runs
  py -3 test\live_plot.py --port COM7        # the real board
  py -3 test\live_plot.py --stdin < bench.txt    # a capture, or a pipe
  ```
  `live_plot.py` sends `T1` (the monitor's stream-on command, so the M7 always
  re-prints its `STREAM` header) to switch the sample stream on, parses the
  `S,<index>,<gen>,<ch>:<count>,...` lines and
  pushes them over Server-Sent Events to a small page it serves on
  `http://127.0.0.1:8000/` - a rolling window of the expected and measured traces
  in AD7606 counts, the point rate, the newest sample index, the width floor the
  detector is judging with (`minw=` in the header: 35 samples = 3.5 ms of *duration* at
  the defaults), and a text pane with everything the board prints that is not a sample
  line (STREAM header, boot banner, every closed event with its verdict). The window is
  measured in **time**: the last
  `--window` seconds (1 by default), with a slider in the control bar to stretch or
  tighten it. Time rather than a point count is what keeps the pane readable, because
  the number of events in it then follows the window instead of piling up in a fixed
  buffer; a second is usually right (the live detector closes a few windows per
  second at most), and widening it redraws from the `--history` seconds (8 by default,
  capped at 60) each trace keeps, without waiting for new samples. Pure stdlib plus your
  browser: no `pyserial`, no npm. Ctrl+C sends `T0` so the 1000 lines/s flood stops again
  (`--keep-streaming` leaves it running).

  `--sim` runs the **firmware**, not a mirror of it. It starts
  `test/test_bench_detect.exe --stream` (built by `test/run_tests.ps1`), which drives
  the *shipped* `EventAggregator` and `EventDetector` over `signal_model.h`'s bench
  waveform and prints the board's own serial text for that run - the `STREAM on:`
  header, one `S,` line per stream point and `printBundle()`'s event lines. The viewer
  replays that text at 10 kSPS through the very parser the COM port feeds, so the
  overlay is the firmware's decision on the samples it was given, and the log pane holds
  the harness's own verdict (`14 event(s) (14 VALID), 0 missed, ...`) next to the peaks
  it is talking about; nothing is interpreted on the way through. `--sim-cycles` (4; one
  cycle = 2 s of stream), `--sim-speed` (N× real time, `0` = as fast as the host
  manages), `--sim-start live` (the board's power-up, where the pulses run through the
  detector's quiet period), `--sim-frontend 1|3`, `--hw` (two slightly mismatched
  modules), `--sim-seed`, `--sim-repeat` (loop the capture, continuing the sample
  indices) and `--firmware EXE` pass straight through. `--sim-cycles 16` is the one to
  reach for when watching the DC drift: one cycle is 2 s whereas `BASELINE_WAVE_SAMPLES`
  makes the wander 30 s long, so a 4-cycle capture shows only a slice of it (and
  `--sim-repeat` restarts the drift on every pass). A built harness is required, and
  `--sim` puts the compiler's DLL folder on the child's `PATH` the same way
  `run_tests.ps1` does, so nothing has to be on PATH to run it.

  `--sim-ladder` runs the harness's **floor ladder** instead of the random train
  (`SIGNALGEN_LADDER_*` in `config.h`): a staircase of *isolated* pulses of one width,
  apexes stepping from inside the dither band up to a few hundred counts, one rung per
  slot of the cycle, with no random pulses, no glitches and no impulses drawn at all.
  It exists because the random train cannot show a small peak: its gaps are 3-90 ms
  against 120 ms-wide pulses and 80 % of its pulses fuse with a neighbour, so a
  300-count apex is a shoulder on a 900-count slope, and the pane's own scale is set by
  the train's full range (~10 000 counts, the DAC clamp) - two or three pixels. Measured
  over 8 cycles: the 19 spans carrying a pulse under 500 counts sit 42-1400 samples from
  neighbours 500-3165 counts tall, i.e. smaller than the ±1056-count impulse spikes
  sharing the trace.  With the ladder the pane settles near 1000 counts instead - 0.37 px
  per count on the 368-px plot area - so the ±42-count dither band is ±15 px, a
  100-count rung 37 px and the 460-count pinnacle 169 px: "how close to the baseline can
  a peak be and still be detected" becomes something to *look at*, with the firmware's
  own windows drawn over the rungs it took and the rung-by-rung table (`---- floor
  ladder: ...` out of the harness's stderr) in the log pane next to them.  The same run
  as a table on its own: `test\test_bench_detect.exe --ladder`, whose
  `--ladder-width / --ladder-min / --ladder-step / --ladder-count` flags move the
  staircase and make it finer - pipe its `--stream` into `--stdin` for a width the viewer
  has no flag for.

  A `--sim` run is a snapshot of *one build*: the log pane names the build it came from
  (`bench: harness built 2026-10-09 17:01:04`), and shouts when the exe is older than
  the sources it was compiled from. Rebuilding `test_bench_detect.exe` while the viewer
  is up **replays the new capture** with the sample indices continuing, so a page still
  showing the previous waveform needs no restart - which, along with `--sim-cycles 16`,
  is the answer to "the page does not show the updated test signal". (The board path has
  the same trap in its own form: the browser can only draw the firmware that is
  actually flashed, so a `--port` page shows `signal_gen.cpp`'s waveform as of the last
  download, not as of the last edit.)

  `--stdin` reads that same text from a capture file or a pipe, which is what makes a
  saved monitor session reviewable after the fact:
  `test\test_bench_detect.exe --stream | py -3 test\live_plot.py --stdin`.
  `--window`, `--history`, `--range`, `--channels` and `--http-port` tune the view.
  `--range` picks the AD7606 strap the page's volts labels and `expected waveform`
  scale are read with (`SIGNALGEN_ADC_PER_DAC`: 2.64 at ±10 V, 5.28 at ±5 V), which is
  also where the COM-port path's counts come from; the `--sim`/`--stdin` counts come out
  of `config.h` instead, so a `--range` that disagrees with how that firmware was built
  is reported in the log pane rather than silently rescaling them. The page draws the
  traces in AD7606 counts and,
  by default, fits the vertical axis to whatever is in the window, in quantised 1-2-5
steps: it grows the moment a sample does not fit and shrinks only after a smaller range
has held for around a second, never below ~800 counts, so the trace does not jump around
the pane as tall peaks and impulses come and go and a quiet window shows noise as noise;
tick **full scale**
  to pin it to ±32768 counts instead, which is how much of the ADC you are actually
  using. In the bench loopback that is only ~10166 counts at ±10 V (~20333 at ±5 V),
  because the generator stops at the DAC's 3.1 V clamp - see *Levels* below.

  Every closed event is also drawn over the traces as the decision the detector took,
  which is what makes a buried or merged pulse readable at all: the window it claimed
  (shaded, with both edges), the width it judged, the baseline it froze for that
  window (dashed violet line) and the peak it reported on top of that baseline (dashed
  line in the verdict's colour - green accepted, orange rejected). Each window carries
  a tag, `#12 ch0 VALID  w=54smp (exc 32smp)  max=1042  base=5412`, and the log pane
  still holds the M7's own wording for the same event; tick **detector overlay** off to
  see the bare traces. All of it comes from the lines `printBundle()` already prints, so
  the board needs no change; the numbers are exactly the ones the log shows, and the tag
  names the channel they came from (a window carries one line per active channel, and the
  overlay shows the one whose crossing opened it). `w=` is the *duration* the verdict was
  judged with (the excursion widened for the pulse's own envelope, see
  `EventDetector::envelopeDuration`), while `(exc Nsmp)` is the span the measurement
  actually covered - printed only when the two differ, which is what a faint pulse does.
  Every window also carries the
  **width floor** the board judged it against: a brown dotted **band** `minw` samples
  wide, centred on the window (`DET_MIN_WIDTH_SAMPLES`; 3.5 ms of *duration* at the
  defaults, printed in the `STREAM` header, so the drawing cannot drift from the
  firmware). The duration the detector measured for that window is drawn as the dashed
  rule along the event's own baseline, and that rule is coloured by the **width question
  alone**: green while the duration cleared the band, orange only when the floor is what
  refused the window. A rule that falls short of the band is an event refused as too
  narrow; one that reaches past it is an event the width bound let through, whatever the
  verdict says - a needle refused on the *shape* bounds (`rej SHORT_FOR_HEIGHT`) keeps a
  green rule, and its tag spells the comparison out (`w=54smp (exc 46smp) >= floor 35smp`
  against `w=27smp (exc 24smp) < floor 35smp` for the one the floor did refuse). In the
  default `SORTER_METRIC_MODE = 1` (linked) every channel's `w=` is the primary's
  duration, so the band, the edges and `w=` are all the same measurement; with per-channel
  metrics a channel may be narrower than its group window, and the tag then adds
  `(exc Nsmp)`. The window's microsecond
  stamp is converted at the stream's sample rate, and a window narrower than
  `SORTER_STREAM_DECIM` samples can fall between two plotted points - the tag still
  reports what the detector measured.

  Three things the lines cannot show are read back off the *plotted* points and drawn
  in colours of their own (never one of the channel colours):

  * **a window that swallows two peaks.** The log says one `VALID` event with a wide
    window, which is exactly what a merged pair looks like. The trace says where the
    peaks were: a square on each peak and a tick down to the valley between them, in
    teal, with the tag reading `2 peaks  dip +58` (the valley in counts above the
    window's own `base=`). A merged pair is *meant* to close as one window - the
    detector cannot end it until the trace has been below `base + k_off*sigma` for
    `DET_END_DEBOUNCE` samples - so this is the overlay saying "that wide window is two
    pulses", not "the detector was wrong";
  * **a `max=` the window's own samples cannot carry.** The detector filters
    single-sample impulses out of its input (`DET_IMPULSE_FILTER_TAPS`), so a `max=`
    higher than anything the trace shows is now a signature of something else: a
    spike wider than the filter, a window whose `base=` was frozen on a drifting
    baseline, or a peak the decimated points stepped over. A blue dotted line is
    drawn at the height the trace does support - with a square on the one sample that
    reaches the max, when there is one - and the tag says how sure the reading is:
    `!max (trace 1170)` when *no* plotted sample comes near the reported max, or
    `?max (lone 637)` when a single sample does reach it. A real apex is broad, so a
    lone point is "where to look" rather than proof: a pulse sharper than the trace
    can draw looks the same;
  * **a peak the trace climbs to too fast.** A bead is a dome: the AD7606 sees its edge
    rising over milliseconds, not samples. The overlay walks back from the apex to the
    last plotted point still within 10 % of the window's baseline and converts that to
    the board's clock; if the climb is inside 1 ms the tag says `!rise <=1000us` (violet
    line along that edge) - which at the default `SORTER_STREAM_DECIM 10` is one plotted
    interval, so the note is a ceiling, not a measurement. It is the shape check the
    numbers cannot do: a `VALID` window can be the right width and height and still be a
    spike, an edge the analog front end is settling, or a bad sample - the same suspects
    the `?max` note points at.

  All three are read off the decimated points the canvas draws, so they describe what you
  can see, never replace the measured numbers in the tag. Each window is read back as
  soon as it has arrived and is on screen, so the stats line is a running count over the
  whole session - `read back 204 windows: 6 two-peak, 9 max?, 0 fast rise` - and it keeps
  windows that have since scrolled out of the trace; a window claiming `max=0` has no
  peak to judge. The one blind spot is honest: an impulse the decimation *did* land on is
  drawn as a plausible apex, so a max inflated that way can pass the `max?` check
  unnoticed (its rise then looks fast, which is why the two notes are worth reading
  together).

  A bare spike with **no** window over it is not a missed peak: the bench waveform
  deliberately injects glitches (`SIGNALGEN_SPIKE_PCT`/% of the samples carry an impulse
  up to `SIGNALGEN_SPIKE_MAX`, plus the pattern's own glitch pulses), and the detector is
  required to suppress them - `--peaks` judges each one `OK: a N-smp glitch was
  suppressed, as it must be`, and the verdict counts them as `glitch-only` spans with
  `spike-only 0` false accepts. A decimated trace draws only the glitches that happen to
  land on a plotted sample, so they show up as isolated needles of up to a few thousand
  counts with nothing over them. The windows and the glitches are read off the same
  trace: a needle *inside* a window's band is what the `max?` note points at.

* **C++ unit tests (need g++/clang++)**
  ```powershell
  pwsh test\run_tests.ps1
  ```
  Builds and runs `test_event_detector`, `test_event_aggregator`,
  `test_signal_model` and `test_bench_detect` against the real headers. If
  `g++`/`clang++` is not on `PATH`, the script also looks in the usual
  install locations (the winget WinLibs package, MSYS2, MinGW-w64) — install one with
  `winget install BrechtSanders.WinLibs.POSIX.UCRT` if needed. If PowerShell refuses
  to run the script, use `powershell -ExecutionPolicy Bypass -File test\run_tests.ps1`.
  On a machine with no compiler at all the script points at the two Python checks
  instead. Once the harness it built passes, it also runs the protocol check below.

* **Stream protocol check (needs the harness above, plus Python)**
  ```powershell
  py -3 test\stream_check.py        # 4 cycles; --cycles/--decim/--exe/--range
  ```
  The C++ tests only ever see the C++ side of the harness's `--stream` text, and
  `live_plot.py` draws nothing for a line it no longer recognises — so a reworded
  `Event #` line would just stop producing overlays, silently, which is exactly how
  the old Python mirror drifted. This runs the harness, feeds its stdout through the
  viewer's own `StreamParser`/`StreamState`, and fails if the two disagree on the
  `STREAM` header (decimation, channels, points per frame and the width floor the
  overlay draws on every window - a band of `minw` samples of *duration*, plus the rule
  each window was judged with), on the `S,` grid (one point
  per line, indices one decimation apart, the frame count that stitches out of them),
  on the printed events (one overlay each, carrying the window, width, baseline, peak
  and verdict words the firmware printed) or on the harness's own verdict on `stderr`
  (same event/VALID counts the viewer ended up holding). It prints `PASS` and exits 0,
  or lists the line and the number that stopped adding up. `test/run_tests.ps1` runs it
  for you; run it by hand after touching `printBundle()`, `--stream` or
  `live_plot.py`'s parser.

* **Pin-map check (needs the `arduino:mbed_giga` core installed)**
  ```powershell
  pwsh test\verify_adc_pinmap.ps1
  ```
  Compares the 32 data lines and the 6 control pins in `config.h` with the GIGA
  `variant.cpp`, checks that `README.md` lists the same pins in the same order,
  and fails if a pin is used twice or carries an on-board peripheral function.

The detector checks use the same scenario: the three broad peaks must be reported
as VALID (max ~= 800, width ~= 1100 samples, leading edge ~500 samples), the
4-sample glitch rejected as `SHORT_FOR_HEIGHT`, and the 1-sample glitch not
reported at all - the impulse filter removes it. Two further cases pin the fixes:
an impulse of 2000 counts landing on and next to an 800-count peak must not become
that peak's `max=`, and a flat 10-sample step of 500 counts (average slope only
50 counts/sample) must be rejected.

`test_bench_detect.cpp` is the end-to-end counterpart of those hand-built
scenarios, and it is the one that found the two bugs the Python tools could not
see. It runs the *real* `EventAggregator` (one `EventDetector` per channel) over
the *real* bench waveform - `signal_model.h`'s own DAC counts - so its ground truth
is the model's pulse train, not what the detector reported:

```powershell
test\test_bench_detect.exe --cycles 6 --detail   # one run, one line per event/span
test\test_bench_detect.exe --cycles 6 --hw       # the two AD7606 modules differ
test\test_bench_detect.exe --cycles 6 --sweep    # candidate tunings side by side
test\test_bench_detect.exe --cycles 6 --peaks    # every model span vs its verdict
test\test_bench_detect.exe --cycles 6 --report   # report, but never fail the shell
test\test_bench_detect.exe --cycles 6 --min-rise 12 --min-width 40
test\test_bench_detect.exe --ladder              # the floor ladder, rung by rung
test\test_bench_detect.exe --ladder --ladder-width 200 --ladder-step 10
```

Without `--frontend` it puts the 3-sample analog front end of the bench section in
front of the converter (`--frontend 1` reproduces the ideal pair the Python tools
model: taps=3 passes *there*, which is exactly why the bug slipped through). It
scores what the other tools never did - whether an event's width is the span it was
injected as, whether a span came back as one event or two (and, when it was split,
whether the noise-free trace really did dip back into the band for
`DET_END_DEBOUNCE` samples), whether a `max=` is more than noise above the
noise-free trace under that window, whether a VALID event covers a glitch or an
impulse, and whether a channel is flagged `[truncated]` while its waveform is still
coming back - and exits non-zero if any of them is wrong, which is why
`run_tests.ps1` includes it.

## Bench test: DAC → AD7606 loopback (hardware-in-the-loop)

The GIGA's two 12-bit DAC outputs can generate a *known* waveform and feed it
straight back into the AD7606 inputs, turning the complete chain (parallel bus →
per-channel detector → aggregation → RPC → M7 printout) into a hardware-in-the-
loop test of the peak detector. It is enabled by `SORTER_SIGNAL_GEN 1` in
`config.h` (the current default): the M4 then drives the DACs once per conversion
— **sample-locked to the ADC** — with the pattern in `signal_gen.cpp`, and
measures only the driven channels.

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

* Prefer the **±5 V** RANGE jumper: it doubles the counts per volt. That strap is
  also what a 0…5 V sensor wants (5 V then reads 32768 counts), but it is a *package
  deal*: build with `SIGNALGEN_ADC_PER_DAC = 5.28` **and** pass `--range 5` to
  `live_plot.py` / `plot_signals.py`, or the printed `max=` values and the plotted
  full scale describe a different strap than the hardware is in.
* `SIGNALGEN_ADC_PER_DAC` (2.64 for ±10 V, 5.28 for ±5 V) converts injected
  AD7606 counts into DAC codes. If the printed `max=` values are off by a constant
  factor, correct this number rather than the pattern.
* **The loopback cannot saturate the ADC**: the most the DAC can produce is its own
  clamp, `(SIGNALGEN_MAX_CODE - SIGNALGEN_BASELINE_CODE) * SIGNALGEN_ADC_PER_DAC` ≈
  **4757 counts** above the baseline at ±10 V. The tallest peaks therefore come back
  flat-topped at that value, well below the 32 000-count rail guard. With the ±5 V
  range the same clamp reaches ≈ 9515 counts, so scale the absolute
  `SIGNALGEN_HEIGHT_*` and `SIGNALGEN_GLITCH_HEIGHT_*` values by 2 as well to keep the
  same "peaks that reach noise up to saturation" behaviour.

### Procedure

1. **Check the DAC first.** After flashing, measure **A12–GND** with a multimeter:
   it must read ≈ 1.65 V (the baseline). If it reads 0 V the DAC is not being
   driven, so fix that before suspecting the ADC.
2. **Open the Serial monitor at 115200 baud.** The M7 prints a `BENCH MODE`
   banner at boot.
3. **Check the loopback.** Every metric line carries `base=`; with the wiring in
   place it must read ≈ **5407** (±10 V) or ≈ **10813** (±5 V). A `base=` near 0
   means the DAC output is not reaching the ADC input. (The generator's own
   `BASELINE_WAVE_*` drift moves that figure ±396 counts over 30 s, so anything in
   5000-5800 is the wander, not a fault - it is slow enough that no bound can
   mistake it for a peak, and `base=` should follow it rather than step.)
4. **Watch one 2 s pattern cycle** (20 000 samples at 10 kSPS). Every cycle draws a
   *new* pulse train (`SIGNALGEN_*` in `config.h`), so the log lines differ from cycle
   to cycle - that is the point. Over a couple of cycles you should see:

   | injected | expected log line |
   |---|---|
   | broad peak (raised cosine), 30-3000 samples wide | `VALID`; `max=` anywhere from `~120` counts (a pulse that only just clears the trigger - see *how small a peak can it see*, below) up to `~4757` (a peak driven into the DAC clamp, whose top is then flat). `WIDTH_SPAN_PCT` % of the peaks are needles down at `WIDTH_MIN` (3.0 ms) - see the row three below |
   | small peak on the detector's frontier (`FRONTIER_PCT`, width in `FRONTIER_WIDTH_MIN..MAX` = 20-80 ms) | `VALID`, with `max=` only `FRONTIER_MARGIN_MIN..MAX` x the smallest apex the rise test can accept at that width (~250-550 counts at ±10 V): the *small* end of the train, drawn on purpose so the floor is exercised in every run. The audit prints the coverage it saw (`small peaks: N span(s) drawn within 4x the on-threshold, M VALID`) and fails if the waveform stops presenting them |
   | two peaks that overlap (`MERGE_PCT`) | ONE `VALID` event with `width` ~1.5-2.0 nominal widths: clearly wider than any single pulse, which is what makes the event-width distribution double-peaked |
   | needle (raised cosine), `WIDTH_MIN`..~60 samples (3-6 ms) | `rejected WIDTH_TOO_NARROW` for the ones drawn under the floor, `rejected SHORT_FOR_HEIGHT` for the rest: the width bound is judged against the pulse's *duration* (`DET_MIN_WIDTH_SAMPLES`, the excursion above widened by the envelope inversion - see `EventDetector::envelopeDuration`), and `DET_MAX_SLOPE` (100 counts/sample) or the measured leading edge (`DET_MIN_RISE_SAMPLES`) is what then stops the ones the width test lets through. Either way a *record*, which is what keeps a 3 ms event visible on the link instead of silently missing (the bench's `rejectionJustified()` names the bound each rejection came from) |
   | rectangular glitch, 1-6 samples, tall | `rejected SHORT_FOR_HEIGHT`: a rectangle is at its top the moment it starts, so the leading-edge guard catches it whatever its height - and a 1-sample one never becomes an event at all |
   | single-sample impulse (`SPIKE_PCT`) | nothing at all: the impulse filter removes it before the detector can see it (before, it closed as a 2-sample event, `rejected WIDTH_TOO_NARROW`) |
   | the DC level itself drifting (`BASELINE_WAVE_CODE` codes peak, `BASELINE_WAVE_SAMPLES` samples = 30 s per cycle) | nothing at all: the `base=` in every line follows the drift - that is what the baseline tracker is for - and no event is invented, because every bound is read against that level. A `base=` that steps, or a `VALID` event with a huge `width` and a tiny `max=`, is the tracker failing to keep up |

   With `SIGNALGEN_LADDER = 1` (or the harness's `--ladder`) that whole train is
   replaced by the **floor ladder**: isolated pulses of *one* width, one per slot of the
   cycle, apexes stepping from inside the noise band up past the floor. Rungs below the
   floor are *supposed* to be missed - they are the measurement, which is why the run
   prints them rung by rung (and to *stderr* when it is streaming, so the page and the
   table are the same run) instead of scoring them like a train. See *how small a peak
   can it see*, below, and `SIGNALGEN_LADDER_*` in `config.h`.

   The last row is the filter catching the *generator's* single-sample impulse. On
   a real board the analog path spreads that same impulse over several samples (the
   bench section above), which is why `DET_IMPULSE_FILTER_TAPS` defaults to 7: a
   `max=` higher than the trace can carry, or a `rejected SHORT_FOR_HEIGHT` line
   sitting right where a pulse's rising edge should be, means the filter is still
   letting a smeared spike through - `test/test_bench_detect.exe --sweep` shows the
   lengths side by side. A `[truncated]` flag on a channel whose waveform is visibly
   still coming back up is the other signature, and it is the *window*, not the
   filter: it is the module mismatch in `test_bench_detect.cpp --hw`.

   The measured `width` is a little shorter than the programmed pulse because the
   raised-cosine tails fall below the detection threshold, and a tall peak measures
   longer than a weak one (its flanks cross the threshold sooner).

   **Watch the first cycles separately.** The quiet period that seeds the reference
   runs while the generator is already drawing pulses (it starts at its own sample 0,
   as `signal_gen.cpp` does not hold the DACs), so the board's *first* quiet period can
   be mostly pulse. `test/test_bench_detect.exe --peaks --start live` replays exactly
   that: the reference seeds to `base 5658, sigma 353 counts` - a `+1764`-count trigger
   instead of the `+52..59` a noise-seeded run arms with - and in an 8-cycle run 3 of
   the 90 peak spans are never reported while those that do come back measure ~113
   samples narrower than the span that was drawn, until the gated EMA walks the
   reference back to the noise over several seconds. How many are lost scales with how
   busy the waveform is: at ~20 events per pattern cycle the 3000-sample quiet period is
   never idle on this bench. That is what "a peak went missing" looks like on the bench
   log right after a reset; the later cycles are the ones to judge the detector on (and
   `DET_QUIET_SAMPLES` in `config.h` explains the fix).

### Tuning with the bench test

* **`SIGNALGEN_*`** in `config.h` shape the waveform; the model itself is
  `signal_model.h`, mirrored by `test/signal_model.py` and verified by
  `test/test_signal_model.cpp` (`py -3 test/signal_model.py` prints the statistics and
  an event-width histogram). `SIGNALGEN_LADDER = 1` replaces the whole train with the
  *floor ladder* - isolated pulses of one width stepping up from below the detector's
  floor, the pattern that makes the floor visible on the live plot and measurable rung
  by rung (see *how small a peak can it see*, above). The defaults are deliberately
  harsh - ~40 % of the pulses
  are glitches, ~14 % of the pulses fuse with the previous one into a pair,
  `HEIGHT_SPAN_PCT` % of the peaks are drawn log-uniform over the whole span, the
  dither is ±`NOISE_CODE` DAC codes and `SPIKE_PCT` % of the samples carry an impulse
  of up to ±`SPIKE_MAX` codes. `MERGE_PCT` is the *attempt* rate, not the result: a
  merge only lands when the two shapes really do overlap, so the share that fuses
  follows from the drawn widths and gaps.
* **The waveform is deliberately harder than the detector's envelope**, which is what
  the last three knobs are for: `WIDTH_SPAN_PCT` % of the peaks are needles drawn
  log-uniform down to `WIDTH_MIN` (3.0 ms), the gaps run `GAP_MIN`..`GAP_MAX`
  (0.3-90 ms) so a 2 s cycle carries ~20 events instead of ~4, and `BASELINE_WAVE_*`
  drifts the whole DC level by ±150 codes (±396 counts) over a 30 s sine. None of it is
  decoration - it is a statement of what the machine has to survive, and on the shipped
  bounds the detector does: `--cycles 6` (12 cycles in the note below) reports 0 missed,
  0 ghost, 0 spike-set `max=`, 0 `[truncated]`, 0 half-width event, 0 peak rejected
  *inside* the envelope, and tracks the drift (`sigma` ~10 counts, `base=` walking with
  the wander). What the needles expose is the envelope itself. The width the bounds are
  compared against is the pulse's *duration* - the excursion widened by inverting the
  pulse's own envelope - and over 12 cycles that number lands within 12 samples (0.12 ms)
  of the width the generator drew, on average (|max| 165), where the excursion itself (the
  span the window covers) is 117 samples short of it (|max| 435). With the window opening
  on the noise band, both of its ends sit on the same level and the relation inverts
  cleanly, so the bound is 3.5 ms / 35 samples: the tightest floor that still rejects
  every needle drawn *below* the requirement without refusing one drawn *over* it (the
  bench's own width clause fails the run if a pulse drawn at or over the floor comes back
  `WIDTH_TOO_NARROW`, with a 4-sample resolution for the measurement itself). A needle is
  still judged against three bounds at once - `DET_MIN_WIDTH_SAMPLES` (3.5 ms),
  `DET_MIN_RISE_SAMPLES` (2 ms) and `DET_MAX_SLOPE` (a 3000-count needle rises ~200
  counts/sample, twice the bound) - so one drawn below the requirement comes back as
  `rejected`, a *record* on the link rather than a silence, and the
  harness counts it as "below the documented floor" instead of as a lost peak.
  `--min-rise N` / `--min-width N` move those bounds for one run without recompiling,
  and `--sweep` lays the candidates out side by side (its own 4-cycle run, so the counts
  are comparable across rows rather than to the note above):

  | candidate (fe=3, shipped taps) | events | VALID | rejected in-envelope | below floor | `[truncated]` |
  |---|---|---|---|---|---|
  | rise ≥ 20, width ≥ 35 (**shipped**) | 42 | 40 | 0 | 2 | 0 |
  | rise ≥ 12, width ≥ 40 | 45 | 42 | 0 | 0 | 0 |
  | rise ≥ 10, width ≥ 30 | 55 | 42 | 0 | 0 | 12 |
  | rise ≥ 8, width ≥ 25 | 61 | 42 | 0 | 0 | 20 |
  | rise ≥ 6, width ≥ 20 | 63 | 42 | 0 | 0 | 24 |

  The first two rows are both clean - `rise ≥ 12` / `width ≥ 40` even recovers two
  more needles - but the moment the rise bound drops to 10 the audit fails, and not on
  a missed peak: 12 `[truncated]` windows, the shared window closing in the middle of a
  rising edge. That is the promotion/end-test coupling `config.h` describes under
  `DET_MIN_RISE_SAMPLES` (the candidate is promoted before the slope-bounded path has
  climbed past the off-threshold, so the end test fires two samples later, on the
  flank). Reaching 3 ms events means fixing that first, then re-deriving
  `DET_MAX_SLOPE`, and re-running this table - the count in the last column is what
  makes it a change to the detector and not a knob turn.
* **`DET_K_ON` / `DET_K_OFF` / `DET_SIGMA_FLOOR`** — read `sigma=` off the log; the
  dither *and* the impulses give a real, non-zero noise floor to gate on (≈25 counts
  at ±10 V with the defaults).
* **`DET_MIN_MAX_VALUE`, `DET_MIN_WIDTH_SAMPLES`, `DET_MIN_RISE_SAMPLES` /
  `DET_MAX_SLOPE`** — the bounds and the shape guards. The glitches are there to show
  false-peak rejection in action; if one comes back `VALID`, the guard that should have
  caught it is too loose. A rectangular glitch is now dealt with twice over: anything
  that does not survive `DET_MIN_RISE_SAMPLES` (2 ms) is *suppressed* before it is ever
  an event, and a longer one that reaches an apex within 2 ms of its crossing is
  reported as `rejected SHORT_FOR_HEIGHT`. `DET_MIN_MAX_VALUE` is checked once more
  just before reporting, so a dither excursion that survived the rise test but never
  reached it is suppressed too rather than filling the log with `MAX_TOO_SMALL`.
  The one measurement detail worth knowing: the height and the area come from the
  slope-bounded path, and an apex that the *signal* jumped to (more than
  `DET_MAX_SLOPE` ahead of the path) is `SHORT_FOR_HEIGHT` - that is what tells a burst
  of impulses apart from a pulse, and a spike on the *front* of a real peak costs that
  peak nothing because its own apex arrives later.
* **`DET_IMPULSE_FILTER_TAPS`** — how many samples wide a noise spike may be and
  still be removed before the detector sees it: `1` = off, `3` = a 1-sample spike
  (what `SPIKE_PCT` produces), `5` = up to 2 samples, `7` = up to 3 (the default),
  `9` = up to 4 - the maximum. Raising it costs another sample of delay and blunts
  anything that narrow - which for this signal is exactly what should happen, and
  for real EMI spikes, or for a smeared impulse after a slower sample rate, is the
  knob to reach for. The default is 7 because a spike that the DAC and the AD7606
  front end have spread over three samples is what a 3-tap median misses.
  `test/test_bench_detect.exe --sweep` prints the lengths side by side (the
  `spikeMax` column is the number of events whose `max=` is a spike, and since the
  waveform carries needles the table also sweeps `DET_MIN_RISE_SAMPLES` /
  `DET_MIN_WIDTH_SAMPLES` - the two bounds a narrow event meets first). The audit of
  record is `test/test_bench_detect.cpp` itself: `--peaks` prints the model's truth
  against the firmware's verdict span by span, and `--min-rise N` / `--min-width N`
  re-run the whole thing with a candidate bound, so no recompile is needed to ask
  "what would 3 ms events cost?" - see "Tuning with the bench test".

  One trade-off to know about: the start threshold is `5σ` **of the filtered
  stream**, and a longer median lowers that `σ`, so on a quiet channel the threshold
  falls with it. Measured on a synthetic channel whose only noise is 5 counts per
  sample: `σ` 3.5 → 2.3 counts from taps 3 → 7, i.e. the threshold drops from ~17 to
  ~12 counts. A 13-count noise bump can then start a rise candidate - and that is all
  it can do: it does not survive the rise test (2 ms) and it never reaches
  `DET_MIN_MAX_VALUE`, so it is suppressed and costs nothing on the link - not even a
  log line. Read the reference the detector actually seeded with
  `test/test_bench_detect.exe --peaks` (`EventDetector::baseline()` / `sigma()`);
  on the bench waveform `sigma` is not one number but a range - over a 12-cycle run it
  seeds at 13.3 counts when the quiet period ends (so the `k_on = 5` trigger opens at
  `+67`) and then runs from 8.3 to 25.5 counts, i.e. the trigger from `+42` to `+128` -
  and `base=` *follows* the drift instead of sitting still (5421 counts when the quiet
  period ends, 5027 at the end of 12 cycles, the `BASELINE_WAVE_*` wander tracked
  without inventing a single event). The dither is `±42` counts raw (`sigma` ~24, 27.9
  as the run reports it for the *events*), but the detector gates the *median-filtered*
  stream, so the level it triggers on sits below the raw-noise figure - which is why
  `DET_MIN_MAX_VALUE` (80 counts) and the rise test matter as much as `DET_K_ON` does
  when judging a weak peak.

  **How small a peak can it see?** Measured, not argued: the bench's own detector
  stack (aggregator + detector, the headers the M4 runs) fed *one* raised cosine at a
  time on the generator's own dither, with a fresh quiet period per run (±10 V,
  `sigma` 10.4..11.9 counts after the quiet period, trigger `+52..59`). The smallest
  apex that comes back `VALID`:

  | width (samples) | 30-50 (3-5 ms) | 60 (6 ms) | 80 (8 ms) | 100 (10 ms) | 150-1200 (15-120 ms) | 2400 (240 ms) |
  |---|---|---|---|---|---|---|
  | smallest VALID apex (counts) | never | 140 | 100 | 60 | 70-100 | 80 |
  | same, at ±10 V (mV) | - | 43 | 31 | 19 | 22-31 | 25 |
  | what sets the limit | the rise test: no edge can last 20 samples | rise test + `DET_MAX_SLOPE` | `DET_MAX_SLOPE` | `DET_MIN_MAX_VALUE` on the reported max | the same, plus the tracker at the wide end | the baseline tracker |

  Read it as the *shape* of the floor, because that is what the bounds are:

  * the first check is the **trigger**: a pulse has to cross `base + k_on*sigma`
    (measured `+52..59` counts here; `+122` against the *raw* dither sigma the bench's
    ground truth uses) - but at these noise levels `DET_MIN_MAX_VALUE` (80 counts) is what
    decides the wide columns, because the excursion that crossed the trigger is only
    *reported* if its own max reaches 80 and a drawn apex has to be about 60-70 counts for
    the dither on it to get there;
  * below ~4.8 ms (48 samples) **no amplitude is enough**: a raised cosine of width
    `w` crosses a trigger `t` with an edge of `(w/2)*(1 - acos(1 - 2t/a)/pi)`
    samples, so `w <= 2*DET_MIN_RISE_SAMPLES` has no solution at any apex. Just above
    it the edge still needs a *tall* pulse - `DET_MAX_SLOPE` caps `apex / rise` - which
    is why the 4.6 ms needle at 3293 counts is `rejected SHORT_FOR_HEIGHT`;
  * a *wide* faint pulse is not automatically easier, but the reason is the
    *reference*, not the width: the trigger is a multiple of the run's own sigma.
    Measured with `--ladder`, 4 cycles, shipped front end: 1545 samples takes 70 counts
    and up (sigma 11.9 → on +59; 30 and 50 come back "never crossed the on-threshold"),
    while the same width asked for as four rungs 212..272 (sigma 10.4 → on +52) is
    `VALID` at every one - window 1180..1228 samples, duration 1413..1493 of the 1545
    drawn. At 907 samples it is the same story: four rungs 335..395 all `VALID` (windows
    730..786, durations 850..912), and the 30..160 walk-up puts the floor between 60 and
    70 counts. What a faint rung must not do is decide that sigma for itself: the
    reference stops refreshing its *noise* estimate while a window is open
    (`EventDetector::updateBaseline`), because a rung whose own flank is inside the gate
    otherwise lifts its trigger as it rises (measured at 600 samples: sigma 13.5 → 19.2
    counts across one 60-count rung, the trigger +68 → +96, above that rung's own apex).
    The tracker still takes the floor over at the very wide end (2400 samples: the
    baseline follows the pulse), and that corner is why `SIGNALGEN_FRONTIER_WIDTH_MAX`
    stops at 80 ms - known gap, not documented behaviour;
  * the numbers do **not** fall smoothly with width: the edge the detector measures is
    read on a *filtered, slope-bounded* path, so it steps by several samples near the
    boundary (measured: 15 → 21 samples between two neighbouring amplitudes at a width
    of 80, and 1 → 22 at 100). The floor is a staircase, not a curve - which is why
    8 ms needs a *taller* pulse than 6 ms. Read the per-span `--peaks` table rather
    than extrapolating a formula;
  * `test_bench_detect --peaks` prints the truth *and* the verdict for every span, so
    any of these can be re-measured on the spot: the `small peaks:` line in its report
    is how many of them a run actually contained.

  **Can I see it?** The table above is a measurement, and the random train cannot show
  it - its small apexes are shoulders on taller neighbours, on a pane scaled to the whole
  DAC range. `SIGNALGEN_LADDER = 1` (or the harness's `--ladder`, the viewer's
  `--sim-ladder`) replaces the train with the staircase the question is about: isolated
  pulses of *one* width, apexes stepping by a known amount, nothing else drawn - no
  random pulses, no glitches, no impulses, so the pane's own scale settles on the ladder
  and each rung stands on the baseline instead of in the dither. Measured with the
  shipped numbers (12 rungs 20…460 counts at 600 samples = 60 ms):

  ```powershell
  py -3 test\live_plot.py --sim --sim-ladder          # the staircase, drawn live
  test\test_bench_detect.exe --ladder                 # the same run, rung by rung
  test\test_bench_detect.exe --ladder --ladder-width 100 --ladder-step 10
  ```

  * at 600 samples the first rung the detector takes is **100 counts** (60 and 20 stay
    silent), and the edge *moves with the staircase*: refined with `--ladder-step 10` from
    20 it lands at 70 (60 silent), started at 40 it lands at 80 (70 silent). So "the floor
    at 60 ms" is a **band** of 60-100 counts, not a number - a borderline apex is decided
    by the dither that lands on it - and the table above rounds that column up to 80,
    inside the band;
  * walking the width reproduces the table's shape: 40 samples (4 ms) takes **nothing** at
    any height, 100 (10 ms) takes 140, 200 (20 ms) takes 100 - and the run says "no claim
    on the pinnacle" instead of failing when the width is one the rise test can never
    accept;
  * the ladder has one caveat of its own, and it is the tracker again: a slot has to be
    long against `1/DET_BASELINE_ALPHA` (1000 samples) or the baseline never gets back to
    the true level between two rungs and every apex measures short. A 1200-sample rung at
    12 rungs/cycle leaves 466 samples of baseline and reads a floor of 380-420 counts; the
    same rungs at `--ladder-count 4` (5000-sample slots) read 60-100, which is the table's
    own 150-1200 column. `--ladder-count` is how you give the tracker the time a wide
    rung needs; a rung wider than its own slot cannot be drawn at all, and the run says so
    (and fails) rather than reporting a floor it did not measure.

  At the widest front end modelled (`--frontend 4`, where the DAC/ADC path smears an
  impulse over four samples) a 7-tap median can no longer reach the spike. Nothing is
  measured wrongly - the spike is still clipped out of the height - but if one lands
  on a peak's *apex* the event is reported as `rejected SHORT_FOR_HEIGHT` (12 cycles:
  124 VALID of 136 events, 4 rejected) instead of being measured, and if one lands
  clear of every pulse its `+750`-count step can outlive the rise test and be reported
  as a peak of its own (1 false accept in the same run). `taps=9` clears the false
  accept and all but one of the rejections (126 VALID, 1 rejected, 0 ghost), and it
  is the one to reach for when the front end or the sample rate makes the spike wider
  than three samples. `--sweep` prints that grid; the table in `config.h`'s
  `DET_IMPULSE_FILTER_TAPS` note is the reading of it.
* **`DET_BASELINE_ALPHA` / `DET_NOISE_ALPHA` / `DET_QUIET_SAMPLES`** are per-sample and
  therefore depend on the sample rate: at 10 kSPS, `0.001` ≈ a 1 s time constant and
  `DET_QUIET_SAMPLES = 3000` ≈ 0.3 s. `DET_QUIET_SEED_POINTS` says how many of those
  samples seed the reference (spread over the whole period) and `DET_DIP_TOLERANCE`
  how long a dip inside the rise test may last.
* **Sample rate** — `SORTER_SAMPLE_RATE_HZ` (10 kSPS) sets the pacing. The detector's
  per-sample constants (`DET_BASELINE_ALPHA`, `DET_NOISE_ALPHA`, `DET_QUIET_SAMPLES`)
  and the sample-counted ones (`DET_MIN_WIDTH_SAMPLES`, `DET_MIN_RISE_SAMPLES`,
  `DET_IMPULSE_FILTER_TAPS`) all shift meaning with it, and so does the AD7606 analog
  front end: it needs a few µs to settle, so faster sampling smears an impulse over
  more samples. Raising the rate therefore means re-checking the impulse filter in
  particular - an EMI spike that is 1-2 samples at 10 kSPS could be 5 at 50 kSPS, and
  `DET_IMPULSE_FILTER_TAPS` only goes to 9.
* Both DAC channels always output the same waveform, and a new random train is drawn
  at the start of every pattern cycle.

### Turning the bench test off

Set `SORTER_SIGNAL_GEN 0` in `config.h`: the DACs are then never touched, the loop
free-runs again, and the 9 wired channels are measured on their own
(`SORTER_ACTIVE_CHANNEL_MASK` = `0x01FF`, channels 0..8) — the production
configuration. The other 7 inputs are not connected to any sensor, so they are left
out of the mask and are neither tracked nor reported.

## Notes / next steps

* The M4->M7 link currently only prints the metrics on the M7. The application
  logic that consumes the `EventRecord` bundle is the next thing to add on the M7.
* `AD7606_NUM_MODULES` assumes both modules sense the same event, so channel `n` of
  module 0 and channel `n` of module 1 come from the same instant and can be
  compared/differenced. Each channel is otherwise detected on its own — there is no
  cross-channel average.
* The loop is paced to `SORTER_SAMPLE_RATE_HZ` (10 kSPS by default) so the
  detector runs at the rate its bounds were written for. Set it to `0` to
  free-run: with no oversampling the AD7606 needs ~3.5 us per conversion plus
  ~11 us of bit-banged bus reads, so the loop free-runs at on the order of
  50 ksamples/s — 10 kSPS leaves a wide margin.
* Hardware-in-the-loop check of the whole chain: see
  "Bench test: DAC → AD7606 loopback" above.

## Troubleshooting

### `undefined reference to pinMode` / `digitalWrite`

The mbed core is compiled once and cached **per build configuration** in
`%LOCALAPPDATA%\arduino\cores\arduino_mbed_giga_giga_split_75_25_<hash>\core.a`,
and `arduino-cli` reuses that archive on every later build. If a build is
**killed** (editor stopped, terminal closed, machine slept), the archive is left
truncated and reused forever, so the link then fails with
`undefined reference to pinMode` / `digitalWrite` — both live in
`wiring_digital.cpp.o`, which is simply missing from the short archive. Delete
both caches and rebuild:

```powershell
Remove-Item "$env:LOCALAPPDATA\arduino\cores"    -Recurse -Force
Remove-Item "$env:LOCALAPPDATA\arduino\sketches" -Recurse -Force
```

A healthy archive for `split=75_25` is ≈52 MB (29 members); a truncated one is
much smaller and misses members. Deleting only `...\sketches` is *not* enough.

### `No DFU capable USB device available` (uploading error: exit status 74)

Two different causes:

* **The port moved between the two passes.** The board re-enumerates while it is
  being flashed: the bootloader keeps the port that was touched and the app comes
  back on a different one (seen here: `-Port COM7` for the M7, app back as
  `COM6`). `flash.ps1` re-detects the port before each upload and warns
  `port COM7 is gone ... - re-detecting`. Check the live port with
  `arduino-cli board list`: a running app reports `Arduino Giga R1`
  (`arduino:mbed_giga:giga`), a board stuck in the bootloader does not.
* **The bootloader WinUSB driver is missing** — see *First-time setup (fresh
  board)* above.
