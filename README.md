# SwitchBank

**SwitchBank** is a small, header-only Arduino/C++ library for reading **1 to 32 physical switches as one debounced packed value**.

It is useful for DIP banks, slide switches, rotary/gear contact groups, mode selectors, button-style maintained contacts, and GPIO-expander inputs where the application wants a clean logical state instead of repeating polarity, debounce, and edge-detection code.

Version **1.2.0** keeps the simple v1.x usage model, while adding the missing reliability contract needed for larger embedded systems: failed acquisitions no longer become switch states, coherent packed hardware reads are supported, freshness is separate from state-change time, and resynchronization no longer resets the sequence history.

ESP32-S3 remains the primary project target, but the core is deliberately portable and allocation-free.

The public version is available through the package-specific `SWITCHBANK_VERSION*` macros so SwitchBank can coexist cleanly with other libraries.

---

## Contents

- [Why SwitchBank exists](#why-switchbank-exists)
- [Design boundaries](#design-boundaries)
- [Installation](#installation)
- [Supported targets](#supported-targets)
- [Beginner path](#beginner-path)
- [Core concepts](#core-concepts)
- [Reader choices](#reader-choices)
- [Validity and freshness](#validity-and-freshness)
- [Debouncing and recovery](#debouncing-and-recovery)
- [Edges and durable change tracking](#edges-and-durable-change-tracking)
- [Polarity and bit order](#polarity-and-bit-order)
- [Coherent packed reads](#coherent-packed-reads)
- [Runtime configuration](#runtime-configuration)
- [Examples](#examples)
- [Testing](#testing)
- [Public API guide](#public-api-guide)
- [Migration from v1.1](#migration-from-v11)
- [Project integration guidance](#project-integration-guidance)
- [Repository structure](#repository-structure)
- [Limitations](#limitations)
- [License](#license)

---

## Why SwitchBank exists

A group of maintained switches looks simple until the application needs to answer all of these correctly:

- Is electrical `LOW` logically ON or OFF?
- How long must the contact remain stable before it is accepted?
- Did bit 2 rise, fall, or remain unchanged?
- Is the current packed value fresh even when the switches have not moved for ten minutes?
- What happens if an I/O expander read fails?
- Can a multi-contact selector be read as one coherent hardware state?
- Did a consumer miss one or more transitions between two observations?
- What happens when the input source is explicitly resynchronized after a disconnect?

SwitchBank centralizes those mechanics while leaving application meaning outside the library.

For example, SwitchBank can report:

```text
stable packed state = 0b0110
valid               = true
sample sequence      = 4821
change sequence      = 17
```

It deliberately does **not** decide whether `0b0110` means `D1`, `SPORT`, `REMOTE`, or an invalid machine state. That belongs to the consuming application.

### What v1.2 specifically improves

Version 1.2 directly addresses the architecture audit findings:

- a missing reader now reports `MissingReader` instead of making every active-low input appear ON;
- validity-aware readers can reject a failed hardware acquisition;
- failed reads retain the last stable state rather than synthesizing all-zero/all-one data;
- one coherent packed-reader callback can acquire an entire selector from one hardware snapshot;
- `sample_ms` is now distinct from `change_ms`;
- `sequence` advances on every successful sample;
- `change_sequence` advances only on stable state transitions;
- successful `sync()` increments a separate `generation` instead of resetting sequence history;
- time spent with invalid hardware cannot satisfy a debounce window;
- explicit `consumeChanged()` and `consumeEdges()` remove the need for hidden clear-on-read behaviour;
- immediate debounce bypass is named `forceCommitForCommissioning()`;
- the old `commit()` remains only as a deprecated v1.x compatibility alias.

---

## Design boundaries

SwitchBank is intentionally narrow.

It **does** provide:

- 1..32 input packing;
- active-low/active-high normalization;
- per-bit debounce;
- optional reversed output bit order;
- hardware acquisition validity;
- coherent packed input acquisition;
- stable value and edge masks;
- acquisition and change timestamps;
- acquisition/change sequence counters;
- explicit resynchronization generations;
- optional minimum polling interval;
- Arduino GPIO convenience wrappers;
- zero dynamic allocation.

It **does not** provide:

- I2C/SPI drivers;
- MCP23017 ownership;
- machine-mode or gear decoding;
- persistence;
- an event queue;
- task scheduling;
- safety policy;
- cross-library dependencies.

This boundary is intentional. A standalone SwitchBank can be used in a toy, machine panel, robot, vehicle, or test fixture without importing any other LMB library.

---

## Installation

### Arduino IDE

Install the library in the Arduino Library Manager when released, or place the repository in your Arduino `libraries` directory.

For the easiest GPIO path:

```cpp
#include <SwitchBank_Arduino.h>
```

For custom readers or I/O expanders:

```cpp
#include <SwitchBank.h>
#include <SwitchBank_Factory.h>
```

### PlatformIO

Use the repository as a project dependency, or install it into the project `lib/` directory.

The library is header-only, so there is no runtime library initialization step beyond configuring the hardware and calling `sync()`.

---

## Supported targets

The library core is standard C++11-compatible code with small Arduino adapters.

The v1.2.0 release was locally compile-validated with PlatformIO for:

- ESP32-S3 DevKitC-1;
- ESP32;
- ESP8266;
- RP2040 Pico;
- SAMD/MKR Zero;
- Nano Every;
- AVR Uno;
- Teensy 4.1;
- STM32 Blue Pill.

The public examples are primarily written around **ESP32-S3** usage.

Cross-platform compilation is not the same as hardware validation. External pull resistors, available GPIOs, `INPUT_PULLDOWN`, I2C behaviour, and electrical characteristics still depend on the board.

GitHub Actions carries the same portable PlatformIO matrix so target compilation remains guarded after release.

---

## Beginner path

This section is the shortest route from a new library install to a working switch bank.

### 1. Wire three switches

For the first example, connect one side of each maintained switch to GND and the other sides to three available ESP32-S3 GPIO pins.

The examples use:

```cpp
const uint8_t DIP_PINS[3] = {35, 36, 37};
```

With `INPUT_PULLUP`:

```text
switch open   -> GPIO HIGH -> logical OFF
switch closed -> GPIO LOW  -> logical ON
```

That is an **active-low** input.

### 2. Create the bank

```cpp
#include <Arduino.h>
#include <SwitchBank_Arduino.h>

const uint8_t DIP_PINS[3] = {35, 36, 37};

auto dip = makeSwitchBankArduino<3>(
    DIP_PINS,
    20,
    Polarity::ActiveLow,
    PinModeCfg::Pullup);
```

`20` is the debounce window in milliseconds.

### 3. Establish the boot baseline

Call `sync()` after the input hardware is ready:

```cpp
void setup()
{
    Serial.begin(115200);
    dip.sync();
}
```

`sync()` reads the current hardware state without creating fake boot edges.

For Arduino GPIO the reader itself cannot normally report `digitalRead()` failure, so beginner sketches may ignore the return value. For external buses, check it.

### 4. Poll the switches

```cpp
void loop()
{
    if (dip.update())
    {
        Serial.println(dip.peekValue());
    }
}
```

`update()` returns `true` only when the **stable debounced packed value changes**.

The packed value for three switches is `0..7`.

### 5. Read individual edges

```cpp
if (dip.rose(0))
{
    Serial.println("Switch 1 turned ON");
}

if (dip.fell(0))
{
    Serial.println("Switch 1 turned OFF");
}
```

### 6. Understand freshness before using external hardware

For GPIO-only beginner projects, `peekValue()` is usually enough.

For an I2C/SPI expander or any selector that matters to machine authority, also use:

```cpp
if (bank.valid())
{
    const uint32_t value = bank.peekValue();
    const uint32_t sample_time = bank.sampleMs();
}
```

A retained value is **not automatically proof that the hardware is still healthy**.

### 7. Use a packed reader for multi-contact selectors

A multi-contact gear or mode selector should ideally come from one hardware snapshot rather than several independent bus transactions.

That is the point of `makeSwitchBankPacked()` and Example 04.

---

## Core concepts

### Electrical level

Readers provide the actual hardware level:

```text
true  = HIGH
false = LOW
```

SwitchBank then applies the configured polarity.

### Logical value

The library exposes logical ON/OFF bits after polarity normalization.

### Stable value

`peekValue()` is the most recently accepted debounced state.

### Sample

A sample is one complete successful hardware acquisition.

### Change

A change occurs only when the stable packed state transitions to a different value.

These are deliberately separate concepts in v1.2.

---

## Reader choices

SwitchBank supports three levels of reader contract.

### 1. Simple bool reader

Best for direct GPIO or hardware APIs that cannot report read failure:

```cpp
bool readPin(void *ctx, uint8_t key);
```

A configured bool reader is assumed to succeed. Therefore **do not hide an I2C error by returning `false`** unless LOW really is the measured electrical level.

### 2. Validity-aware per-key reader

Use this when each input acquisition can fail:

```cpp
SwitchBankReadResult readPin(void *ctx, uint8_t key)
{
    if (!hardwareReadSucceeded())
    {
        return SwitchBankReadResult::failure();
    }

    return SwitchBankReadResult::success(levelIsHigh());
}
```

Factory:

```cpp
auto bank = makeSwitchBankResultCtx<4>(keys, 20, readPin, &context, millisFn);
```

If any input in the bank fails, the whole acquisition is rejected and stable state is retained.

### 3. Coherent packed reader

Preferred for multi-contact selectors on an expander:

```cpp
SwitchBankPackedReadResult readAll(void *ctx)
{
    const uint32_t electrical_levels = readOneHardwareSnapshot();
    return SwitchBankPackedReadResult::success(electrical_levels);
}
```

Factory:

```cpp
auto bank = makeSwitchBankPacked<4>(keys, 20, readAll, &context, millisFn);
```

Bit `i` returned by the packed reader represents **bank input `i` before polarity normalization and before optional ReverseOrder packing**.

The callback therefore performs exactly one logical bank acquisition.

---

## Validity and freshness

A stable selector can remain unchanged for hours. Its last **change** time is therefore not a useful freshness signal.

SwitchBank v1.2 exposes separate metadata:

```cpp
const SwitchBankStatus st = bank.status();
```

Important fields:

| Field | Meaning |
|---|---|
| `configured` | A hardware reader callback is installed |
| `has_sample` | At least one successful acquisition has occurred |
| `valid` | The most recent attempted acquisition succeeded |
| `error` | Latest acquisition error |
| `sample_ms` | Last successful hardware acquisition |
| `attempt_ms` | Last attempted hardware acquisition |
| `error_ms` | Last failed hardware acquisition |
| `sequence` | Every successful acquisition |
| `change_ms` | Last stable state transition |
| `change_sequence` | Stable state transitions only |
| `generation` | Successful explicit resynchronizations |

### Missing reader behaviour

A default/missing reader now fails closed:

```cpp
bank.sync();

bank.configured(); // false
bank.valid();      // false
bank.hasSample();  // false
bank.lastError();  // SwitchBankReadError::MissingReader
bank.peekValue();  // retained safe/default state
```

The v1.1 behaviour where an absent reader appeared as LOW and therefore turned every active-low bit ON has been removed.

### Acquisition failure

A validity-aware failure:

- sets `valid = false`;
- records `error_ms` and `attempt_ms`;
- does not advance `sample_ms`;
- does not increment `sequence`;
- does not modify the stable state;
- does not create an edge;
- does not allow unobserved time to satisfy debounce.

When valid sampling resumes, `valid` returns to true after that successful read. The application can decide whether a higher-level fault should remain latched.

---

## Debouncing and recovery

Debouncing is per bit.

When an input first changes, SwitchBank starts a stability timer for that input. The new logical state is accepted only after the input has remained at the same observed level for the configured debounce window.

### Why failed-read time does not count

Consider a 20 ms debounce:

```text
0 ms   candidate changes
5 ms   external expander stops responding
100 ms expander recovers
```

It would be unsafe to assume the contact was stable for 100 ms because the library did not observe it during the gap.

SwitchBank v1.2 therefore rebases the candidate debounce window on the **first valid sample after an acquisition failure**.

For `debounce_ms == 0`, recovery can still take effect immediately.

### Time wrap

All elapsed-time comparisons use unsigned subtraction, so normal `uint32_t` millisecond wraparound is handled correctly.

---

## Edges and durable change tracking

SwitchBank retains the most recent stable transition through:

```cpp
bank.changedMask();
bank.risingMask();
bank.fallingMask();
bank.rose(index);
bank.fell(index);
```

### Explicit consumption

New code should prefer explicit consumption:

```cpp
const SwitchBankEdges edges = bank.consumeEdges();
```

This returns the retained masks and then clears them.

Or consume only the legacy changed latch:

```cpp
const bool changed = bank.consumeChanged();
```

### Why `change_sequence` matters

Edge masks represent latest retained state, not an event history.

If a consumer last saw:

```text
change_sequence = 17
```

and later sees:

```text
change_sequence = 20
```

then it knows three stable transitions occurred, even if it did not observe all intermediate edge masks.

If every transition must be processed individually, transport those events through an application-level queue/counter. Do not treat SwitchBank as an event journal.

### Legacy ClearOnRead mode

`LatchMode::ClearOnRead` is retained for v1.x compatibility, but it makes `value()` have a hidden side effect.

New code should keep the default:

```cpp
LatchMode::ManualClear
```

and use `peekValue()`, `consumeChanged()`, or `consumeEdges()` explicitly.

---

## Polarity and bit order

### Active-low

For a pull-up switch connected to GND:

```text
HIGH = OFF
LOW  = ON
```

Set the corresponding active-low mask bit to `1`.

### Active-high

For an input where HIGH means ON, the active-low mask bit is `0`.

### Mask helpers

```cpp
mask_all_active_low<N>();
mask_all_active_high<N>();
mask_from_active_high_indices<N>(...);
mask_from_active_low_array(...);
```

### ReverseOrder

Normal packing:

```text
keys[0] -> output bit 0
keys[1] -> output bit 1
...
```

With `ReverseOrder = true`:

```text
keys[0] -> output bit N-1
...
keys[N-1] -> output bit 0
```

Reader acquisition order does not change; only logical output packing changes.

---

## Coherent packed reads

Sequential reads are fine for unrelated GPIO switches.

They are less ideal for a selector made from several contacts. If the selector moves between two per-key reads, software may temporarily assemble a bit pattern that never existed as one physical state.

For a hardware expander that can return all inputs in one register read, use the packed-reader path:

```text
ONE hardware snapshot
        |
        v
packed electrical levels
        |
        v
polarity + bit-order normalization
        |
        v
per-bit debounce
        |
        v
stable packed selector state
```

For MCP23017 specifically, a single 16-bit GPIO read can supply the complete contact set before SwitchBank decodes it.

This does not make an electrically break-before-make selector magically atomic. It simply prevents **software acquisition skew** from adding another impossible intermediate state.

The application must still validate legal combinations such as gear contact patterns.

---

## Runtime configuration

### Poll throttling

```cpp
bank.setMinPollMs(5);
```

A throttled call does not count as an acquisition and does not increment `sequence`.

### Debounce window

```cpp
bank.setDebounceMs(20);
```

For substantial runtime configuration changes, explicitly resynchronize after the hardware/configuration is settled.

### Runtime polarity

```cpp
if (!bank.setActiveLowMask(new_mask))
{
    // Hardware could not be re-read under the new interpretation.
}
```

A successful polarity change resynchronizes without edges and increments `generation`.

If the read fails, SwitchBank restores the previous polarity mask and retains the previous stable state.

### Immediate commissioning acceptance

The old broadly named `commit()` API could bypass the library's main debounce guarantee.

The explicit v1.2 name is:

```cpp
bank.forceCommitForCommissioning();
```

Use this only when immediate acceptance is deliberate—for example during commissioning or controlled reconfiguration.

Normal runtime code should call `update()`.

---

## API reference

### Package version macros

Use `SWITCHBANK_VERSION`, `SWITCHBANK_VERSION_MAJOR`, `SWITCHBANK_VERSION_MINOR`, and `SWITCHBANK_VERSION_PATCH`. The generic `LIBRARY_VERSION*` aliases from v1.2.0 are intentionally absent from this development baseline because they are not safe in multi-library applications.

### Main types

```cpp
SwitchBank<N>
SwitchBankArduino<N>
SwitchBankHandler
SwitchBankReadResult
SwitchBankPackedReadResult
SwitchBankStatus
SwitchBankEdges
```

### Normal lifecycle

```cpp
bank.sync();
bank.update();
bank.peekValue();
```

### Health/freshness

```cpp
bank.configured();
bank.valid();
bank.hasSample();
bank.sampleMs();
bank.lastError();
bank.sequence();
bank.changeSequence();
bank.generation();
bank.status();
```

### Edge information

```cpp
bank.changed();
bank.changedMask();
bank.risingMask();
bank.fallingMask();
bank.rose(index);
bank.fell(index);
bank.consumeChanged();
bank.consumeEdges();
```

### Factories

Legacy/simple readers:

```cpp
makeSwitchBankPins<N>(...)
makeSwitchBankCtx<N>(...)
makeSwitchBankPinsMasked<N>(...)
makeSwitchBankCtxMasked<N>(...)
```

Validity-aware readers:

```cpp
makeSwitchBankResultCtx<N>(...)
makeSwitchBankResultCtxMasked<N>(...)
```

Coherent packed readers:

```cpp
makeSwitchBankPacked<N>(...)
makeSwitchBankPackedMasked<N>(...)
```

Compile-time/reversed helpers remain available through `SwitchBank_Factory.h`.

### Builder

`SwitchBankBuilder<N>` retains the existing reader paths and adds:

```cpp
.withResultReader(...)
.withPackedReader(...)
```

The builder still performs no dynamic allocation.

---

## Examples

The existing four-example progression is intentionally retained.

### `01_SimpleDIP3Bit`

The beginner GPIO path:

- three ESP32-S3 GPIOs;
- active-low pull-ups;
- 20 ms debounce;
- rising/falling edge messages.

### `02_ModeNames3Bit`

Shows how the packed `0..7` value can be translated into application-specific mode names.

### `03_PortExpander8Bit`

Shows a straightforward MCP23017 per-pin reader. This is easy to understand, but each switch may require a separate expander access.

### `04_PortExpanderCachedRead`

Shows the preferred coherent selector pattern in v1.2: one MCP23017 register snapshot is passed through `makeSwitchBankPacked()`.

The example driver does not expose a definitive bus-error result for `readGPIOAB()`, so it returns `success(...)`. A hardware adapter that can detect transfer failure should return `SwitchBankPackedReadResult::failure()` instead.

---

## Testing

Testing now lives under `test/` rather than being mixed into a PlatformIO CI configuration.

Run the complete host-side suite with:

```bash
./test/run_host_checks.sh
```

Current v1.2.0 host result: `26 tests / 1182 assertions`, repeated with Clang when available, plus strict warning-clean example syntax, ASan/UBSan, and release-contract checks.

### Native deterministic suite

Run:

```bash
./test/run_native_tests.sh
```

The suite covers:

- missing/null readers;
- active-low and active-high polarity;
- validity-aware failed reads;
- post-failure debounce recovery;
- stable-source freshness;
- sample/change timestamps;
- acquisition and transition sequences;
- explicit sync generation;
- packed coherent acquisition;
- selector transition integrity;
- debounce bounce patterns;
- `uint32_t` timestamp wrap;
- explicit edge consumption;
- missed-transition detection;
- out-of-range bit helpers;
- reversed bit order;
- runtime polarity rollback;
- commissioning bypass semantics;
- 32-bit bank masks;
- PW_PVT constructor compatibility;
- gear-selector legal packed states;
- builder/factory paths.

### Sanitizers

Run:

```bash
./test/run_sanitizers.sh
```

The deterministic host suite is built with AddressSanitizer and UndefinedBehaviorSanitizer.

### Cross-platform compile matrix

The v1.2.0 release was locally compiled with PlatformIO for:

- `esp32-s3-devkitc-1` using Espressif 32 `6.12.0`;
- `esp32dev` using Espressif 32 `6.12.0`;
- `esp8266_nodemcuv2` using Espressif 8266 `4.2.1`;
- `pico` using Raspberry Pi RP2040 `1.20.0+sha.9c167c6`;
- `mkrzero` using Atmel SAM `8.3.0`;
- `nano_every` using Atmel megaAVR `1.9.0`;
- `uno` using Atmel AVR `5.1.0`;
- `teensy41` using Teensy `5.0.0`;
- `bluepill_f103c8` using ST STM32 `19.4.0`.

All four public examples were also compiled locally for `esp32-s3-devkitc-1`.

The RP2040 toolchain emitted warnings from its bundled Pico SDK timer headers. No SwitchBank source warnings were emitted in the local PlatformIO matrix.

The local release checklist distinguishes between tests actually executed locally and CI/hardware gates that still need the target environment.

---

## Migration from v1.1

Most v1.1 sketches continue to compile unchanged.

### 1. Missing reader now fails closed

**v1.1:** null reader -> electrical LOW -> all active-low bits logically ON.

**v1.2:** null reader -> `valid=false`, `MissingReader`, stable state retained.

This is an intentional safety correction.

### 2. `sync()` now reports success

You may continue to write:

```cpp
bank.sync();
```

or check it:

```cpp
if (!bank.sync())
{
    // Input source is not ready.
}
```

A successful `sync()` no longer resets the sequence counters.

### 3. Sequence meaning is clearer

In snapshots:

```cpp
snapshot.seq
```

now means **successful acquisition sequence**.

Use:

```cpp
snapshot.change_seq
bank.changeSequence()
bank.changeCount()
```

for stable state transition count.

`changeCount()` is now monotonic across `sync()`.

### 4. Freshness uses `sampleMs()`

`lastCommitMs()` remains as a compatibility name for the latest stable state transition.

Do not use it to decide whether a stable input source is fresh.

Use:

```cpp
bank.sampleMs();
bank.sequence();
bank.valid();
```

### 5. Prefer explicit edge consumption

Instead of relying on `LatchMode::ClearOnRead`, prefer:

```cpp
bank.peekValue();
bank.consumeChanged();
bank.consumeEdges();
```

`ClearOnRead` remains available in v1.x for compatibility.

### 6. `commit()` has a clearer replacement

Use:

```cpp
bank.forceCommitForCommissioning();
```

The old `commit()` remains as a deprecated alias unless:

```cpp
#define SWITCHBANK_NO_LEGACY_COMMIT
```

is defined before including the library.

### 7. External buses should move to validity-aware readers

Existing bool callbacks remain valid for GPIO and simple APIs.

Where a bus transaction can fail, return `SwitchBankReadResult` or `SwitchBankPackedReadResult` instead of converting failure into LOW/HIGH.

---

## Project integration guidance

For a larger state/snapshot architecture, a useful application frame is conceptually:

```text
SwitchBank
   |
   +-- stable value
   +-- valid
   +-- sample_ms
   +-- sequence
   +-- change_sequence
   +-- generation
           |
           v
application adapter / state bus
           |
           v
mode, authority, gear or safety policy
```

A consumer should generally require:

1. `valid == true`;
2. `sample_ms` within its own freshness window;
3. a legal application-specific packed combination;
4. any required startup/safety state outside SwitchBank.

### PW_PVT specifically

PW_PVT currently uses one cached MCP23017 hardware snapshot for both the DIP bank and gear bank before calling SwitchBank, which already avoids independent I2C transactions for each contact.

The existing constructor shape remains source-compatible in v1.2.

A later PW_PVT integration pass should additionally propagate SwitchBank's own:

```cpp
valid()
sampleMs()
sequence()
changeSequence()
```

into the application's provider-health contract, rather than relying only on the surrounding MCP health flag.

For the multi-contact shifter, a packed-reader adapter is also a natural future simplification because the cached MCP sample is already coherent.

SwitchBank itself must remain unaware of PW_PVT, SnapshotBus, SafetyCore, MCP23017, or any other LMB library.

---

## Limitations

- A legacy bool reader cannot report acquisition failure. Use an explicit validity-aware reader when failure matters.
- A packed reader makes software acquisition coherent; it does not remove electrical contact bounce or mechanically invalid transition states.
- Edge masks represent the latest retained transition, not an event history.
- `uint32_t` sequence counters eventually wrap naturally.
- SwitchBank is not thread-safe by itself. Keep one owner/task for mutation, or place synchronization at the application boundary.
- The library cannot determine whether a packed selector value is semantically safe or legal for your machine.
- Hardware bus recovery, retries, and fault latching belong in the hardware/application layer.

---

## Repository structure

```text
SwitchBank/
├── .github/
│   └── workflows/
│       └── ci.yml
├── examples/
│   ├── 01_SimpleDIP3Bit/
│   ├── 02_ModeNames3Bit/
│   ├── 03_PortExpander8Bit/
│   └── 04_PortExpanderCachedRead/
├── src/
│   ├── SwitchBank.h
│   ├── SwitchBank_Arduino.h
│   ├── SwitchBank_Compatibility.h
│   ├── SwitchBank_Factory.h
│   └── SwitchBank_Handler.h
├── test/
│   ├── native/
│   ├── host_stubs/
│   ├── portable_compile/
│   ├── README.md
│   ├── check_examples_host.sh
│   ├── check_release_contracts.sh
│   ├── run_host_checks.sh
│   ├── run_native_tests.sh
│   └── run_sanitizers.sh
├── .gitignore
├── CHANGELOG.md
├── LICENSE
├── README.md
├── RELEASE_CHECKLIST.md
├── keywords.txt
├── library.json
├── library.properties
└── platformio.ini
```

---

## Version history

Current version: **1.2.0**. See [CHANGELOG.md](CHANGELOG.md) for detailed release notes.

---

## License

SwitchBank is released under the **MIT License**. See [LICENSE](LICENSE).

Copyright © 2026 Little Man Builds (Darren Osborne).
