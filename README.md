# SwitchBank

Generic **N-bit switch bank** library for Arduino-class targets.

SwitchBank is a header-only input-state engine for DIP/slide/rocker switch banks with:

- per-bit debounce
- active-low/active-high polarity control
- edge detection helpers (`rose`, `fell`, masks)
- optional scan throttling
- compile-time or runtime polarity paths
- optional reversed bit order
- optional commit callback

The core is device-agnostic. You provide a reader function that returns an electrical level (`HIGH` or `LOW`) for each key.

An optional Arduino wrapper (`SwitchBank_Arduino.h`) adds:

- automatic `pinMode(...)`
- `digitalRead(...)` integration
- `millis()` integration

> Author: Little Man Builds  
> License: MIT

---

## Highlights

- **Header-only core** (no `.cpp` implementation units required)
- **N = 1..32** switches per bank
- **Per-bit debounce** (raw + stable tracking per input)
- **Edge helpers** (`rose(i)`, `fell(i)`, `risingMask()`, `fallingMask()`)
- **State snapshots** (`snapshot()`)
- **Latch behavior control** (`ManualClear`, `ClearOnRead`)
- **Scan throttling** (`setMinPollMs(...)`) for slow buses
- **Factory helpers + fluent builder**
- **Portable mask helpers** (array/variadic, optional `initializer_list`)

---

## Installation

### Arduino IDE

- Install from Library Manager (if published), or
- Add as ZIP from this repository.

### PlatformIO

Use as a library dependency or local project library.

Core library dependencies: **none**.  
Some expander examples use:

- `adafruit/Adafruit MCP23017 Arduino Library`

---

## Supported Platforms

The repository currently carries PlatformIO environments for:

- AVR (`uno`)
- megaAVR (`nano_every`)
- ESP32 (`esp32-s3-devkitc-1`)
- ESP8266 (`esp8266_nodemcuv2`)
- RP2040 (`pico`)
- SAM (`mkrzero`)
- STM32 (`bluepill_f103c8`)
- Teensy (`teensy41`)

See `platformio.ini` for exact environment names.

---

## Core Concepts

- **Key**: the identifier stored in your key array (GPIO number, expander pin index, etc.)
- **Logical index**: `0..N-1`, position in the packed switch bank
- **Electrical level**: what your reader returns (`true` for HIGH, `false` for LOW)
- **Logical ON/OFF**: normalized state after polarity is applied
- **Committed state**: debounced stable state

### Reader Contract

Reader functions must return the **electrical** level:

- `true` => pin reads HIGH
- `false` => pin reads LOW

SwitchBank applies polarity to map electrical levels to logical ON/OFF.

---

## Lifecycle: `sync()`, `update()`, `commit()`

Constructors/factories are intentionally side-effect free (no reads, no time calls).

Typical startup flow:

1. Configure IO (`pinMode`, expander init, bus init).
2. Call `sync()` once to establish baseline.
3. Call `update()` repeatedly in `loop()`/task.

### `sync()`

- reads hardware now
- sets `current == previous`
- clears `changed()`
- resets debounce history

Use this on boot and after polarity reconfiguration.

### `update()`

- runs debounce integration
- commits only when stable value changes
- returns `true` only when a commit occurred

Time-source behavior:

- `update(now_ms)` always uses your explicit timestamp
- no-arg `update()` uses injected time source when provided
- on Arduino builds, no-arg `update()` falls back to `millis()` if no time source is injected
- on non-Arduino builds without a time source, no-arg `update()` returns `false`

### `commit()`

- bypasses debounce and commits immediate reading
- may generate edges if value differs

Use only when you explicitly want immediate commit behavior.

---

## Quick Start: Arduino Wrapper

```cpp
#include <Arduino.h>
#include <SwitchBank_Arduino.h>

const uint8_t DIP_PINS[3] = {25, 26, 27};

auto dip = makeSwitchBankArduino<3>(
    DIP_PINS,
    20,                  // debounce ms
    Polarity::ActiveLow, // LOW means logical ON
    PinModeCfg::Pullup   // INPUT_PULLUP
);

void setup()
{
    Serial.begin(115200);
    dip.sync(); // baseline without edges
}

void loop()
{
    if (dip.update())
    {
        for (uint8_t i = 0; i < dip.size(); ++i)
        {
            if (dip.rose(i))
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" ON");
            }

            if (dip.fell(i))
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" OFF");
            }
        }
    }
}
```

Portability note:

- `PinModeCfg::Pulldown` maps to `INPUT_PULLDOWN` when available.
- If the selected core does not define `INPUT_PULLDOWN`, wrapper falls back to `INPUT`.

---

## Quick Start: Core + MCP23017

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

const uint8_t MCP_ADDR = 0x20;
const uint8_t EXP_PINS[8] = {0, 1, 2, 3, 4, 5, 6, 7};

Adafruit_MCP23X17 mcp;

uint32_t now_ms() { return (uint32_t)millis(); }
bool readExpander(uint8_t pin) { return mcp.digitalRead(pin) == HIGH; } // electrical level

static auto bank = makeSwitchBankPins<8>(EXP_PINS, 20, readExpander, now_ms);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    if (!mcp.begin_I2C(MCP_ADDR))
    {
        Serial.println("MCP23017 not found");
        while (true) { delay(1000); }
    }

    for (uint8_t i = 0; i < 8; ++i)
        mcp.pinMode(EXP_PINS[i], INPUT_PULLUP);

    bank.setMinPollMs(5);
    bank.sync(now_ms());
}

void loop()
{
    if (bank.update(now_ms()))
    {
        // Handle changes...
    }
}
```

---

## Example Sketches

- `examples/01_Simple_DIP_3-bit/01_Simple_DIP_3-bit.ino`
  - Basic Arduino wrapper usage and edge printing.
- `examples/02_Mode_Names_3-bit/02_Mode_Names_3-bit.ino`
  - Mapping packed values to mode names.
- `examples/03_Port_Expander_8-bit/03_Port_Expander_8-bit.ino`
  - Direct MCP23017 per-pin reads.
- `examples/04_Port_Expander_Cached_Read/04_Port_Expander_Cached_Read.ino`
  - Cached expander reads (one bus read per loop).

---

## Public API Summary

### Core Type

```cpp
template <size_t N, int64_t PolarityMask = -1, bool ReverseOrder = false>
class SwitchBank;
```

- `N`: number of switches (`1..32`)
- `PolarityMask`:
  - `-1` => runtime polarity mask
  - `>= 0` => compile-time active-low mask
- `ReverseOrder`:
  - `false` => `keys[0] -> bit0`
  - `true` => reversed bit packing

### Key Methods (Core)

- lifecycle: `sync`, `update`, `commit`
- state: `value`, `peekValue`, `prevValue`, `changed`
- edges: `rose`, `fell`, `changedMask`, `risingMask`, `fallingMask`
- config: `setDebounceMs`, `setMinPollMs`, `setLatchMode`, `setTimeSource`
- polarity: `setActiveLowMask`, `activeLowMask`
- metadata: `size`, `kSize`, `lastCommitMs`, `changeCount`
- helpers: `value8`, `changedMask8` (only when `N <= 8`)
- snapshot: `snapshot`
- optional callback API (when `SWITCHBANK_ENABLE_COMMIT_CALLBACK` is defined): `setOnCommit`

### Factories (`SwitchBank_Factory.h`)

- runtime polarity:
  - `makeSwitchBankPins`
  - `makeSwitchBankCtx`
  - `makeSwitchBankPinsMasked`
  - `makeSwitchBankCtxMasked`
- reversed runtime bit order:
  - `makeSwitchBankPinsRev`
  - `makeSwitchBankCtxRev`
- compile-time polarity (+ optional reverse):
  - `makeSwitchBankPinsCT`
  - `makeSwitchBankCtxCT`

### Builder (`SwitchBankBuilder<N>`)

Fluent runtime-polarity construction with:

- `withDebounce`
- `withAllActiveLow` / `withAllActiveHigh`
- `withActiveLowMask` / `withActiveHighIndices`
- `withTime`
- `withReader` (pin reader or context reader)
- `build`
- optional callback hookup (when `SWITCHBANK_ENABLE_COMMIT_CALLBACK` is defined): `onCommit`

Important:

- configure a reader before `build()`
- in debug builds, `build()` asserts if no reader is configured

### Arduino Wrapper (`SwitchBank_Arduino.h`)

- `makeSwitchBankArduino<N>(...)` returns `SwitchBankArduino<N>` (runtime polarity).
- `core()` provides access to the underlying `SwitchBank<N, PolarityMask, ReverseOrder>`.
- For compile-time polarity/reversed order with wrapper ergonomics, instantiate directly:

```cpp
const uint8_t PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
SwitchBankArduino<8, 0xFF, true> bank(
    PINS,
    20,
    Polarity::ActiveLow,
    PinModeCfg::Pullup
);
```

---

## Polarity Masks

Mask bit semantics:

- bit = `1` => active-low
- bit = `0` => active-high

Helpers:

- `mask_all_active_low<N>()`
- `mask_all_active_high<N>()`
- `mask_from_active_high_indices<N, K>(array)`
- `mask_from_active_high_indices<N>(idx0, idx1, ...)` (portable variadic)
- `mask_from_active_low_array<N>(bool_array)`
- `mask_from_active_high_indices({1, 3})` (`initializer_list` overload when available on toolchain)

---

## Bit Order

Default mapping:

- `keys[0] -> bit 0`
- `keys[1] -> bit 1`
- ...

Reversed mapping (`ReverseOrder=true` or `*Rev` factories):

- `keys[0] -> bit N-1`
- `keys[N-1] -> bit 0`

Bit order affects packed values, edge masks, helper index interpretation, and polarity masks.

---

## Latch Modes

- `ManualClear`: `changed()` stays true until `clearChanged()`
- `ClearOnRead`: calling `value()` clears `changed()`

Use `peekValue()` for side-effect-free reads.

---

## Scan Throttling

Use to reduce expensive scans (I2C/SPI expanders):

```cpp
bank.setMinPollMs(10);
```

Guidelines:

- direct GPIO: often `0`
- bus expanders: start around `5..20` ms

This is not a debounce replacement; it only limits scan frequency.

---

## Port Expander Performance: Cached Reads

For expanders, avoid N bus transactions per scan when possible.

```cpp
static uint8_t g_portA_cache = 0;

static void refreshExpanderCache()
{
    const uint16_t gpioAB = mcp.readGPIOAB();   // one I2C read
    g_portA_cache = (uint8_t)(gpioAB & 0x00FF); // GPA0..GPA7
}

static bool readExpander(uint8_t pin)
{
    return ((g_portA_cache >> pin) & 1) != 0;   // electrical HIGH/LOW
}

void loop()
{
    refreshExpanderCache();
    bank.update();
}
```

---

## Runtime Polarity Changes

```cpp
bank.setActiveLowMask(0xFF);
```

Behavior note:

- when runtime polarity is enabled (`PolarityMask == -1`), mask is updated and the bank resyncs
- when compile-time polarity is used, runtime mask changes are ignored (no-op)

---

## Snapshot and Callback

`snapshot()` returns:

```cpp
struct SwitchBankSnapshot {
    uint32_t value;
    uint32_t changed;
    uint32_t rising;
    uint32_t falling;
    uint32_t t_ms;
    uint32_t seq;
};
```

Enable callback support by defining:

```cpp
#define SWITCHBANK_ENABLE_COMMIT_CALLBACK
```

Then register:

```cpp
bank.setOnCommit([](const SwitchBankSnapshot& s) noexcept {
    Serial.print("Commit value=");
    Serial.println((unsigned long)s.value);
});
```

Callbacks execute in caller context (`update` / `commit`), so keep them fast and non-blocking.

---

## Testing and Validation

This repository uses PlatformIO environments in `platformio.ini`.

### Build current default environment

```bash
pio run
```

### Build a specific environment

```bash
pio run -e nano_every
```

### Build all configured environments

```bash
for env in esp32-s3-devkitc-1 esp8266_nodemcuv2 pico mkrzero nano_every uno teensy41 bluepill_f103c8; do
  pio run -e "$env" || break
done
```

### Compile each example across all environments

```bash
examples=(
  examples/01_Simple_DIP_3-bit
  examples/02_Mode_Names_3-bit
  examples/03_Port_Expander_8-bit
  examples/04_Port_Expander_Cached_Read
)

envs=(esp32-s3-devkitc-1 esp8266_nodemcuv2 pico mkrzero nano_every uno teensy41 bluepill_f103c8)

for ex in "${examples[@]}"; do
  for env in "${envs[@]}"; do
    echo "=== $env :: $ex ==="
    pio run -e "$env" --project-option "src_dir=$ex" || exit 1
  done
done
```

---

## Troubleshooting

- No changes detected:
  - verify reader returns electrical level (`HIGH=true`, `LOW=false`)
  - verify polarity mask / `Polarity` selection
  - ensure `sync()` was called after IO init
- Unexpected startup edges:
  - call `sync()` once after initialization, before normal polling
- Expander polling feels slow:
  - use cached-read pattern and tune `setMinPollMs(...)`

---

## Contributing

When reporting an issue, include:

- board name
- core/platform version
- PlatformIO or Arduino IDE version
- minimal reproducible sketch

---

## License

MIT Copyright (c) Little Man Builds
