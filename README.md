# SwitchBank

Generic **N-bit switch bank** handler for Arduino/ESP platforms with robust
**per-bit debounce**, optional **compile-time or runtime polarity**,
**latch modes**, **scan throttling**, and **pluggable readers**
(GPIO, expanders, or custom sources).

The **core library is device-agnostic** and does not depend on Arduino GPIO.
An optional **Arduino helper layer** provides a beginner-friendly path with
automatic `pinMode`, `digitalRead`, and `millis()` handling.

> **Author:** Little Man Builds  
> **License:** MIT

---

## Highlights

- **Per-bit debounce** (independent raw + stable tracking)
- **Polarity control** (active-low / active-high, compile-time or runtime)
- **Latch modes** (`ManualClear`, `ClearOnRead`)
- **Scan throttling** (minimum poll interval, ideal for I²C/SPI expanders)
- **Optional reversed bit order**
- **Edge helpers**: `rose(i)`, `fell(i)` + packed masks
- **Packed helpers**: `value()`, `changedMask()`, `value8()` / `changedMask8()` (`N ≤ 8`)
- **POD snapshot** (`snapshot()`)
- **Optional on-commit callback** (zero-cost when disabled)
- **Pluggable readers** (GPIO, MCP23017, custom)
- **Header-only core**

---

## Installation

**Arduino IDE:** Install via Library Manager or ZIP  
**PlatformIO:** Add to `lib_deps` or install from ZIP

> The core library has **no external dependencies**.  
> Some examples use _Adafruit_MCP23X17_ to demonstrate expanders.

---

## Supported Platforms

This library is continuously tested on the following platforms:

- AVR (Uno, Nano, Mega-class boards)
- megaAVR (Nano Every)
- ESP32
- ESP8266
- RP2040 (Raspberry Pi Pico)
- SAMD (MKR / Zero)
- STM32 (Nucleo-class boards)
- Teensy 4.x

Other Arduino-compatible boards may work, but are not currently part of the automated test matrix.

---

## Core Concepts

- **Key** – Value stored in the key array (GPIO number, expander index, etc.).
- **Logical index** – Position in the bank (`0..N-1`).
- **Raw state** – Instantaneous electrical read (may bounce).
- **Committed state** – Debounced, stable state.

### Debounce Model

For each bit:

1. Track last raw value and timestamp.
2. When raw input remains unchanged for `debounce_ms`, **commit** the new state.
3. Edge helpers (`rose`, `fell`) compare previous vs current committed states.

### Sync vs Commit

SwitchBank intentionally keeps **constructors / factories side-effect free** (no pin reads, no time reads).
That makes the library more portable and makes README code safer to copy/paste.

> **Rule of thumb:**  
> Use `sync()` for startup and reconfiguration.  
> Use `commit()` only when you intentionally want edges _right now_.

After IO is configured (e.g. after `pinMode(...)` / expander init), call **`sync()` once** to establish a baseline:

- `sync()` reads the current hardware levels
- sets `current == previous`
- clears `changed()`
- resets per-bit debounce history

A **commit** is different:

- `commit()` reads hardware and commits immediately
- if the value differs from the current state, it **will** generate edges (`changed/rose/fell`)

In short: use **`sync()` on boot**, use **`commit()` when you intentionally want an immediate commit**.

---

## Quick Use (GPIO – Beginner Path)

Uses **`SwitchBank_Arduino.h`**, which:

- configures `pinMode` automatically,
- uses `digitalRead` internally,
- uses `millis()` as the time source.

> **Portability note:** `PinModeCfg::Pulldown` maps to `INPUT_PULLDOWN` when the selected Arduino core provides it.
> If `INPUT_PULLDOWN` is not available, SwitchBank falls back to plain `INPUT` (so you can still compile, but you must use an external pulldown resistor).

```cpp
#include <Arduino.h>
#include <SwitchBank_Arduino.h>

// DIP switch pins
const uint8_t DIP_PINS[3] = {25, 26, 27};

auto dip = makeSwitchBankArduino<3>(
    DIP_PINS,
    20,
    Polarity::ActiveLow,
    PinModeCfg::Pullup
);

void setup()
{
    Serial.begin(115200);
    dip.sync(); // establish baseline (no edges on boot)
}

void loop()
{
    if (dip.update())
    {
        for (int i = 0; i < dip.size(); ++i)
        {
            if (dip.rose(i)) // OFF -> ON
                Serial.printf("Switch %d ON\n", i + 1);

            if (dip.fell(i)) // ON -> OFF
                Serial.printf("Switch %d OFF\n", i + 1);
        }
    }
    delay(100);
}
```

---

## Quick Use (Port Expander – Advanced Path)

Uses the **core factories** with a custom reader (GPIO, expanders, or anything you can index).

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

// Default I2C address 0x20.
const uint8_t MCP_ADDR = 0x20;

// SwitchBank index 0..7 maps to these expander pins (using GPA0..GPA7).
const uint8_t EXP_PINS[8] = {0, 1, 2, 3, 4, 5, 6, 7};

// MCP23017 I2C GPIO expander instance.
Adafruit_MCP23X17 mcp;

// Time source (ms).
uint32_t now_ms() { return (uint32_t)millis(); }

// Reader returns the *electrical* level: true == HIGH, false == LOW.
bool readExpander(uint8_t pin)
{
    return mcp.digitalRead(pin) == HIGH;
}

// Create the SwitchBank as a normal global object.
// Constructors / factories are side-effect free (no pin reads, no time reads).
static auto bank = makeSwitchBankPins<8>(EXP_PINS, 30, readExpander, now_ms);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    if (!mcp.begin_I2C(MCP_ADDR))
    {
        Serial.println("MCP23017 not found!");
        while (true) delay(100);
    }

    for (uint8_t i = 0; i < 8; ++i)
        mcp.pinMode(EXP_PINS[i], INPUT_PULLUP);

    bank.setMinPollMs(5);

    // Establish baseline after IO is configured (no edges on boot).
    bank.sync();
}

void loop()
{
    if (bank.update())
    {
        for (int i = 0; i < bank.size(); ++i)
        {
            if (bank.rose(i)) Serial.printf("S%d ON\n", i + 1);
            if (bank.fell(i)) Serial.printf("S%d OFF\n", i + 1);
        }
    }
    delay(100);
}
```

> **Note:** The core library does not depend on Adafruit. Only this example requires it.

---

## Advanced Usage

### SwitchBankBuilder (Fluent Construction)

Sometimes you want a readable, chainable setup without picking a specific factory overload.
`SwitchBankBuilder` gives you a **runtime-polarity** construction path that stays lightweight (no heap) and keeps configuration in one place.

```cpp
#include <Arduino.h>
#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

static const uint8_t KEYS[4] = {25, 26, 27, 14};

void setup()
{
    for (uint8_t p : KEYS) pinMode(p, INPUT_PULLUP);

    // Reader returns the *electrical* level: true == HIGH, false == LOW.
    // Polarity is controlled by the active-low mask (below).
    static auto bank_storage = SwitchBankBuilder<4>{KEYS}
        .withDebounce(20)
        .withAllActiveLow() // LOW means logical ON
        .withTime([]() -> uint32_t { return (uint32_t)millis(); })
        .withReader([](uint8_t pin) -> bool { return digitalRead(pin) == HIGH; })
        .build();

    bank_storage.sync();
}

void loop()
{
    bank_storage.update();
}

// Mixing polarities is also straightforward:

static const uint8_t ACTIVE_HIGH[1] = {2}; // only index 2 is active-high; all others active-low

void setup_mixed()
{
    for (uint8_t p : KEYS) pinMode(p, INPUT_PULLUP);

    static auto mixed_storage = SwitchBankBuilder<4>{KEYS}
        .withDebounce(20)
        .withActiveHighIndices(ACTIVE_HIGH)
        .withTime([]() -> uint32_t { return (uint32_t)millis(); })
        .withReader([](uint8_t pin) -> bool { return digitalRead(pin) == HIGH; })
        .build();

    mixed_storage.sync();
}
```

If you prefer named functions over lambdas, here's the same pattern:

```cpp
#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

static const uint8_t KEYS[4] = {0, 1, 2, 3};

uint32_t now_ms() { return (uint32_t)millis(); }
bool readPin(uint8_t key) { return digitalRead(key) == HIGH; } // electrical level

void setup()
{
    for (uint8_t p : KEYS) pinMode(p, INPUT_PULLUP);

    static auto bank_storage = SwitchBankBuilder<4>{KEYS}
        .withDebounce(20)
        .withAllActiveLow()          // or: .withActiveLowMask(...)
        .withTime(&now_ms)           // enables no-arg update()
        .withReader(&readPin)        // or: .withReader(readCtx, ctx)
        .build();

    bank_storage.sync();
}
```

> Builder instances are simple aggregates; they do not allocate memory.

### Factory Selection

- **Pins** factories: direct GPIO or expander pin numbers.
- **Ctx** factories: use a context pointer (shared state, cached reads).
- **Masked** variants: supply a fixed active-low mask at construction.
- **CT (compile-time)** variants: smallest and fastest; polarity fixed at compile time.
- **Rev** variants: reverse bit order when wiring is mirrored.

### Polarity Masks

A polarity mask defines which bits are **active-low**.

- Bit = 1 → active-low
- Bit = 0 → active-high

Example for a 4-bit bank:

```
Mask = 0b0101
Bit 0: active-low
Bit 1: active-high
Bit 2: active-low
Bit 3: active-high
```

#### Polarity Mask Helpers

SwitchBank includes a set of helpers to make polarity masks easy and self-documenting (see `SwitchBank_Factory.h`):

- `mask_all_active_low<N>()` – all bits active-low
- `mask_all_active_high<N>()` – all bits active-high
- `mask_from_active_high_indices<N, K>(idx)` – indices listed are active-high, all others active-low (compile-time array)
- `mask_from_active_high_indices({1, 3})` – runtime `initializer_list` variant
- `mask_from_active_low_array<N>(active_low)` – build a mask from a per-bit `bool` array (`true` means active-low)

Example (4 inputs; indices 1 and 3 are active-high, the rest active-low):

```cpp
static const uint8_t AH[2] = {1, 3};
const uint32_t mask = mask_from_active_high_indices<4, 2>(AH);
// mask bits set to 1 => active-low
```

### Bit Order (LSB / MSB)

By default, SwitchBank packs switch states **LSB-first**.

This means:

- `keys[0]` maps to **bit 0** (least-significant bit)
- `keys[1]` maps to **bit 1**
- ...
- `keys[N-1]` maps to **bit N-1**

#### Example (3-bit bank)

```cpp
uint8_t keys[3] = {A, B, C};
```

| Switch | Bit |
| -----: | :-: |
|      A |  0  |
|      B |  1  |
|      C |  2  |

If switches **A** and **C** are ON:

```
value() == 0b101 == 5
```

This packing is used consistently by:

- `value()` / `prevValue()`
- `changedMask()`, `risingMask()`, `fallingMask()`
- `rose(i)` / `fell(i)`
- snapshots and polarity masks

#### Reversed Bit Order

Some hardware is wired in the opposite direction (for example, the leftmost switch should be the MSB).
Rather than reordering your pin array, SwitchBank supports **compile-time bit order reversal**.

Use a `Rev` factory or set `ReverseOrder = true`.

```cpp
auto bank = makeSwitchBankPinsRev<3>(keys, 20, readPin, now_ms);
```

With reversed order, the mapping becomes:

| Switch | Bit |
| -----: | :-: |
|      A |  2  |
|      B |  1  |
|      C |  0  |

The same physical switches now produce:

```
value() == 0b101
```

> **Important**
>
> Bit order reversal affects **everything**:
>
> - packed values
> - per-bit helpers (`rose(i)`, `fell(i)`)
> - polarity masks
> - snapshot fields
>
> Logical indices (`i`) always refer to the **logical switch order**, not the physical wiring order.

### Latch Modes

| Mode        | Behavior                                        |
| ----------- | ----------------------------------------------- |
| ManualClear | `changed()` remains true until `clearChanged()` |
| ClearOnRead | Reading `value()` clears the changed flag       |

Use `peekValue()` if you need a read without side effects.

### Scan Throttling

For slow buses (I²C/SPI):

```cpp
bank.setMinPollMs(10); // limit polling rate
```

Practical starting points:

- **GPIO (direct pins):** `min_poll_ms = 0` (poll as fast as your loop/task allows)
- **I²C expanders (e.g., MCP23017):** start around `5..20 ms` depending on bus speed and how many devices share the bus

Scan throttling is not a debounce substitute; it simply limits how often reads happen (useful for slow buses or power saving).

### Port Expander Notes (Performance)

This addendum is intentionally short and example-focused. It does not change any APIs; it just explains two patterns used in the MCP23017 example.

#### Cached reads (fast I2C scans)

`makeSwitchBankPins()` uses a per-key reader (`bool read(key)`) and will call it once per input during an `update()` scan.

For I²C expanders, a naive reader like `mcp.digitalRead(pin)` can cause **N I²C transactions per scan** (one call per input).
If you want performance, cache a packed port read **once per loop()** and serve per-key reads from the cached value:

```cpp
static uint8_t g_portA_cache = 0;

static void refreshExpanderCache()
{
    const uint16_t gpioAB = mcp.readGPIOAB();       // one I2C transaction per loop()
    g_portA_cache = (uint8_t)(gpioAB & 0x00FF);     // GPA0..GPA7
}

static bool readExpander(uint8_t pin)
{
    return ((g_portA_cache >> pin) & 1) != 0;       // HIGH=true, LOW=false
}

void loop()
{
    refreshExpanderCache();
    bank.update();
}
```

### Runtime Polarity Changes

```cpp
bank.setActiveLowMask(0xFF); // all bits active-low
```

### Snapshot Logging

`snapshot()` returns a POD struct without side effects:

```cpp
auto s = bank.snapshot();
Serial.println(s.value);
```

---

## On-Commit Callback

Enable with:

```cpp
#define SWITCHBANK_ENABLE_COMMIT_CALLBACK
```

Then:

```cpp
bank.setOnCommit([](const SwitchBankSnapshot& s) noexcept {
    Serial.printf("Commit value=%lu seq=%lu\n",
                  (unsigned long)s.value,
                  (unsigned long)s.seq);
});
```

Callbacks run in the caller context and must be fast.

> **Note:** `commit()` is an advanced operation.  
> It is not required for normal polling and should not be used for boot-time initialization.

---

## Snapshot Struct

```cpp
struct SwitchBankSnapshot {
    uint32_t value;    // stable packed value
    uint32_t changed;  // bits that toggled on last commit
    uint32_t rising;   // 0→1 edges on last commit
    uint32_t falling;  // 1→0 edges on last commit
    uint32_t t_ms;     // timestamp (ms) of last commit
    uint32_t seq;      // commit sequence number
};
```

---

## Performance Notes

- Call `update()` every 1–10 ms for GPIO.
- Use scan throttling for expanders.
- Prefer compile-time polarity when wiring is fixed.

---

## Compatibility Testing

All examples are regularly compiled across supported platforms using PlatformIO.

Before each release, `pio run -c platformio.ci.ini` is used to verify cross-platform compatibility.

---

## Contributing / Issues

If you encounter issues on unsupported platforms, please include:

- Board name
- Core version
- PlatformIO / Arduino IDE version
- Minimal repro sketch

---

## License

MIT © Little Man Builds
