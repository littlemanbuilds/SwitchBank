# SwitchBank tests

SwitchBank keeps its verification under `test/` so the release library and the validation evidence have a clear separation.

## Complete host-side suite

Run:

```bash
./test/run_host_checks.sh
```

This runs the deterministic suite, repeats it with Clang when available, runs sanitizers, compiles public examples against host stubs, and checks release contracts.

Current v1.2.0 result: `26 tests / 1182 assertions`.

## Native deterministic tests

Run:

```bash
./test/run_native_tests.sh
```

The suite uses C++11 with strict warnings and directly covers the v1.2 audit remediations: missing/failing readers, coherent packed acquisition, active-high/low normalization, debounce and bounce, recovery after acquisition gaps, timestamp wrap, monotonic acquisition/change sequences, resynchronization generations, explicit edge consumption, runtime polarity rollback, 32-bit packing, PW_PVT constructor compatibility, and selector legal-state handling.

Set `CXX=clang++` to repeat the same suite with Clang.

## Sanitizers

Run:

```bash
./test/run_sanitizers.sh
```

This builds the complete deterministic suite with AddressSanitizer and UndefinedBehaviorSanitizer.

## Public example syntax gate

Run:

```bash
./test/check_examples_host.sh
```

Small host stubs let the four Arduino examples compile under the same strict warning policy without pretending to emulate actual hardware.

## Release contracts

Run:

```bash
./test/check_release_contracts.sh
```

This checks version consistency, required README structure, package hygiene, example presence, LMB Doxygen headers, and removal of obsolete CI artifacts.

## Cross-platform CI

GitHub Actions installs PlatformIO and compiles `test/portable_compile/portable_compile.ino` for the configured MCU families. Public examples are compiled separately against ESP32-S3.

The v1.2.0 release was also locally compiled with PlatformIO for `esp32-s3-devkitc-1`, `esp32dev`, `esp8266_nodemcuv2`, `pico`, `mkrzero`, `nano_every`, `uno`, `teensy41`, and `bluepill_f103c8`. The four public examples were locally compiled for `esp32-s3-devkitc-1`.

These compile gates do not replace physical testing. GPIO electrical behaviour, MCP23017 disconnect handling, I2C driver failure reporting, and real selector transitions still require target hardware.
