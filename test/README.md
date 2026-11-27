# SwitchBank tests

SwitchBank keeps its verification under `test/` so the release library and the validation evidence have a clear separation.

## Complete host-side suite

Run:

```bash
bash test/run_host_checks.sh
```

This requires the deterministic suite on genuine GNU G++ and Clang, ASan/UBSan execution, public example syntax against host stubs, and release contracts. Missing capabilities fail. Set `GCC_CXX` or `CLANG_CXX` (`GNU_CXX` remains a compatibility alias) to select compilers outside PATH. `/usr/bin/g++` on macOS may be Clang and does not satisfy the GNU requirement.

## Native deterministic tests

Run:

```bash
bash test/run_native_tests.sh
```

The suite uses C++11 with strict warnings and directly covers the acquisition and state contracts: missing/failing readers, coherent packed acquisition, active-high/low normalization, debounce and bounce, recovery after acquisition gaps, timestamp wrap, monotonic acquisition/change sequences, resynchronization generations, explicit edge consumption, runtime polarity rollback, 32-bit packing, consumer constructor compatibility, and selector legal-state handling.

Set `CXX=clang++` to repeat the same suite with Clang.

## Sanitizers

Run:

```bash
bash test/run_sanitizers.sh
```

This builds the complete deterministic suite with AddressSanitizer and UndefinedBehaviorSanitizer.

## Public example syntax gate

Run:

```bash
bash test/check_examples_host.sh
```

Small host stubs let the four Arduino examples compile under the same strict warning policy without pretending to emulate actual hardware.

## Release contracts

Run:

```bash
bash test/check_release_contracts.sh
```

This checks version consistency, required README structure, package hygiene, example presence, source documentation headers, and removal of obsolete CI artifacts.

## Public distribution

`test/public_files.json` lists the complete GitHub source tree and the PlatformIO and Arduino projections. Keep the PlatformIO list synchronized with `library.json`'s `export.include`.

Run `python3 -B test/test_release_packaging.py` with PlatformIO Core 6.1.19 to test the public inventories, archive member checks and commit/index boundaries using a temporary repository. Build references from a full commit SHA with `python3 -B test/package_release.py COMMIT NEW_OUTPUT_DIRECTORY`. The directory must be outside the checkout. The PlatformIO tarball produced here is the raw packer output for normalization and release verification.

## Cross-platform CI

GitHub Actions installs PlatformIO and compiles `test/portable_compile/portable_compile.ino` for the configured MCU families. Public examples are compiled separately against ESP32-S3.

The v1.2.0 release was also locally compiled with PlatformIO for `esp32-s3-devkitc-1`, `esp32dev`, `esp8266_nodemcuv2`, `pico`, `mkrzero`, `nano_every`, `uno`, `teensy41`, and `bluepill_f103c8`. The four public examples were locally compiled for `esp32-s3-devkitc-1`.

These compile gates do not replace physical testing. GPIO electrical behaviour, MCP23017 disconnect handling, I2C driver failure reporting, and real selector transitions still require target hardware.

## Integer-width boundaries

Native tests cover 15, 16, 17, 31 and 32 inputs, indices 15/16/31, low-bit clearing with upper-bit preservation, all-low/all-high and mixed polarity, reverse packing, reader variants, and polarity resynchronization/rollback. The portable sketch asserts masks in C++11 and instantiates 32-input acquisition paths. Its optional C++14 assertions additionally evaluate loop-based factories; compile them with the real AVR toolchain, without Arduino stubs:

```bash
"$AVR_CXX" -mmcu=atmega328p -std=c++14 -Werror=shift-count-overflow -DSWITCHBANK_MASK_PROBE -x c++ -fsyntax-only -Isrc test/portable_compile/portable_compile.ino
```

Set `AVR_CXX` to the installed `avr-g++` executable. The probe asserts the actual target integer widths. C++14 is only for this additional constant-expression test, not a library requirement. The 1.2.1 package build scope is ESP32-S3, Uno and Nano Every plus the four ESP32-S3 examples; the broader list above is historical evidence. Runtime fixture code is compile coverage until executed on hardware.
