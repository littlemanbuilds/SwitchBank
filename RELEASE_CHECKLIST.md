# SwitchBank release checklist

SwitchBank 1.2.2 candidate prepared 2026-11-27. Use this checklist before publication.

SwitchBank 1.2.2 completed software qualification at the engineering candidate stage. The library headers differ from the accepted 1.2.1 baseline only in the package version macros, preserving the 16-bit mask-width correction. The two beginner GPIO examples use GPIO 4–6 and were compiled for ESP32-S3 during 1.2.2 qualification.

## Source and API

- [x] Library version agrees across `SwitchBank.h`, `library.properties`, and `library.json`.
- [x] Existing beginner GPIO API remains source-compatible.
- [x] Missing-reader behaviour fails closed.
- [x] External acquisition failure cannot become a logical switch state.
- [x] Packed-reader acquisition is coherent at the SwitchBank callback boundary.
- [x] `sample_ms`, `change_ms`, `sequence`, `change_sequence`, and `generation` have distinct documented meanings.
- [x] `sync()` does not reset sequence history.
- [x] Debounce time does not accumulate across invalid acquisition gaps.
- [x] Immediate debounce bypass is explicitly named for commissioning.

## Documentation and examples

- [x] README includes installation, a beginner path, API reference, testing and migration guidance.
- [x] Source headers document callable contracts and preserve creation dates.
- [x] Existing four-example progression is retained.
- [x] Example GPIO choices remain appropriate for the ESP32-S3 target.
- [x] Example 04 demonstrates one coherent expander snapshot.
- [x] `keywords.txt`, package metadata, and repository tree match the public API.

## Automated validation performed locally

- [x] Native C++11 deterministic suite with genuine GNU G++.
- [x] Native C++11 deterministic suite with Clang.
- [x] `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror`.
- [x] AddressSanitizer + UndefinedBehaviorSanitizer.
- [x] Strict host syntax compile of public examples.
- [x] PlatformIO package builds for ESP32-S3, AVR Uno and megaAVR Nano Every.
- [x] PlatformIO compile of all public examples for ESP32-S3.
- [x] Release-contract/package-hygiene checks.
- [x] Target builds use a sanitized library package.
- [x] AVR C++14 mask assertions use real 16-bit unsigned int; C++11 package builds retained.

## 1.2.2 release gates

- [ ] Confirm GitHub Actions on the exact release commit.
- [ ] Check the GitHub, PlatformIO and Arduino inventories and exact commit archives.
- [ ] Unpack and check the final release ZIP from a clean directory.
- [ ] Verify the actual public downloads after publication.

The exact 1.2.1 commit passed the historical GitHub Actions portable PlatformIO matrix ([run 34374074794](https://github.com/littlemanbuilds/SwitchBank/actions/runs/34374074794)).

## Physical integration

The SwitchBank owner approved a software-qualified public release for 1.2.2. Electrical behavior depends on the installed board, switches, wiring and expander driver, so the hardware integration owner is responsible for the following checks on the actual installation.

| Physical check | Disposition |
| --- | --- |
| ESP32-S3 GPIO debounce and edge behavior | DEFERRED / NOT RUN |
| Physical selector transitions and legal-state behavior | DEFERRED / NOT RUN |
| MCP23017 coherent packed acquisition | DEFERRED / NOT RUN |
| MCP23017 disconnect and recovery with a driver that reports acquisition failure | DEFERRED / NOT RUN |
