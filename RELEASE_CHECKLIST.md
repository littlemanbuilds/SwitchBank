# SwitchBank release checklist

SwitchBank 1.2.1 preparation, 2026-09-08. Use this checklist before publication.

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
- [ ] Final release ZIP unpacked and retested from a clean directory.
- [x] Target builds use a sanitized library package.
- [x] AVR C++14 mask assertions use real 16-bit unsigned int; C++11 package builds retained.

## CI / hardware gates

- [ ] GitHub Actions portable PlatformIO matrix completed remotely.
- [ ] Broader platform matrix rerun for 1.2.1 (prior 1.2.0 evidence retained).
- [ ] Physical selector transitions and legal-state behavior.
- [ ] Physical ESP32-S3 GPIO debounce/edge smoke test.
- [ ] Physical MCP23017 coherent packed-read test.
- [ ] Physical MCP23017 disconnect/recovery validation with a driver capable of reporting acquisition failure.

Hardware and remote CI gates must not be reported as passed until they have actually run.
