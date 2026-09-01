# SwitchBank release checklist

Use this checklist before publishing a SwitchBank release.

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

- [x] README includes the standard LMB introduction / contents / installation / beginner path / technical reference / testing / migration flow.
- [x] All source headers use the LMB Doxygen file-header style.
- [x] Existing four-example progression is retained.
- [x] Example GPIO choices remain appropriate for the ESP32-S3 target.
- [x] Example 04 demonstrates one coherent expander snapshot.
- [x] `keywords.txt`, package metadata, and repository tree match the public API.

## Automated validation performed locally

- [x] Native C++11 deterministic suite with GCC-compatible compiler.
- [x] Native C++11 deterministic suite with Clang when available.
- [x] `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror`.
- [x] AddressSanitizer + UndefinedBehaviorSanitizer.
- [x] Strict host syntax compile of public examples.
- [x] PlatformIO portable compile for every claimed board environment.
- [x] PlatformIO compile of all public examples for ESP32-S3.
- [x] Release-contract/package-hygiene checks.
- [x] Final release ZIP unpacked and retested from a clean directory.

## CI / hardware gates

- [ ] GitHub Actions portable PlatformIO matrix completed remotely.
- [ ] Physical ESP32-S3 GPIO debounce/edge smoke test.
- [ ] Physical MCP23017 coherent packed-read test.
- [ ] Physical MCP23017 disconnect/recovery validation with a driver capable of reporting acquisition failure.

Hardware and remote CI gates must not be reported as passed until they have actually run.
