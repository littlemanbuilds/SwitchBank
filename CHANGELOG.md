# Changelog

All notable changes to SwitchBank are documented here.

## [Unreleased]

## [1.2.1] - 2026-09-08

### Fixed

- Widen mask operands before shifts and complements on AVR/megaAVR, preserving inputs 16..31 during acquisition, polarity changes, reverse packing and debounce updates.
- Retain safe full-width handling for 32-input banks and all existing compatibility aliases.

### Validation and documentation

- Add boundary regressions for 15/16/17/31/32-input banks and target-width constant-expression checks for mixed polarity and upper-bit preservation.
- Require genuine GNU GCC, Clang and sanitizer execution in the host gate; strengthen the portable package fixture.
- Complete callable API contracts and clarify consumer adapter, example type-width and physical-validation limitations.

This date identifies release preparation; it does not record publication or physical validation.

## [1.2.0] - 2026-09-01

### Added

- Explicit `SwitchBankReadError`, `SwitchBankReadResult`, `SwitchBankPackedReadResult`, `SwitchBankStatus`, and `SwitchBankEdges` contracts.
- Validity-aware per-key readers and coherent packed-reader factories.
- `valid()`, `hasSample()`, `sampleMs()`, `lastAttemptMs()`, `lastErrorMs()`, `sequence()`, `changeSequence()`, `generation()`, and `status()`.
- Explicit `consumeChanged()` and `consumeEdges()` helpers.
- `forceCommitForCommissioning()` as the intentionally named debounce-bypass API.
- Builder support for validity-aware and packed readers.
- Dedicated deterministic native tests, sanitizer runner, release-contract checks, and GitHub Actions cross-platform compile matrix.
- Beginner-first README journey while retaining the detailed technical reference.

### Changed

- Missing readers now fail closed instead of being interpreted as electrical LOW.
- Failed acquisitions retain the last stable value and do not advance the successful-sample sequence.
- Time spent without valid acquisition can no longer satisfy the debounce window.
- `sample_ms`/successful acquisition sequence are separate from stable transition time/count.
- `sync()` now reports success, increments `generation`, and no longer resets transition history.
- `changeCount()` is monotonic across successful `sync()` calls.
- Snapshot `seq` now represents successful acquisition sequence; `change_seq` represents stable transitions.
- Runtime polarity changes are resynchronized and roll back the mask if the required read fails.
- The first `update()` is no longer blocked by `setMinPollMs()` before any acquisition timestamp exists.
- Example 04 now demonstrates the coherent packed-reader path.
- Standardized public documentation, metadata, `.gitignore`, keywords, Doxygen headers, testing, and release structure.
- Local PlatformIO release validation now covers every claimed board environment plus all public examples on ESP32-S3.

### Fixed

- Removed generic `LIBRARY_VERSION*` aliases from the public surface because those global macro names collide across libraries; `SWITCHBANK_VERSION*` remains authoritative.

### Deprecated

- `commit()` / `commit(now)` in favour of `forceCommitForCommissioning()`.
- `LatchMode::ClearOnRead` remains supported for v1.x compatibility; explicit consumption is preferred.

## [1.1.0]

- Previous public release; detailed notes were not recorded in this changelog.
