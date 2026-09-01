# Changelog

All notable changes to SwitchBank are documented here.

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
- Public documentation, metadata, `.gitignore`, keywords, Doxygen headers, testing, and release structure are standardized with the current LMB library pattern.
- Local PlatformIO release validation now covers every claimed board environment plus all public examples on ESP32-S3.

### Fixed

- Removed generic `LIBRARY_VERSION*` aliases from the public surface because those global macro names collide across libraries; `SWITCHBANK_VERSION*` remains authoritative.

### Deprecated

- `commit()` / `commit(now)` in favour of `forceCommitForCommissioning()`.
- `LatchMode::ClearOnRead` remains supported for v1.x compatibility; explicit consumption is preferred.

## [1.1.0]

- Previous public release baseline from the supplied project archive.
