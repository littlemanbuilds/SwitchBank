/**
 * MIT License
 *
 * @brief Generic N-bit switch bank with explicit acquisition validity, per-bit debouncing,
 *        polarity control, coherent packed reads, and durable state metadata.
 *
 * @file SwitchBank.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-01
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include "SwitchBank_Compatibility.h"
#include "SwitchBank_Handler.h"

// ---- Version ---- //

#define SWITCHBANK_VERSION "1.2.1"
#define SWITCHBANK_VERSION_MAJOR 1
#define SWITCHBANK_VERSION_MINOR 2
#define SWITCHBANK_VERSION_PATCH 1


/**
 * @brief Acquisition error reported by SwitchBank.
 */
enum class SwitchBankReadError : uint8_t
{
    None = 0,         ///< The most recent acquisition succeeded.
    MissingReader,    ///< No hardware reader is configured.
    AcquisitionFailed ///< The configured validity-aware reader reported failure.
};

/**
 * @brief Result from a validity-aware per-input electrical reader.
 *
 * The level is electrical, not logical: true means HIGH and false means LOW.
 * When valid is false, level_high is ignored and SwitchBank keeps its last stable state.
 */
struct SwitchBankReadResult
{
    bool level_high; ///< Electrical input level when valid is true.
    bool valid;      ///< True when the hardware acquisition succeeded.

    /// @brief Construct an electrical sample; defaults to an invalid result.
    ///
    /// @param high Electrical HIGH when true, LOW when false; ignored if invalid.
    /// @param is_valid Whether the acquisition succeeded.
    SwitchBankReadResult(bool high = false, bool is_valid = false) noexcept
        : level_high{high}, valid{is_valid} {}

    /// @brief Build a successful electrical-level result.
    ///
    /// @param high Electrical HIGH when true, LOW when false.
    /// @return A valid acquisition result containing the supplied electrical levels.
    static SwitchBankReadResult success(bool high) noexcept { return SwitchBankReadResult(high, true); }

    /// @brief Build a failed acquisition result.
    ///
    /// @return An invalid acquisition result whose level payload must be ignored.
    static SwitchBankReadResult failure() noexcept { return SwitchBankReadResult(false, false); }
};

/**
 * @brief Result from one coherent packed hardware acquisition.
 *
 * Bit i is the electrical level for bank input i before polarity normalization and before
 * optional ReverseOrder packing. This allows an expander or hardware snapshot to be read
 * once and then decoded consistently by SwitchBank.
 */
struct SwitchBankPackedReadResult
{
    uint32_t levels_high; ///< Electrical HIGH mask in bank-input order.
    bool valid;           ///< True when the complete packed acquisition succeeded.

    /// @brief Construct a coherent electrical sample; defaults to invalid.
    ///
    /// @param levels Electrical HIGH bits in input order; ignored if invalid.
    /// @param is_valid Whether the entire acquisition succeeded.
    SwitchBankPackedReadResult(uint32_t levels = 0u, bool is_valid = false) noexcept
        : levels_high{levels}, valid{is_valid} {}

    /// @brief Build a successful packed electrical-level result.
    ///
    /// @param levels Electrical HIGH bits in bank-input order.
    /// @return A valid acquisition result containing the supplied electrical levels.
    static SwitchBankPackedReadResult success(uint32_t levels) noexcept
    {
        return SwitchBankPackedReadResult(levels, true);
    }

    /// @brief Build a failed packed acquisition result.
    ///
    /// @return An invalid acquisition result whose level payload must be ignored.
    static SwitchBankPackedReadResult failure() noexcept
    {
        return SwitchBankPackedReadResult(0u, false);
    }
};

/**
 * @brief Health and freshness metadata for the latest SwitchBank acquisition.
 */
struct SwitchBankStatus
{
    bool configured{false};                               ///< True when a hardware reader callback is configured.
    bool has_sample{false};                               ///< True after at least one successful hardware acquisition.
    bool valid{false};                                    ///< True when the most recent attempted acquisition succeeded.
    SwitchBankReadError error{SwitchBankReadError::None}; ///< Most recent acquisition error.
    uint32_t sample_ms{0u};                               ///< Timestamp of the last successful acquisition.
    uint32_t attempt_ms{0u};                              ///< Timestamp of the most recent attempted acquisition.
    uint32_t error_ms{0u};                                ///< Timestamp of the most recent failed acquisition.
    uint32_t sequence{0u};                                ///< Increments on every successful hardware acquisition.
    uint32_t change_ms{0u};                               ///< Timestamp of the most recent stable state transition.
    uint32_t change_sequence{0u};                         ///< Increments only when the stable packed state changes.
    uint32_t generation{0u};                              ///< Increments after each successful explicit sync/resynchronization.
};

/**
 * @brief Explicitly consumed edge information.
 */
struct SwitchBankEdges
{
    uint32_t changed{0u};         ///< Bits that changed on the latest retained transition.
    uint32_t rising{0u};          ///< Bits that changed from OFF to ON.
    uint32_t falling{0u};         ///< Bits that changed from ON to OFF.
    uint32_t change_sequence{0u}; ///< Durable transition sequence associated with the edge state.
};

/**
 * @brief Generic debounced switch bank.
 *
 * @tparam N Number of switches (1..32).
 * @tparam PolarityMask Compile-time active-low mask (bit=1 -> active-low). Use -1 for runtime polarity.
 * @tparam ReverseOrder If true, bit packing is reversed (bit 0 corresponds to keys[N-1]).
 *
 * Electrical vs logical:
 * - Readers return the electrical level for each input (true = HIGH, false = LOW).
 * - SwitchBank normalizes that level to logical ON using the active-low mask.
 * - Validity-aware and packed readers may reject an acquisition without altering stable state.
 *
 * SwitchBank is latest-state storage with transition metadata. It is not an event queue. Consumers
 * that cannot afford to miss individual transitions should use change_sequence to detect a gap and
 * transport events with an application-level queue/counter as appropriate.
 */
template <size_t N, int64_t PolarityMask = -1, bool ReverseOrder = false>
class SwitchBank : public SwitchBankHandler
{
    static_assert(N > 0 && N <= 32, "SwitchBank<N>: N must be in 1..32");

public:
    using SwitchBankHandler::update; ///< Bring no-argument update() into scope.

    /// @brief Legacy per-key electrical reader: key -> HIGH/LOW. Cannot report acquisition failure.
    using ReadPinFn = bool (*)(uint8_t /*key*/);

    /// @brief Legacy context-aware electrical reader. Cannot report acquisition failure.
    using ReadFn = bool (*)(void * /*ctx*/, uint8_t /*key*/);

    /// @brief Validity-aware per-key electrical reader.
    using ReadResultFn = SwitchBankReadResult (*)(void * /*ctx*/, uint8_t /*key*/);

    /// @brief Coherent packed electrical reader. Bit i corresponds to bank input i.
    using PackedReadFn = SwitchBankPackedReadResult (*)(void * /*ctx*/);

    /**
     * @brief Latch policy for the legacy changed() flag.
     *
     * @note ManualClear is recommended. ClearOnRead is retained for v1.x compatibility; new code
     *       should prefer consumeChanged() or consumeEdges() so observation is explicit.
     */
    enum class LatchMode : uint8_t
    {
        ManualClear, ///< changed() remains latched until explicitly consumed/cleared.
        ClearOnRead  ///< Legacy behaviour: value() clears the changed latch.
    };

    /**
     * @brief Plain snapshot of stable state plus acquisition metadata.
     */
    struct SwitchBankSnapshot
    {
        uint32_t value;            ///< Current stable logical packed value.
        uint32_t changed;          ///< Bits changed on the latest retained transition.
        uint32_t rising;           ///< Bits that rose 0->1 on that transition.
        uint32_t falling;          ///< Bits that fell 1->0 on that transition.
        uint32_t t_ms;             ///< Compatibility alias for change_ms.
        uint32_t seq;              ///< Successful acquisition sequence.
        uint32_t sample_ms;        ///< Last successful acquisition timestamp.
        uint32_t change_ms;        ///< Last stable transition timestamp.
        uint32_t change_seq;       ///< Stable transition sequence.
        uint32_t generation;       ///< Successful resynchronization generation.
        bool valid;                ///< Whether the most recent attempted acquisition succeeded.
        SwitchBankReadError error; ///< Most recent acquisition error.
    };

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    /**
     * @brief Optional callback invoked after a stable state transition is committed.
     *
     * @note Called in the caller's context; keep it fast and non-blocking.
     */
    using OnCommitFn = void (*)(const SwitchBankSnapshot &) noexcept;
#endif

    // ---- Construction: legacy bool readers ---- //

    /**
     * @brief Construct with a legacy per-key reader.
     *
     * @param keys Array of N keys (pins or expander identifiers).
     * @param stable_ms Debounce window in milliseconds (0 disables debouncing).
     * @param active_low_mask Runtime polarity mask (bit=1 -> active-low).
     * @param read Reader returning electrical HIGH/LOW. A null reader is invalid and fails closed.
     * @param timeFn Optional millisecond time source for no-argument operations.
     */
    SwitchBank(const uint8_t (&keys)[N],
               uint16_t stable_ms = 0,
               uint32_t active_low_mask = defaultActiveLowMaskRuntime(),
               ReadPinFn read = nullptr,
               TimeFn timeFn = nullptr) noexcept
        : SwitchBankHandler(timeFn),
          read_pin_fn_{read},
          debounce_ms_{stable_ms},
          active_low_mask_{resolveInitialMask(active_low_mask)}
    {
        copyKeys(keys);
    }

    /**
     * @brief Construct with a legacy context-aware per-key reader.
     *
     * @param keys Array of N keys.
     * @param stable_ms Debounce window in milliseconds.
     * @param active_low_mask Runtime polarity mask.
     * @param read Reader returning electrical HIGH/LOW. A null reader is invalid and fails closed.
     * @param ctx Opaque reader context.
     * @param timeFn Optional millisecond time source.
     */
    SwitchBank(const uint8_t (&keys)[N],
               uint16_t stable_ms,
               uint32_t active_low_mask,
               ReadFn read,
               void *ctx,
               TimeFn timeFn = nullptr) noexcept
        : SwitchBankHandler(timeFn),
          read_fn_{read},
          ctx_{ctx},
          debounce_ms_{stable_ms},
          active_low_mask_{resolveInitialMask(active_low_mask)}
    {
        copyKeys(keys);
    }

    /**
     * @brief Construct an explicitly unconfigured context-reader bank.
     *
     * @note This overload resolves nullptr source compatibility after v1.2 added additional
     *       callback types. sync()/update() will report MissingReader until a configured bank
     *       instance is used.
     *
     * @param keys N identifiers copied into the bank.
     * @param stable_ms Debounce window in milliseconds; zero disables it.
     * @param active_low_mask Runtime active-low bits in logical packed order.
     * @param ctx Unowned context stored for compatibility; no reader will use it.
     * @param timeFn Optional millisecond clock.
     */
    SwitchBank(const uint8_t (&keys)[N],
               uint16_t stable_ms,
               uint32_t active_low_mask,
               decltype(nullptr),
               void *ctx,
               TimeFn timeFn = nullptr) noexcept
        : SwitchBankHandler(timeFn),
          ctx_{ctx},
          debounce_ms_{stable_ms},
          active_low_mask_{resolveInitialMask(active_low_mask)}
    {
        copyKeys(keys);
    }

    // ---- Construction: explicit validity readers ---- //

    /**
     * @brief Construct with a validity-aware context reader.
     *
     * @param keys Array of N keys.
     * @param stable_ms Debounce window in milliseconds.
     * @param active_low_mask Runtime polarity mask.
     * @param read Reader returning an explicit valid/invalid electrical sample.
     * @param ctx Opaque reader context.
     * @param timeFn Optional millisecond time source.
     */
    SwitchBank(const uint8_t (&keys)[N],
               uint16_t stable_ms,
               uint32_t active_low_mask,
               ReadResultFn read,
               void *ctx,
               TimeFn timeFn = nullptr) noexcept
        : SwitchBankHandler(timeFn),
          read_result_fn_{read},
          ctx_{ctx},
          debounce_ms_{stable_ms},
          active_low_mask_{resolveInitialMask(active_low_mask)}
    {
        copyKeys(keys);
    }

    /**
     * @brief Construct with one coherent packed reader.
     *
     * @param keys Array of N key identifiers retained for API consistency. The packed callback
     *             returns bits in bank-input order and therefore does not receive these keys.
     * @param stable_ms Debounce window in milliseconds.
     * @param active_low_mask Runtime polarity mask.
     * @param read Packed reader returning one valid/invalid electrical snapshot.
     * @param ctx Opaque reader context.
     * @param timeFn Optional millisecond time source.
     */
    SwitchBank(const uint8_t (&keys)[N],
               uint16_t stable_ms,
               uint32_t active_low_mask,
               PackedReadFn read,
               void *ctx,
               TimeFn timeFn = nullptr) noexcept
        : SwitchBankHandler(timeFn),
          packed_read_fn_{read},
          ctx_{ctx},
          debounce_ms_{stable_ms},
          active_low_mask_{resolveInitialMask(active_low_mask)}
    {
        copyKeys(keys);
    }

    // ---- Synchronization ---- //

    /**
     * @brief Synchronize to current hardware without emitting switch edges.
     *
     * @return true when a complete valid acquisition established the baseline.
     */
    bool sync() noexcept { return sync(now_ms()); }

    /**
     * @brief Synchronize to current hardware without emitting edges.
     *
     * @param now Timestamp in milliseconds.
     * @return true on valid acquisition; false on missing/failed hardware read.
     *
     * A successful sync increments generation and acquisition sequence, but never resets sequence
     * or change_sequence. A failed sync leaves the previous stable value and debounce history intact.
     */
    bool sync(uint32_t now) noexcept
    {
        uint32_t raw = 0u;
        if (!acquireLogicalPacked_(now, raw))
        {
            return false;
        }

        previous_ = current_ = raw;
        seedDebounce_(raw, now);
        acquisition_gap_ = false;
        changed_ = false;
        ++generation_;
        last_poll_ms_ = now;
        has_poll_timestamp_ = true;
        return true;
    }

    // ---- Core API ---- //

    /**
     * @brief Return a plain snapshot of current stable state and health metadata.
     *
     * @return A copy of stable state, retained edges and health metadata; no acquisition occurs.
     */
    SWITCHBANK_NODISCARD SwitchBankSnapshot snapshot() const noexcept
    {
        return SwitchBankSnapshot{
            current_,
            changedMask(),
            risingMask(),
            fallingMask(),
            last_change_ms_,
            sequence_,
            last_sample_ms_,
            last_change_ms_,
            change_sequence_,
            generation_,
            valid_,
            last_error_};
    }

    /** @brief Number of switches in the bank. */
    static constexpr uint8_t kSize = static_cast<uint8_t>(N);

    /**
     * @brief Replace or inject the millisecond time source.
     *
     * @param tf Optional millisecond clock; null restores the default clock behavior.
     */
    void setTimeSource(TimeFn tf) noexcept { setTimeFn(tf); }

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    /**
     * @brief Set the optional state-transition callback.
     *
     * @param cb Callback invoked in the caller context after a stable transition; null disables it.
     */
    void setOnCommit(OnCommitFn cb) noexcept { on_commit_ = cb; }
#endif

    /**
     * @brief Set the minimum interval between hardware acquisition attempts.
     *
     * @param ms Minimum milliseconds between attempts; zero disables throttling.
     */
    void setMinPollMs(uint16_t ms) noexcept { min_poll_ms_ = ms; }

    /**
     * @brief Configure legacy changed-latch behaviour.
     *
     * @param m ManualClear or legacy ClearOnRead policy for changed(), not edge masks.
     */
    void setLatchMode(LatchMode m) noexcept { latch_mode_ = m; }

    /**
     * @brief Set the per-bit debounce window.
     *
     * @param ms Required observed stability in milliseconds; zero disables debounce.
     */
    void setDebounceMs(uint16_t ms) noexcept { debounce_ms_ = ms; }

    /**
     * @brief Poll hardware and update the debounced state.
     *
     * @param now_ms Timestamp in milliseconds.
     * @return true only when the stable packed value changed.
     *
     * A successful acquisition increments sequence and sample_ms even when the stable state does not
     * change. Failed acquisitions set valid=false and leave stable/debounce state untouched.
     */
    bool update(uint32_t now_ms) override
    {
        if ((min_poll_ms_ != 0u) && has_poll_timestamp_)
        {
            const uint32_t delta = now_ms - last_poll_ms_;
            if (delta < min_poll_ms_)
            {
                return false;
            }
        }
        last_poll_ms_ = now_ms;
        has_poll_timestamp_ = true;

        uint32_t raw = 0u;
        if (!acquireLogicalPacked_(now_ms, raw))
        {
            return false;
        }

        uint32_t next = current_;
        const bool recovering_from_gap = acquisition_gap_;
        acquisition_gap_ = false;

        // Integrate each logical input independently through its debounce window.
        for (uint8_t i = 0u; i < static_cast<uint8_t>(N); ++i)
        {
            const bool r = ((raw >> i) & 1u) != 0u;
            BitState &b = bits_[i];

            // Time spent without valid hardware samples cannot count as debounce evidence.
            // Rebase the candidate window on the first valid sample after an acquisition gap.
            if (recovering_from_gap)
            {
                b.last_raw = r;
                b.last_change_ms = now_ms;
            }
            else if (r != b.last_raw)
            {
                b.last_raw = r;
                b.last_change_ms = now_ms;
            }

            if ((now_ms - b.last_change_ms) >= debounce_ms_ && b.stable != r)
            {
                b.stable = r;
            }

            if (b.stable)
            {
                next |= (uint32_t{1} << i);
            }
            else
            {
                next &= ~(uint32_t{1} << i);
            }
        }

        return commitIfChanged_(next, now_ms);
    }

    /**
     * @brief Re-acquire and immediately accept the current hardware state, bypassing debounce.
     *
     * @param now Timestamp in milliseconds.
     * @return true when the hardware acquisition succeeded.
     * @note Intended for commissioning/reconfiguration, not normal runtime input processing.
     */
    bool forceCommitForCommissioning(uint32_t now) noexcept
    {
        last_poll_ms_ = now;
        has_poll_timestamp_ = true;

        uint32_t raw = 0u;
        if (!acquireLogicalPacked_(now, raw))
        {
            return false;
        }

        seedDebounce_(raw, now);
        acquisition_gap_ = false;
        (void)commitIfChanged_(raw, now);
        return true;
    }

    /**
     * @brief Re-acquire and immediately accept the current hardware state using the configured clock.
     *
     * @return true when the hardware acquisition succeeded.
     */
    bool forceCommitForCommissioning() noexcept { return forceCommitForCommissioning(now_ms()); }

#ifndef SWITCHBANK_NO_LEGACY_COMMIT
    /**
     * @brief Legacy alias for forceCommitForCommissioning().
     *
     * @note Retained for v1.x source compatibility. New code should use the explicit commissioning name.
     */
    SWITCHBANK_DEPRECATED("Use forceCommitForCommissioning(); runtime code should normally use update().")
    void commit() noexcept
    {
        (void)forceCommitForCommissioning();
    }

    /**
     * @brief Legacy timestamped alias for forceCommitForCommissioning(now).
     *
     * @param now Acquisition timestamp in milliseconds.
     */
    SWITCHBANK_DEPRECATED("Use forceCommitForCommissioning(now); runtime code should normally use update(now).")
    void commit(uint32_t now) noexcept
    {
        (void)forceCommitForCommissioning(now);
    }
#endif

    /**
     * @brief Current stable packed value.
     *
     * @note In legacy LatchMode::ClearOnRead, this also clears changed(). Prefer peekValue() for
     *       side-effect-free reads and consumeChanged()/consumeEdges() for explicit consumption.
     *
     * @return Stable logical packed value; ClearOnRead also clears the changed latch.
     */
    SWITCHBANK_NODISCARD uint32_t value() const noexcept override
    {
        if (latch_mode_ == LatchMode::ClearOnRead)
        {
            changed_ = false;
        }
        return current_;
    }

    /**
     * @brief Current stable packed value without side effects.
     *
     * @return Stable logical packed value without consuming edges or the changed latch.
     */
    SWITCHBANK_NODISCARD uint32_t peekValue() const noexcept override { return current_; }

    /**
     * @brief Previous stable value associated with the retained edge masks.
     *
     * @return Baseline for retained edges, reset to the current value by sync() or clearEdges().
     */
    SWITCHBANK_NODISCARD uint32_t prevValue() const noexcept override { return previous_; }

    /**
     * @brief Whether a stable transition is currently latched.
     *
     * @return True while a stable transition remains latched; clearing edges alone does not clear it.
     */
    SWITCHBANK_NODISCARD bool changed() const noexcept override { return changed_; }

    /**
     * @brief Explicitly consume and clear the changed latch.
     *
     * @return The previous latch state; retained edge masks are unchanged.
     */
    bool consumeChanged() noexcept
    {
        const bool was_changed = changed_;
        changed_ = false;
        return was_changed;
    }

    /**
     * @brief Explicitly consume retained edge masks and changed latch.
     *
     * @return Edge masks and the durable change_sequence observed at consumption time.
     */
    SwitchBankEdges consumeEdges() noexcept
    {
        SwitchBankEdges edges;
        edges.changed = changedMask();
        edges.rising = risingMask();
        edges.falling = fallingMask();
        edges.change_sequence = change_sequence_;
        previous_ = current_;
        changed_ = false;
        return edges;
    }

    /** @brief Clear only the changed latch. */
    void clearChanged() noexcept override { changed_ = false; }

    /** @brief Clear retained edge masks without changing the current stable value. */
    void clearEdges() noexcept override { previous_ = current_; }

    /**
     * @brief Number of switches in this bank.
     *
     * @return Number of inputs, in the range 1..32.
     */
    SWITCHBANK_NODISCARD uint8_t size() const noexcept override { return static_cast<uint8_t>(N); }

    /**
     * @brief Bitwise delta for the retained latest stable transition.
     *
     * @return Bits differing between the current value and retained previous value.
     */
    SWITCHBANK_NODISCARD uint32_t changedMask() const noexcept override { return current_ ^ previous_; }

    /**
     * @brief Retained OFF->ON transition mask.
     *
     * @return Bits that changed from logical OFF to ON in the retained transition.
     */
    SWITCHBANK_NODISCARD uint32_t risingMask() const noexcept override { return (~previous_) & current_ & valueMask_(); }

    /**
     * @brief Retained ON->OFF transition mask.
     *
     * @return Bits that changed from logical ON to OFF in the retained transition.
     */
    SWITCHBANK_NODISCARD uint32_t fallingMask() const noexcept override { return previous_ & (~current_) & valueMask_(); }

    /**
     * @brief Check whether bit i is currently ON.
     *
     * @param i Logical packed-bit index (0..N-1).
     * @return True if logical bit i is ON; false for an out-of-range index.
     */
    SWITCHBANK_NODISCARD bool isOn(uint8_t i) const noexcept
    {
        return (i < static_cast<uint8_t>(N)) && (((current_ >> i) & 1u) != 0u);
    }

    /**
     * @brief Check whether bit i rose on the retained transition.
     *
     * @param i Logical packed-bit index (0..N-1).
     * @return True if logical bit i rose in the retained transition; false out of range.
     */
    SWITCHBANK_NODISCARD bool rose(uint8_t i) const noexcept
    {
        return (i < static_cast<uint8_t>(N)) && (((risingMask() >> i) & 1u) != 0u);
    }

    /**
     * @brief Check whether bit i fell on the retained transition.
     *
     * @param i Logical packed-bit index (0..N-1).
     * @return True if logical bit i fell in the retained transition; false out of range.
     */
    SWITCHBANK_NODISCARD bool fell(uint8_t i) const noexcept
    {
        return (i < static_cast<uint8_t>(N)) && (((fallingMask() >> i) & 1u) != 0u);
    }

    /**
     * @brief Change runtime polarity and resynchronize atomically from the caller's perspective.
     *
     * @param mask New active-low mask.
     * @return true when the new interpretation was established from a valid hardware sample.
     * @note No-op returning true when compile-time polarity is used. On acquisition failure, the
     *       previous runtime mask is restored and the stable value is preserved.
     */
    bool setActiveLowMask(uint32_t mask) noexcept
    {
        return setActiveLowMask(mask, now_ms());
    }

    /**
     * @brief Timestamped overload of setActiveLowMask().
     *
     * @param mask New active-low bits in logical packed order; bits above N are ignored.
     * @param now Acquisition timestamp in milliseconds.
     * @return True after successful resynchronization or a compile-time-policy no-op; false restores the old mask and stable state.
     */
    bool setActiveLowMask(uint32_t mask, uint32_t now) noexcept
    {
        if (hasCompileTimePolarity())
        {
            return true;
        }

        const uint32_t previous_mask = active_low_mask_;
        active_low_mask_ = mask & valueMask_();
        if (sync(now))
        {
            return true;
        }

        active_low_mask_ = previous_mask;
        return false;
    }

    /**
     * @brief Effective active-low mask.
     *
     * @return Effective polarity mask in logical packed-bit order; bits above N are zero.
     */
    SWITCHBANK_NODISCARD uint32_t activeLowMask() const noexcept { return effectiveMask(); }

    // ---- Validity / freshness ---- //

    /**
     * @brief Whether any supported hardware reader is configured.
     *
     * @return True when a reader is configured, without probing hardware connectivity.
     */
    SWITCHBANK_NODISCARD bool configured() const noexcept { return hasReader_(); }

    /**
     * @brief Whether the most recent attempted hardware acquisition succeeded.
     *
     * @return True if the most recent attempted acquisition succeeded; false before the first sample.
     */
    SWITCHBANK_NODISCARD bool valid() const noexcept override { return valid_; }

    /**
     * @brief Whether at least one successful hardware acquisition has occurred.
     *
     * @return True after any successful acquisition, even if a later acquisition fails.
     */
    SWITCHBANK_NODISCARD bool hasSample() const noexcept { return has_sample_; }

    /**
     * @brief Most recent acquisition error.
     *
     * @return Error from the latest acquisition attempt; None after success or before any attempt.
     */
    SWITCHBANK_NODISCARD SwitchBankReadError lastError() const noexcept { return last_error_; }

    /**
     * @brief Timestamp of the latest successful acquisition.
     *
     * @return Latest successful acquisition time in milliseconds, or zero before a sample.
     */
    SWITCHBANK_NODISCARD uint32_t sampleMs() const noexcept override { return last_sample_ms_; }

    /**
     * @brief Timestamp of the latest stable state transition.
     *
     * @return Latest stable transition time in milliseconds, or zero before a transition.
     */
    SWITCHBANK_NODISCARD uint32_t changeMs() const noexcept { return last_change_ms_; }

    /**
     * @brief Timestamp of the latest acquisition attempt.
     *
     * @return Latest acquisition attempt time in milliseconds, or zero before an attempt.
     */
    SWITCHBANK_NODISCARD uint32_t lastAttemptMs() const noexcept { return last_attempt_ms_; }

    /**
     * @brief Timestamp of the latest failed acquisition.
     *
     * @return Latest failed acquisition time in milliseconds, or zero before a failure.
     */
    SWITCHBANK_NODISCARD uint32_t lastErrorMs() const noexcept { return last_error_ms_; }

    /**
     * @brief Successful acquisition sequence; monotonic across sync().
     *
     * @return Successful acquisition count modulo 2^32; sync() increments it without resetting it.
     */
    SWITCHBANK_NODISCARD uint32_t sequence() const noexcept override { return sequence_; }

    /**
     * @brief Stable transition sequence; monotonic across sync().
     *
     * @return Stable transition count modulo 2^32; sync() does not increment or reset it.
     */
    SWITCHBANK_NODISCARD uint32_t changeSequence() const noexcept override { return change_sequence_; }

    /**
     * @brief Successful resynchronization generation.
     *
     * @return Successful explicit resynchronization count modulo 2^32.
     */
    SWITCHBANK_NODISCARD uint32_t generation() const noexcept { return generation_; }

    /**
     * @brief Gather health/freshness metadata without changing state.
     *
     * @return A copy of health and freshness metadata; no acquisition or consumption occurs.
     */
    SWITCHBANK_NODISCARD SwitchBankStatus status() const noexcept
    {
        SwitchBankStatus st;
        st.configured = hasReader_();
        st.has_sample = has_sample_;
        st.valid = valid_;
        st.error = last_error_;
        st.sample_ms = last_sample_ms_;
        st.attempt_ms = last_attempt_ms_;
        st.error_ms = last_error_ms_;
        st.sequence = sequence_;
        st.change_ms = last_change_ms_;
        st.change_sequence = change_sequence_;
        st.generation = generation_;
        return st;
    }

    /**
     * @brief Compatibility alias for the last stable transition timestamp.
     *
     * @note Use sampleMs() for freshness. A stable, healthy bank may have an old change timestamp.
     *
     * @return Latest stable transition time in milliseconds, not the latest sample time.
     */
    SWITCHBANK_NODISCARD uint32_t lastCommitMs() const noexcept { return last_change_ms_; }

    /**
     * @brief Total stable state transitions since construction.
     *
     * @note Unlike v1.1, this is not reset by sync(). Use generation() to identify resynchronization.
     *
     * @return Stable transition count modulo 2^32, retained across sync().
     */
    SWITCHBANK_NODISCARD uint32_t changeCount() const noexcept { return change_sequence_; }

    /** @brief value() as uint8_t for N <= 8.
     *
     * @tparam M Accessor width constraint; leave at its default N. Enabled for M <= 8.
     * @return value() narrowed to eight bits, including its legacy latch side effect.
     */
    template <size_t M = N>
    typename switchbank::compat::enable_if<(M <= 8), uint8_t>::type value8() const noexcept
    {
        return static_cast<uint8_t>(value());
    }

    /** @brief changedMask() as uint8_t for N <= 8.
     *
     * @tparam M Accessor width constraint; leave at its default N. Enabled for M <= 8.
     * @return Retained changed bits narrowed to eight bits.
     */
    template <size_t M = N>
    typename switchbank::compat::enable_if<(M <= 8), uint8_t>::type changedMask8() const noexcept
    {
        return static_cast<uint8_t>(changedMask());
    }

private:
    struct BitState
    {
        uint32_t last_change_ms{0u}; ///< Timestamp when raw logical input last toggled.
        bool last_raw{false};        ///< Latest acquired logical level.
        bool stable{false};          ///< Current debounced logical level.
    };

    // ---- Compile-time / runtime polarity ---- //

    /// @brief Select the compile-time polarity policy.
    ///
    /// @return True if PolarityMask is nonnegative; -1 selects runtime polarity.
    static constexpr bool hasCompileTimePolarity() noexcept { return (PolarityMask >= 0); }

    /// @brief Limit state to N bits without a shift by the integer width.
    ///
    /// @return Low N bits set, including all 32 bits when N is 32.
    static constexpr uint32_t valueMask_() noexcept
    {
        return (N == 32) ? 0xFFFFFFFFu : ((uint32_t{1} << N) - 1u);
    }

    /// @brief Normalize compile-time polarity to the bank width.
    ///
    /// @return Compile-time active-low bits, or zero for runtime polarity.
    static constexpr uint32_t compileTimeMask() noexcept
    {
        return hasCompileTimePolarity() ? (static_cast<uint32_t>(PolarityMask) & valueMask_()) : 0u;
    }

    /// @brief Choose active-low polarity for every input by default.
    ///
    /// @return Low N bits set.
    static constexpr uint32_t defaultActiveLowMaskRuntime() noexcept { return valueMask_(); }

    /// @brief Resolve constructor polarity with compile-time policy taking precedence.
    ///
    /// @param runtime_mask Requested runtime active-low bits in logical order.
    /// @return Selected polarity mask truncated to N bits.
    static constexpr uint32_t initialMaskForCtor(uint32_t runtime_mask) noexcept
    {
        return hasCompileTimePolarity() ? compileTimeMask() : (runtime_mask & valueMask_());
    }

    /// @brief Resolve the initial stored polarity mask.
    ///
    /// @param runtime_mask Requested runtime active-low bits in logical order.
    /// @return Compile-time mask when configured, otherwise truncated runtime mask.
    uint32_t resolveInitialMask(uint32_t runtime_mask) const noexcept
    {
        return initialMaskForCtor(runtime_mask);
    }

    /// @brief Read the selected polarity policy.
    ///
    /// @return Active-low mask in logical packed-bit order, truncated to N bits.
    uint32_t effectiveMask() const noexcept
    {
        return hasCompileTimePolarity() ? compileTimeMask() : (active_low_mask_ & valueMask_());
    }

    // ---- Hardware acquisition ---- //

    // Reader contexts and function pointers are borrowed. Callbacks must remain valid
    // for the bank lifetime; callers serialize all acquisition and mutation operations.

    /// @brief Copy identifiers so the bank does not borrow the caller array.
    ///
    /// @param keys N input identifiers copied in acquisition order.
    void copyKeys(const uint8_t (&keys)[N]) noexcept
    {
        for (size_t i = 0u; i < N; ++i)
        {
            keys_[i] = keys[i];
        }
    }

    /// @brief Check reader configuration without attempting hardware access.
    ///
    /// @return True when at least one reader callback is non-null.
    bool hasReader_() const noexcept
    {
        return (packed_read_fn_ != nullptr) || (read_result_fn_ != nullptr) ||
               (read_pin_fn_ != nullptr) || (read_fn_ != nullptr);
    }

    /**
     * @brief Acquire all electrical inputs exactly once.
     *
     * @param levels_high Output electrical HIGH mask in bank-input order.
     * @return true when the complete bank acquisition is valid.
     */
    bool acquireElectrical_(uint32_t &levels_high) const noexcept
    {
        levels_high = 0u;

        if (packed_read_fn_ != nullptr)
        {
            const SwitchBankPackedReadResult result = packed_read_fn_(ctx_);
            if (!result.valid)
            {
                return false;
            }
            levels_high = result.levels_high & valueMask_();
            return true;
        }

        if (read_result_fn_ != nullptr)
        {
            for (uint8_t i = 0u; i < static_cast<uint8_t>(N); ++i)
            {
                const SwitchBankReadResult result = read_result_fn_(ctx_, keys_[i]);
                if (!result.valid)
                {
                    return false;
                }
                if (result.level_high)
                {
                    levels_high |= (uint32_t{1} << i);
                }
            }
            return true;
        }

        if (read_pin_fn_ != nullptr)
        {
            for (uint8_t i = 0u; i < static_cast<uint8_t>(N); ++i)
            {
                if (read_pin_fn_(keys_[i]))
                {
                    levels_high |= (uint32_t{1} << i);
                }
            }
            return true;
        }

        if (read_fn_ != nullptr)
        {
            for (uint8_t i = 0u; i < static_cast<uint8_t>(N); ++i)
            {
                if (read_fn_(ctx_, keys_[i]))
                {
                    levels_high |= (uint32_t{1} << i);
                }
            }
            return true;
        }

        return false;
    }

    /**
     * @brief Acquire, normalize polarity/order, and update freshness metadata.
     *
     * @param now Timestamp in milliseconds.
     * @param logical_packed Output logical packed value when successful.
     * @return True only for a complete valid sample; output is unchanged on failure.
     */
    bool acquireLogicalPacked_(uint32_t now, uint32_t &logical_packed) noexcept
    {
        last_attempt_ms_ = now;

        if (!hasReader_())
        {
            recordFailure_(now, SwitchBankReadError::MissingReader);
            return false;
        }

        uint32_t electrical = 0u;
        if (!acquireElectrical_(electrical))
        {
            recordFailure_(now, SwitchBankReadError::AcquisitionFailed);
            return false;
        }

        // Electrical bits are in input order; polarity is indexed after reverse packing.
        const uint32_t mask = effectiveMask();
        uint32_t logical = 0u;
        const uint8_t n8 = static_cast<uint8_t>(N);

        for (uint8_t i = 0u; i < n8; ++i)
        {
            const uint8_t bi = ReverseOrder ? static_cast<uint8_t>(n8 - 1u - i) : i;
            const bool level_high = ((electrical >> i) & 1u) != 0u;
            const bool active_low = ((mask >> bi) & 1u) != 0u;
            const bool on = active_low ? !level_high : level_high;
            if (on)
            {
                logical |= (uint32_t{1} << bi);
            }
        }

        logical_packed = logical & valueMask_();
        recordSuccess_(now);
        return true;
    }

    /// @brief Record successful acquisition without creating a state transition.
    ///
    /// @param now Successful acquisition time in milliseconds.
    void recordSuccess_(uint32_t now) noexcept
    {
        has_sample_ = true;
        valid_ = true;
        last_error_ = SwitchBankReadError::None;
        last_sample_ms_ = now;
        ++sequence_;
    }

    /// @brief Record a failed read and require fresh debounce evidence on recovery.
    ///
    /// @param now Failed acquisition time in milliseconds.
    /// @param error MissingReader or AcquisitionFailed reason.
    void recordFailure_(uint32_t now, SwitchBankReadError error) noexcept
    {
        valid_ = false;
        last_error_ = error;
        last_error_ms_ = now;
        acquisition_gap_ = true;
    }

    /// @brief Establish raw and stable per-bit baselines without emitting edges.
    ///
    /// @param raw Logical packed value already normalized to N bits.
    /// @param now Baseline timestamp in milliseconds.
    void seedDebounce_(uint32_t raw, uint32_t now) noexcept
    {
        for (uint8_t i = 0u; i < static_cast<uint8_t>(N); ++i)
        {
            const bool r = ((raw >> i) & 1u) != 0u;
            bits_[i].last_raw = r;
            bits_[i].stable = r;
            bits_[i].last_change_ms = now;
        }
    }

    /// @brief Commit a changed stable value before notifying the optional callback.
    ///
    /// @param next Proposed logical packed value; bits above N are ignored.
    /// @param now_ms Transition timestamp in milliseconds.
    /// @return True if state changed; unchanged input leaves edge metadata intact.
    bool commitIfChanged_(uint32_t next, uint32_t now_ms) noexcept
    {
        next &= valueMask_();
        if (next == current_)
        {
            return false;
        }

        previous_ = current_;
        current_ = next;
        changed_ = true;
        last_change_ms_ = now_ms;
        ++change_sequence_;

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
        if (on_commit_ != nullptr)
        {
            on_commit_(snapshot());
        }
#endif
        return true;
    }

    // ---- Configuration / reader hooks ---- //
    uint8_t keys_[N]{};                    ///< Key identifiers for per-key readers.
    ReadPinFn read_pin_fn_{nullptr};       ///< Legacy no-context reader.
    ReadFn read_fn_{nullptr};              ///< Legacy context reader.
    ReadResultFn read_result_fn_{nullptr}; ///< Validity-aware context reader.
    PackedReadFn packed_read_fn_{nullptr}; ///< Coherent packed reader.
    void *ctx_{nullptr};                   ///< Opaque reader context.

    uint16_t debounce_ms_{0u};                                ///< Per-bit debounce window.
    uint16_t min_poll_ms_{0u};                                ///< Minimum acquisition interval.
    LatchMode latch_mode_{LatchMode::ManualClear};            ///< Legacy changed-latch policy.
    uint32_t active_low_mask_{defaultActiveLowMaskRuntime()}; ///< Runtime polarity mask.

    BitState bits_[N]{}; ///< Per-bit debounce state.

    uint32_t current_{0u};         ///< Current stable logical packed value.
    uint32_t previous_{0u};        ///< Previous value for retained edge masks.
    mutable bool changed_{false};  ///< Legacy transition latch.
    uint32_t last_change_ms_{0u};  ///< Latest stable state-transition time.
    uint32_t change_sequence_{0u}; ///< Durable transition sequence.
    uint32_t generation_{0u};      ///< Successful sync generation.

    bool has_sample_{false};                                    ///< At least one valid acquisition occurred.
    bool valid_{false};                                         ///< Most recent attempted acquisition succeeded.
    SwitchBankReadError last_error_{SwitchBankReadError::None}; ///< Most recent acquisition error.
    uint32_t last_sample_ms_{0u};                               ///< Latest successful acquisition time.
    uint32_t last_attempt_ms_{0u};                              ///< Latest attempted acquisition time.
    uint32_t last_error_ms_{0u};                                ///< Latest failed acquisition time.
    uint32_t sequence_{0u};                                     ///< Successful acquisition sequence.
    bool acquisition_gap_{false};                               ///< True after a failed read until valid sampling resumes.

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    OnCommitFn on_commit_{nullptr}; ///< Optional stable-transition callback.
#endif

    uint32_t last_poll_ms_{0u};       ///< Latest hardware acquisition attempt used by poll throttling.
    bool has_poll_timestamp_{false};  ///< True after at least one acquisition attempt establishes min-poll timing.
};
