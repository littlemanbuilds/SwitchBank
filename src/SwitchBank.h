/**
 * MIT License
 *
 * @brief Generic N-bit switch bank with per-bit debouncing, optional compile-time polarity,
 *        and a pluggable time source.
 *
 * @file SwitchBank.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-01
 * @copyright Copyright © 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <SwitchBankHandler.h>

// ---- Version macro ---- //

#define SWITCHBANK_VERSION "1.0.0"

/**
 * @tparam N Number of switches (1..32).
 * @tparam PolarityMask Compile-time active-low mask (bit=1 -> active-low). Use -1 for runtime mask.
 * @tparam ReverseOrder If true, bit packing is reversed (bit 0 corresponds to keys[N-1]). Default is LSB-first.
 *
 * Electrical vs logical:
 * - Readers return the *electrical* level for a key (true = HIGH, false = LOW).
 * - The class normalizes to *logical* ON using the active-low/high polarity mask:
 *   - active-low  (mask bit = 1): LOW  => ON
 *   - active-high (mask bit = 0): HIGH => ON
 */
template <std::size_t N, std::int64_t PolarityMask = -1, bool ReverseOrder = false>
class SwitchBank : public SwitchBankHandler
{
    static_assert(N > 0 && N <= 32, "SwitchBank<N>: N must be in 1..32");

public:
    using SwitchBankHandler::update; ///< Bring no-arg update() into scope.

    /// @brief Fast per-key electrical reader: key -> electrical level (true=HIGH, false=LOW).
    using ReadPinFn = bool (*)(std::uint8_t /*key*/);

    /// @brief Context-aware electrical reader: (ctx, key) -> electrical level (true=HIGH, false=LOW).
    using ReadFn = bool (*)(void * /*ctx*/, std::uint8_t /*key*/);

    /**
     * @brief Latch policy for the "changed" flag.
     */
    enum class LatchMode : std::uint8_t
    {
        ManualClear, ///< The changed flag remains set until clearChanged() is called.
        ClearOnRead  ///< The changed flag is cleared when value() is read.
    };

    // ---- Snapshot (POD) ---- //
    struct SwitchBankSnapshot
    {
        std::uint32_t value;   ///< Stable packed value after last commit.
        std::uint32_t changed; ///< Bits that toggled on last commit.
        std::uint32_t rising;  ///< Bits that rose 0->1 on last commit.
        std::uint32_t falling; ///< Bits that fell 1->0 on last commit.
        std::uint32_t t_ms;    ///< Timestamp (ms) of last commit.
        std::uint32_t seq;     ///< Commit sequence counter.
    };

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    /**
     * @brief Optional callback invoked after a successful commit.
     *
     * The callback is called from the caller's context (inside update/commit),
     * so it should be fast and non-blocking.
     */
    using OnCommitFn = void (*)(const SwitchBankSnapshot &) noexcept;
#endif

    // ---- Construction ---- //

    /**
     * @brief Construct with a per-key reader.
     * @param keys Array of N keys (pins or expander identifiers).
     * @param stable_ms Debounce window in milliseconds (0 disables debouncing).
     * @param active_low_mask Runtime polarity (bit=1 -> active-low). Ignored if PolarityMask >= 0.
     * @param read Reader function returning the electrical level (true=HIGH, false=LOW).
     * @param timeFn Optional millisecond time source for no-arg update()/commit().
     */
    SwitchBank(const std::uint8_t (&keys)[N],
               std::uint16_t stable_ms = 0,
               std::uint32_t active_low_mask = defaultActiveLowMaskRuntime(),
               ReadPinFn read = nullptr,
               TimeFn timeFn = nullptr) noexcept
        : SwitchBankHandler(timeFn),
          read_pin_fn_{read},
          read_fn_{nullptr},
          ctx_{nullptr},
          debounce_ms_{stable_ms},
          active_low_mask_{resolveInitialMask(active_low_mask)}
    {
        copyKeys(keys);
    }

    /**
     * @brief Construct with a context-aware reader.
     * @param keys Array of N keys.
     * @param stable_ms Debounce window in milliseconds (0 disables debouncing).
     * @param active_low_mask Runtime polarity (bit=1 -> active-low). Ignored if PolarityMask >= 0.
     * @param read Reader taking (ctx, key) and returning electrical level (true=HIGH, false=LOW).
     * @param ctx Opaque reader context pointer.
     * @param timeFn Optional millisecond time source for no-arg update()/commit().
     */
    SwitchBank(const std::uint8_t (&keys)[N],
               std::uint16_t stable_ms,
               std::uint32_t active_low_mask,
               ReadFn read,
               void *ctx,
               TimeFn timeFn = nullptr) noexcept
        : SwitchBankHandler(timeFn),
          read_pin_fn_{nullptr},
          read_fn_{read},
          ctx_{ctx},
          debounce_ms_{stable_ms},
          active_low_mask_{resolveInitialMask(active_low_mask)}
    {
        copyKeys(keys);
    }

    /**
     * @brief Synchronize internal state to the current hardware levels **without emitting edges**.
     */
    void sync() { sync(now_ms()); }

    /**
     * @brief Synchronize internal state to the current hardware levels using an explicit timestamp.
     * @param now Timestamp (ms).
     */
    void sync(std::uint32_t now)
    {
        const std::uint32_t raw = readPackedOnce();

        previous_ = current_ = raw;

        for (std::uint8_t i = 0; i < static_cast<std::uint8_t>(N); ++i)
        {
            const bool r = (raw >> i) & 1u;
            bits_[i].last_raw = r;
            bits_[i].stable = r;
            bits_[i].last_change_ms = now;
        }

        last_commit_ms_ = now;
        changed_ = false;
        change_count_ = 0;
        last_poll_ms_ = now; ///< Avoid an immediate throttled scan if min_poll_ms_ is set.
    }

    // ---- API ---- //

    /**
     * @brief Return a plain (POD) snapshot of the current stable state.
     * @return SwitchBankSnapshot with fields: value, changed, rising, falling, t_ms, seq.
     */
    [[nodiscard]] SwitchBankSnapshot snapshot() const noexcept
    {
        return SwitchBankSnapshot{current_, changedMask(), risingMask(), fallingMask(), last_commit_ms_, change_count_};
    }

    /**
     * @brief Number of switches in this bank (compile-time constant).
     * @return Count of bits (N).
     */
    static constexpr std::uint8_t kSize = static_cast<std::uint8_t>(N);

    /**
     * @brief Replace or inject the millisecond time source function.
     * @param tf Function returning monotonic milliseconds (nullable).
     */
    void setTimeSource(TimeFn tf) { setTimeFn(tf); }

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    /**
     * @brief Set a commit callback invoked after each successful commit.
     * @param cb Callback pointer (nullable).
     */
    void setOnCommit(OnCommitFn cb) noexcept { on_commit_ = cb; }
#endif

    /**
     * @brief Set the minimum interval between hardware reads.
     * @param ms Minimum poll interval in milliseconds (0 disables throttling).
     */
    void setMinPollMs(std::uint16_t ms) noexcept { min_poll_ms_ = ms; }

    /**
     * @brief Configure how the 'changed' flag is handled.
     * @param m Latch policy (e.g., ManualClear or ClearOnRead).
     */
    void setLatchMode(LatchMode m) noexcept { latch_mode_ = m; }

    /**
     * @brief Set the debounce window duration.
     * @param ms Debounce time in milliseconds (per-bit stabilization window).
     */
    void setDebounceMs(std::uint16_t ms) noexcept { debounce_ms_ = ms; }

    /**
     * @brief Poll hardware and update debounced state.
     * @param now_ms Milliseconds timestamp.
     * @return true If a new stable packed value was committed; otherwise false.
     */
    bool update(std::uint32_t now_ms) override
    {
        if (min_poll_ms_ != 0u)
        {
            const std::uint32_t delta = now_ms - last_poll_ms_;
            if (delta < min_poll_ms_)
            {
                return false;
            }
        }
        last_poll_ms_ = now_ms;

        const std::uint32_t raw = readPackedOnce(); ///< logical packed (post-polarity).
        std::uint32_t next = current_;

        // Per-bit debounce integration.
        for (std::uint8_t i = 0; i < static_cast<std::uint8_t>(N); ++i)
        {
            const bool r = (raw >> i) & 1u; ///< Logical ON (post-polarity).
            BitState &b = bits_[i];

            if (r != b.last_raw)
            {
                b.last_raw = r;
                b.last_change_ms = now_ms;
            }
            if ((now_ms - b.last_change_ms) >= debounce_ms_)
            {
                if (b.stable != r)
                {
                    b.stable = r;
                }
            }

            if (b.stable)
            {
                next |= (1u << i);
            }
            else
            {
                next &= ~(1u << i);
            }
        }

        return commitIfChanged(next, now_ms);
    }

    /**
     * @brief Re-read and commit immediately (bypasses debouncing).
     *        Also resets per-bit debounce history to the current raw state.
     */
    void commit()
    {
        commit(now_ms());
    }

    /**
     * @brief Re-read and commit immediately using an explicit timestamp.
     * @param now Timestamp (ms).
     */
    void commit(std::uint32_t now)
    {
        const std::uint32_t raw = readPackedOnce();

        for (std::uint8_t i = 0; i < static_cast<std::uint8_t>(N); ++i)
        {
            const bool r = (raw >> i) & 1u;
            BitState &b = bits_[i];
            b.last_raw = r;
            b.stable = r;
            b.last_change_ms = now;
        }
        (void)commitIfChanged(raw, now);
    }

    /**
     * @brief Current stable value; may clear 'changed' if LatchMode::ClearOnRead.
     * @return Packed bitmask of current value.
     */
    [[nodiscard]] std::uint32_t value() const noexcept override
    {
        if (latch_mode_ == LatchMode::ClearOnRead)
        {
            changed_ = false;
        }
        return current_;
    }

    /**
     * @brief Stable value without side effects.
     * @return Packed bitmask of current value.
     */
    [[nodiscard]] std::uint32_t peekValue() const noexcept override { return current_; }

    /**
     * @brief Previous committed value.
     * @return Packed bitmask of previous value.
     */
    [[nodiscard]] std::uint32_t prevValue() const noexcept override { return previous_; }

    /**
     * @brief True if last commit changed the value (per latch policy).
     * @return Whether the value changed on the last commit.
     */
    [[nodiscard]] bool changed() const noexcept override { return changed_; }

    /**
     * @brief Manually clear the 'changed' latch.
     */
    void clearChanged() noexcept override { changed_ = false; }

    /**
     * @brief Number of switches (bits).
     * @return Count of bits (N).
     */
    [[nodiscard]] std::uint8_t size() const noexcept override { return static_cast<std::uint8_t>(N); }

    /**
     * @brief Bitwise delta between the current and previous committed values.
     * @return XOR mask of (current ^ previous). This does not depend on the 'changed' latch policy.
     */
    [[nodiscard]] std::uint32_t changedMask() const noexcept override { return current_ ^ previous_; }

    /**
     * @brief Bits that rose 0→1 on the last commit.
     * @return Packed rising-edge mask.
     */
    [[nodiscard]] std::uint32_t risingMask() const noexcept override { return (~previous_) & current_; }

    /**
     * @brief Bits that fell 1→0 on the last commit.
     * @return Packed falling-edge mask.
     */
    [[nodiscard]] std::uint32_t fallingMask() const noexcept override { return previous_ & (~current_); }

    /**
     * @brief Check if bit i is ON.
     * @param i Bit index [0..N-1].
     * @return True if ON; false if OFF or out of range.
     */
    bool isOn(std::uint8_t i) const noexcept
    {
        if (i >= N)
        {
            return false;
        }
        return (current_ >> i) & 1u;
    }

    /**
     * @brief Check if bit i had a rising edge (0→1) on the last commit.
     * @param i Bit index [0..N-1].
     * @return True if rising; false otherwise or out of range.
     */
    bool rose(std::uint8_t i) const noexcept
    {
        if (i >= N)
        {
            return false;
        }
        return ((~previous_ & current_) >> i) & 1u;
    }

    /**
     * @brief Check if bit i had a falling edge (1→0) on the last commit.
     * @param i Bit index [0..N-1].
     * @return True if falling; false otherwise or out of range.
     */
    bool fell(std::uint8_t i) const noexcept
    {
        if (i >= N)
        {
            return false;
        }
        return ((previous_ & ~current_) >> i) & 1u;
    }

    /**
     * @brief Set runtime active-low mask (no-op if fixed at compile time).
     * @param mask Bit=1 means active-low; Bit=0 active-high.
     */
    void setActiveLowMask(std::uint32_t mask) noexcept
    {
        if (hasCompileTimePolarity()) ///< Runtime mask disabled when polarity is fixed at compile time.
        {
            return;
        }
        active_low_mask_ = mask;
        sync(); ///< Polarity changes reinterpret electrical levels; resync prevents spurious edges.
    }

    /**
     * @brief Effective active-low mask (compile-time or runtime).
     * @return Active-low mask.
     */
    std::uint32_t activeLowMask() const noexcept { return effectiveMask(); }

    /**
     * @brief Timestamp of the last commit.
     * @return Milliseconds since time source epoch.
     */
    std::uint32_t lastCommitMs() const noexcept { return last_commit_ms_; }

    /**
     * @brief Number of commits since the last sync().
     * @return Commit counter.
     */
    std::uint32_t changeCount() const noexcept { return change_count_; }

    /**
     * @brief value() as uint8_t for N ≤ 8.
     * @return Current value truncated to 8 bits.
     */
    template <typename Dummy = void>
    typename std::enable_if<(N <= 8), std::uint8_t>::type value8() const noexcept
    {
        return static_cast<std::uint8_t>(value());
    }

    /**
     * @brief changedMask() as uint8_t for N ≤ 8.
     * @return Changed mask truncated to 8 bits.
     */
    template <typename Dummy = void>
    typename std::enable_if<(N <= 8), std::uint8_t>::type changedMask8() const noexcept
    {
        return static_cast<std::uint8_t>(changedMask());
    }

private:
    /**
     * @brief Tracks the per-bit debounce state for a single switch.
     */
    struct BitState
    {
        std::uint32_t last_change_ms{0}; ///< Millis when the input last toggled.
        bool last_raw{false};            ///< Last sampled logical level before stabilization.
        bool stable{false};              ///< Current debounced logical level.
    };

    // ---- Compile-time / runtime mask plumbing ---- //

    /**
     * @brief Whether a compile-time polarity mask is provided.
     * @return true if PolarityMask is enabled; false if runtime mask is used.
     */
    static constexpr bool hasCompileTimePolarity() noexcept { return (PolarityMask >= 0); }

    /**
     * @brief Compile-time active-low mask, truncated to N bits.
     * @return Active-low bitmask (0 if no compile-time polarity).
     */
    static constexpr std::uint32_t compileTimeMask() noexcept
    {
        // PolarityMask is int64_t so -1 can be a sentinel (runtime polarity) while still allowing full 32-bit masks.
        return hasCompileTimePolarity()
                   ? ((N == 32) ? static_cast<std::uint32_t>(PolarityMask)
                                : (static_cast<std::uint32_t>(PolarityMask) & ((1u << N) - 1u)))
                   : 0u;
    }

    /**
     * @brief Default runtime active-low mask (all bits active-low).
     * @return 0xFFFFFFFF for N==32, else (1u<<N)-1.
     */
    static constexpr std::uint32_t defaultActiveLowMaskRuntime() noexcept
    {
        return (N == 32) ? 0xFFFFFFFFu : ((1u << N) - 1u);
    }

    /**
     * @brief Initial mask chosen for construction (CT if available, else runtime).
     * @param runtime_mask Caller-provided runtime mask.
     * @return Mask to use in the constructor.
     */
    static constexpr std::uint32_t initialMaskForCtor(std::uint32_t runtime_mask) noexcept
    {
        return hasCompileTimePolarity() ? compileTimeMask() : runtime_mask;
    }

    /**
     * @brief Resolve initial mask at runtime (helper for constructors).
     * @param runtime_mask Caller-provided runtime mask.
     * @return Mask to use for initial state.
     */
    std::uint32_t resolveInitialMask(std::uint32_t runtime_mask) const noexcept
    {
        return initialMaskForCtor(runtime_mask);
    }

    /**
     * @brief Effective active-low mask used for reads (CT preferred).
     * @return Active-low mask (compile-time if present, otherwise runtime).
     */
    std::uint32_t effectiveMask() const noexcept
    {
        return hasCompileTimePolarity() ? compileTimeMask() : active_low_mask_;
    }

    // ---- Hardware access ---- //

    /**
     * @brief Copy external key identifiers into the internal LSB-first array.
     * @param keys Array of N key IDs (pins/expander addresses), LSB-first.
     */
    void copyKeys(const std::uint8_t (&keys)[N]) noexcept
    {
        for (std::size_t i = 0; i < N; ++i)
        {
            keys_[i] = keys[i];
        }
    }

    /**
     * @brief Read the raw electrical level for a single key (no polarity normalization).
     * @param idx Key index in [0..N-1].
     * @return true if the configured reader reports HIGH for this key; false if reader is null.
     */
    bool readOnePhysical(std::uint8_t idx) const noexcept
    {
        if (read_pin_fn_)
        {
            return read_pin_fn_(keys_[idx]);
        }
        return read_fn_ ? read_fn_(ctx_, keys_[idx]) : false;
    }

    /**
     * @brief Read all inputs once, normalize polarity, and return a packed value.
     */
    std::uint32_t readPackedOnce() const noexcept
    {
        const std::uint32_t mask = effectiveMask();
        std::uint32_t v = 0;

        const std::uint8_t n8 = static_cast<std::uint8_t>(N);
        for (std::uint8_t i = 0; i < n8; ++i)
        {
            // Packed bit index (logical output index).
            const std::uint8_t bi = ReverseOrder ? static_cast<std::uint8_t>(n8 - 1u - i) : i;

            // Electrical level (true=HIGH, false=LOW).
            const bool level_high = readOnePhysical(i);

            // Polarity for this packed bit.
            const bool is_active_low = (mask >> bi) & 1u;

            // Normalize to logical ON.
            const bool on = is_active_low ? (!level_high) : (level_high);

            v |= static_cast<std::uint32_t>(on) << bi;
        }
        return v;
    }

    /**
     * @brief Commit a new packed value if it differs from the current stable value.
     * @param next Next debounced packed value.
     * @param now_ms Current timestamp in milliseconds.
     * @return true if a commit occurred (state changed); false otherwise.
     */
    bool commitIfChanged(std::uint32_t next, std::uint32_t now_ms) noexcept
    {
        if (next != current_)
        {
            previous_ = current_;
            current_ = next;
            changed_ = true;
            last_commit_ms_ = now_ms;
            ++change_count_;
#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
            if (on_commit_)
                on_commit_(snapshot());
#endif
            return true;
        }
        return false;
    }

    // ---- Config / hooks ---- //
    std::uint8_t keys_[N]{};         ///< Key identifiers (pins or expander addresses).
    ReadPinFn read_pin_fn_{nullptr}; ///< Per-key electrical reader (fast path).
    ReadFn read_fn_{nullptr};        ///< Context-aware electrical reader.
    void *ctx_{nullptr};             ///< Opaque reader context.

    // ---- Timing / policy ---- //
    std::uint16_t debounce_ms_{0};                 ///< Debounce window in milliseconds.
    std::uint16_t min_poll_ms_{0};                 ///< Minimum interval between reads (0 disables).
    LatchMode latch_mode_{LatchMode::ManualClear}; ///< Changed flag policy.

    // ---- Polarity ---- //
    std::uint32_t active_low_mask_{defaultActiveLowMaskRuntime()}; ///< Runtime active-low mask (ignored if compile-time mask is set).

    // ---- Per-bit debounce state ---- //
    BitState bits_[N]; ///< Bitwise debounce state (O(N) RAM).

    // ---- Commit state ---- //
    std::uint32_t current_{0};        ///< Current committed packed value.
    std::uint32_t previous_{0};       ///< Previous committed packed value.
    mutable bool changed_{false};     ///< True if value changed on last commit.
    std::uint32_t last_commit_ms_{0}; ///< Timestamp of last commit (ms).
    std::uint32_t change_count_{0};   ///< Number of commits since construction.
#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    /// Optional callback invoked after each successful commit.
    OnCommitFn on_commit_{nullptr};
#endif

    // ---- Scan control ---- //
    std::uint32_t last_poll_ms_{0}; ///< Timestamp of last hardware scan (ms).
};
