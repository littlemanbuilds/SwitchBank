/**
 * MIT License
 *
 * @brief Abstract interface for polling and reading a debounced packed switch bank.
 *
 * @file SwitchBank_Handler.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-01
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include "SwitchBank_Compatibility.h"
#include <stdint.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

/**
 * @class SwitchBankHandler
 * @brief Minimal polymorphic interface for a packed switch source.
 *
 * Concrete SwitchBank instances add polarity, acquisition validity, debouncing, edge metadata,
 * and freshness counters. The base remains intentionally small so applications that only need a
 * generic packed switch provider do not inherit hardware-specific policy.
 */
class SwitchBankHandler
{
public:
    /// @brief Millisecond time source function pointer.
    using TimeFn = uint32_t (*)();

    /// @brief Virtual destructor for safe polymorphic deletion.
    virtual ~SwitchBankHandler() = default;

    /**
     * @brief Poll hardware and update internal state.
     *
     * @param now_ms Current timestamp in milliseconds.
     * @return true when the stable packed value changed.
     */
    virtual bool update(uint32_t now_ms) = 0;

    /**
     * @brief Poll using the configured clock or Arduino millis().
     *
     * @return true when the stable packed value changed.
     * @return false when no transition occurred or no clock is available.
     */
    bool update()
    {
        if (time_fn_ != nullptr)
        {
            return update(time_fn_());
        }

#if defined(ARDUINO)
        return update(::millis());
#else
        return false;
#endif
    }

    /// @brief Current stable packed value.
    ///
    /// @return Stable logical packed value; ClearOnRead also clears the changed latch.
    virtual uint32_t value() const noexcept = 0;

    /// @brief Current stable packed value without side effects.
    ///
    /// @return value() by default; overrides must provide side-effect-free reads.
    virtual uint32_t peekValue() const noexcept { return value(); }

    /// @brief Previous stable value associated with retained edge metadata.
    ///
    /// @return Baseline for retained edges, reset to the current value by sync() or clearEdges().
    virtual uint32_t prevValue() const noexcept = 0;

    /// @brief Whether a stable transition is currently latched.
    ///
    /// @return True while a stable transition remains latched; clearing edges alone does not clear it.
    virtual bool changed() const noexcept = 0;

    /// @brief Explicitly clear the changed latch.
    virtual void clearChanged() noexcept = 0;

    /// @brief Clear retained edge masks without changing current state.
    virtual void clearEdges() noexcept {}

    /// @brief Number of switches in the concrete bank.
    ///
    /// @return Number of inputs, in the range 1..32.
    virtual uint8_t size() const noexcept = 0;

    /// @brief Retained transition mask.
    ///
    /// @return Bits differing between the current value and retained previous value.
    virtual uint32_t changedMask() const noexcept { return peekValue() ^ prevValue(); }

    /// @brief Retained OFF->ON mask.
    ///
    /// @return Bits that changed from logical OFF to ON in the retained transition.
    virtual uint32_t risingMask() const noexcept
    {
        const uint32_t cur = peekValue();
        const uint32_t prev = prevValue();
        return (~prev) & cur;
    }

    /// @brief Retained ON->OFF mask.
    ///
    /// @return Bits that changed from logical ON to OFF in the retained transition.
    virtual uint32_t fallingMask() const noexcept
    {
        const uint32_t cur = peekValue();
        const uint32_t prev = prevValue();
        return prev & (~cur);
    }

    // ---- Optional health/freshness contract ---- //

    /**
     * @brief Whether the most recent hardware acquisition succeeded.
     *
     * @note Base implementations that do not expose validity return false.
     *
     * @return False by default; override to expose hardware acquisition validity.
     */
    virtual bool valid() const noexcept { return false; }

    /// @brief Timestamp of the latest successful acquisition, when supported.
    ///
    /// @return Latest successful sample timestamp, or zero when unsupported.
    virtual uint32_t sampleMs() const noexcept { return 0u; }

    /// @brief Successful acquisition sequence, when supported.
    ///
    /// @return Successful acquisition count, or zero when unsupported.
    virtual uint32_t sequence() const noexcept { return 0u; }

    /// @brief Stable transition sequence, when supported.
    ///
    /// @return Stable transition count, or zero when unsupported.
    virtual uint32_t changeSequence() const noexcept { return 0u; }

protected:
    /**
     * @brief Construct with an optional time source.
     *
     * @param tf Time source function pointer (nullable).
     */
    explicit SwitchBankHandler(TimeFn tf = nullptr) noexcept
        : time_fn_{tf} {}

    /// @brief Replace the injected time source.
    ///
    /// @param tf Optional millisecond clock; null restores the default clock behavior.
    void setTimeFn(TimeFn tf) noexcept { time_fn_ = tf; }

    /**
     * @brief Read current time from the configured source.
     *
     * @return Timestamp in milliseconds, or zero when unavailable on a native build.
     */
    uint32_t now_ms() const noexcept
    {
        if (time_fn_ != nullptr)
        {
            return time_fn_();
        }

#if defined(ARDUINO)
        return ::millis();
#else
        return 0u;
#endif
    }

private:
    TimeFn time_fn_{nullptr}; ///< Injected millisecond clock.
};
