/**
 * MIT License
 *
 * @brief Abstract base class for an N-bit switch bank (DIP/slide/rocker).
 *
 * @file SwitchBank_Handler.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-10-01
 * @copyright Copyright © 2025 Little Man Builds
 */

#pragma once

#include <stdint.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

/**
 * @class SwitchBank_Handler
 * @brief Interface for polling and reading a debounced, packed switch value.
 *
 * Implementations provide debouncing and change detection. The packed value
 * uses LSB-first ordering (bit 0 == switch index 0). Time is provided by an
 * injected millisecond source (see TimeFn).
 */
class SwitchBankHandler
{
public:
    // Millisecond time source function pointer (injected).
    using TimeFn = uint32_t (*)();

    /**
     * @brief Virtual destructor for safe polymorphic deletion.
     */
    virtual ~SwitchBankHandler() = default;

    /**
     * @brief Poll hardware and update internal state.
     * @param now_ms Milliseconds timestamp from the injected time source.
     * @return true If a new stable packed value was committed.
     * @return false If nothing changed/committed.
     */
    virtual bool update(uint32_t now_ms) = 0;

    /**
     * @brief Convenience overload that uses the configured time source.
     * @return true If a new stable packed value was committed.
     * @return false If nothing changed, or no time source is available.
     */
    bool update()
    {
        if (time_fn_)
            return update(time_fn_());

#if defined(ARDUINO)
        return update(::millis());
#else
        return false;
#endif
    }

    /**
     * @brief Get the current committed packed value (lower N bits).
     * @return Packed value. Bit-to-input mapping is implementation-defined
     *         (e.g., may support reversed packing).
     */
    virtual uint32_t value() const noexcept = 0;

    /**
     * @brief Get the current committed packed value without side effects.
     * @return Packed current value snapshot.
     */
    virtual uint32_t peekValue() const noexcept { return value(); }

    /**
     * @brief Get the previous committed packed value.
     * @return Packed value prior to the last commit.
     */
    virtual uint32_t prevValue() const noexcept = 0;

    /**
     * @brief Whether the packed value changed since the last commit.
     * @return true If the value changed on the previous commit.
     * @return false Otherwise.
     */
    virtual bool changed() const noexcept = 0;

    /**
     * @brief Clear the internal 'changed' flag.
     */
    virtual void clearChanged() noexcept = 0;

    /**
     * @brief Number of switches in the concrete bank (N).
     * @return Switch count (1..32).
     */
    virtual uint8_t size() const noexcept = 0;

    /**
     * @brief Bit mask of changes between current and previous values.
     * @return Change mask.
     */
    virtual uint32_t changedMask() const noexcept { return peekValue() ^ prevValue(); }

    /**
     * @brief Bits that rose 0→1 on the last commit.
     * @return Packed rising-edge mask.
     */
    virtual uint32_t risingMask() const noexcept
    {
        const uint32_t cur = peekValue();
        const uint32_t prev = prevValue();
        return (~prev) & cur;
    }

    /**
     * @brief Bits that fell 1→0 on the last commit.
     * @return Packed falling-edge mask.
     */
    virtual uint32_t fallingMask() const noexcept
    {
        const uint32_t cur = peekValue();
        const uint32_t prev = prevValue();
        return prev & (~cur);
    }

protected:
    /**
     * @brief Construct with an optional time function.
     * @param tf Time source function pointer (nullable).
     */
    explicit SwitchBankHandler(TimeFn tf = nullptr)
        : time_fn_{tf} {}

    /**
     * @brief Update the injected time function.
     * @param tf Time source function pointer (nullable).
     */
    void setTimeFn(TimeFn tf) { time_fn_ = tf; }

    /**
     * @brief Read current time in milliseconds.
     * @return Milliseconds timestamp, or 0 if unavailable.
     */
    uint32_t now_ms() const
    {
        if (time_fn_)
            return time_fn_();

#if defined(ARDUINO)
        return ::millis();
#else
        return 0u;
#endif
    }

private:
    TimeFn time_fn_{nullptr}; ///< Injected millisecond time source.
};