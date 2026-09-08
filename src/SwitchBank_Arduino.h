/**
 * MIT License
 *
 * @brief Arduino convenience helpers for SwitchBank (beginner-friendly).
 *
 * @file SwitchBank_Arduino.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-01-12
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "SwitchBank_Factory.h"

/// @brief Logical switch polarity.
enum class Polarity : uint8_t
{
    ActiveLow = 0, ///< Logical ON when pin reads LOW.
    ActiveHigh     ///< Logical ON when pin reads HIGH.
};

/// @brief Arduino pinMode configuration.
enum class PinModeCfg : uint8_t
{
    Pullup = 0, ///< pinMode(INPUT_PULLUP).
    Pulldown,   ///< pinMode(INPUT_PULLDOWN).
    Input       ///< pinMode(INPUT) (external resistor).
};

/**
 * @brief Owns per-instance Arduino read configuration + an underlying SwitchBank.
 *
 * @tparam N Number of inputs (1..32).
 * @tparam PolarityMask Compile-time polarity mask passed through to SwitchBank (default: runtime).
 * @tparam ReverseOrder Bit packing order passed through to SwitchBank.
 */
template <size_t N, int64_t PolarityMask = -1, bool ReverseOrder = false>
class SwitchBankArduino
{
public:
    using Bank = SwitchBank<N, PolarityMask, ReverseOrder>;

    /**
     * @brief Construct a SwitchBankArduino using Arduino GPIO pins.
     *
     * @param pins GPIO pins (keys).
     * @param debounce_ms Debounce window (ms).
     * @param pol Logical switch polarity (ActiveLow or ActiveHigh).
     *            Ignored when a compile-time PolarityMask is used.
     * @param pin Arduino pinMode configuration.
     * @param skipPinInit If true, does not call pinMode().
     */
    SwitchBankArduino(const uint8_t (&pins)[N],
                      uint16_t debounce_ms,
                      Polarity pol = Polarity::ActiveLow,
                      PinModeCfg pin = PinModeCfg::Pullup,
                      bool skipPinInit = false) noexcept
        : bank_{Bank(
              pins,
              debounce_ms,
              activeLowMaskFromPolarity(pol),
              &readArduinoElectricalLevelCtx,
              nullptr,
              &now_ms)}
    {
        if (!skipPinInit)
        {
            for (uint8_t p : pins)
            {
                applyPinMode(p, pin);
            }
        }
    }

    // ---- Update ---- //

    /// @brief Poll hardware using millis() (injected).
    ///
    /// @return True only when the stable packed value changes; a failed or throttled read returns false.
    bool update() noexcept { return bank_.update(); }

    /// @brief Poll hardware with explicit timestamp.
    ///
    /// @param now Acquisition timestamp in milliseconds.
    /// @return True only when the stable packed value changes; a failed or throttled read returns false.
    bool update(uint32_t now) noexcept { return bank_.update(now); }

    /// @brief Synchronize baseline to current hardware without emitting edges.
    /// @return true when the GPIO acquisition succeeded.
    bool sync() noexcept { return bank_.sync(); }

    /// @brief Synchronize baseline with an explicit timestamp.
    /// @return true when the GPIO acquisition succeeded.
    ///
    /// @param now Acquisition timestamp in milliseconds.
    bool sync(uint32_t now) noexcept { return bank_.sync(now); }

    /// @brief Immediately accept the current GPIO state, bypassing debounce.
    /// @return true when the GPIO acquisition succeeded.
    bool forceCommitForCommissioning() noexcept { return bank_.forceCommitForCommissioning(); }

    /// @brief Timestamped commissioning commit.
    /// @return true when the GPIO acquisition succeeded.
    ///
    /// @param now Acquisition timestamp in milliseconds.
    bool forceCommitForCommissioning(uint32_t now) noexcept { return bank_.forceCommitForCommissioning(now); }

#ifndef SWITCHBANK_NO_LEGACY_COMMIT
    /// @brief Legacy compatibility alias. Prefer forceCommitForCommissioning().
    SWITCHBANK_DEPRECATED("Use forceCommitForCommissioning().")
    void commit() noexcept { (void)bank_.forceCommitForCommissioning(); }

    /// @brief Legacy timestamped compatibility alias.
    ///
    /// @param now Acquisition timestamp in milliseconds.
    SWITCHBANK_DEPRECATED("Use forceCommitForCommissioning(now).")
    void commit(uint32_t now) noexcept { (void)bank_.forceCommitForCommissioning(now); }
#endif

    // ---- Readout ---- //

    /// @brief Get the current committed packed switch value.
    ///
    /// @return Stable logical packed value; ClearOnRead also clears the changed latch.
    SWITCHBANK_NODISCARD uint32_t value() const noexcept { return bank_.value(); }

    /// @brief Get the current packed value without committing.
    ///
    /// @return Stable logical packed value without consuming edges or the changed latch.
    SWITCHBANK_NODISCARD uint32_t peekValue() const noexcept { return bank_.peekValue(); }

    /// @brief Get the previously committed packed switch value.
    ///
    /// @return Baseline for retained edges, reset to the current value by sync() or clearEdges().
    SWITCHBANK_NODISCARD uint32_t prevValue() const noexcept { return bank_.prevValue(); }

    /// @brief Check whether any switch changed on the last commit.
    ///
    /// @return True while a stable transition remains latched; clearing edges alone does not clear it.
    SWITCHBANK_NODISCARD bool changed() const noexcept { return bank_.changed(); }

    /// @brief Get the bitmask of switches that changed on the last commit.
    ///
    /// @return Bits differing between the current value and retained previous value.
    SWITCHBANK_NODISCARD uint32_t changedMask() const noexcept { return bank_.changedMask(); }

    /// @brief Get the bitmask of OFF→ON transitions.
    ///
    /// @return Bits that changed from logical OFF to ON in the retained transition.
    SWITCHBANK_NODISCARD uint32_t risingMask() const noexcept { return bank_.risingMask(); }

    /// @brief Get the bitmask of ON→OFF transitions.
    ///
    /// @return Bits that changed from logical ON to OFF in the retained transition.
    SWITCHBANK_NODISCARD uint32_t fallingMask() const noexcept { return bank_.fallingMask(); }

    /// @brief Check whether a switch transitioned from OFF to ON.
    ///
    /// @param i Logical packed-bit index (0..N-1).
    /// @return True if logical bit i rose in the retained transition; false out of range.
    SWITCHBANK_NODISCARD bool rose(uint8_t i) const noexcept { return bank_.rose(i); }

    /// @brief Check whether a switch transitioned from ON to OFF.
    ///
    /// @param i Logical packed-bit index (0..N-1).
    /// @return True if logical bit i fell in the retained transition; false out of range.
    SWITCHBANK_NODISCARD bool fell(uint8_t i) const noexcept { return bank_.fell(i); }

    /// @brief Check whether a switch is currently ON.
    ///
    /// @param i Logical packed-bit index (0..N-1).
    /// @return True if logical bit i is ON; false for an out-of-range index.
    SWITCHBANK_NODISCARD bool isOn(uint8_t i) const noexcept { return bank_.isOn(i); }

    /// @brief Get the timestamp (ms) of the latest stable state transition.
    ///
    /// @return Latest stable transition time in milliseconds, not the latest sample time.
    SWITCHBANK_NODISCARD uint32_t lastCommitMs() const noexcept { return bank_.lastCommitMs(); }

    /// @brief Get the total stable state transition count.
    ///
    /// @return Stable transition count modulo 2^32, retained across sync().
    SWITCHBANK_NODISCARD uint32_t changeCount() const noexcept { return bank_.changeCount(); }

    /// @brief Whether the Arduino GPIO reader is configured.
    ///
    /// @return True when a reader is configured, without probing hardware connectivity.
    SWITCHBANK_NODISCARD bool configured() const noexcept { return bank_.configured(); }

    /// @brief Whether the most recent GPIO acquisition succeeded.
    ///
    /// @return True if the most recent attempted acquisition succeeded; false before the first sample.
    SWITCHBANK_NODISCARD bool valid() const noexcept { return bank_.valid(); }

    /// @brief Whether at least one successful GPIO acquisition has occurred.
    ///
    /// @return True after any successful acquisition, even if a later acquisition fails.
    SWITCHBANK_NODISCARD bool hasSample() const noexcept { return bank_.hasSample(); }

    /// @brief Timestamp of the latest successful GPIO acquisition.
    ///
    /// @return Latest successful acquisition time in milliseconds, or zero before a sample.
    SWITCHBANK_NODISCARD uint32_t sampleMs() const noexcept { return bank_.sampleMs(); }

    /// @brief Stable state transition sequence.
    ///
    /// @return Stable transition count modulo 2^32; sync() does not increment or reset it.
    SWITCHBANK_NODISCARD uint32_t changeSequence() const noexcept { return bank_.changeSequence(); }

    /// @brief Successful acquisition sequence.
    ///
    /// @return Successful acquisition count modulo 2^32; sync() increments it without resetting it.
    SWITCHBANK_NODISCARD uint32_t sequence() const noexcept { return bank_.sequence(); }

    /// @brief Successful synchronization generation.
    ///
    /// @return Successful explicit resynchronization count modulo 2^32.
    SWITCHBANK_NODISCARD uint32_t generation() const noexcept { return bank_.generation(); }

    /// @brief Get complete health/freshness metadata.
    ///
    /// @return A copy of health and freshness metadata; no acquisition or consumption occurs.
    SWITCHBANK_NODISCARD SwitchBankStatus status() const noexcept { return bank_.status(); }

    /// @brief Get the number of switches managed by this bank.
    ///
    /// @return Number of inputs, in the range 1..32.
    SWITCHBANK_NODISCARD uint8_t size() const noexcept { return bank_.size(); }

    // ---- Control ---- //

    /// @brief Explicitly consume and clear the changed latch.
    ///
    /// @return The previous latch state; retained edge masks are unchanged.
    bool consumeChanged() noexcept { return bank_.consumeChanged(); }

    /// @brief Explicitly consume retained edge masks and the changed latch.
    ///
    /// @return Retained edges and transition sequence; clears edges and the changed latch.
    SwitchBankEdges consumeEdges() noexcept { return bank_.consumeEdges(); }

    /// @brief Clear only the changed latch; edge masks still describe previous/current.
    void clearChanged() noexcept { bank_.clearChanged(); }

    /// @brief Clear previous/current edge masks without changing the current value.
    void clearEdges() noexcept { bank_.clearEdges(); }

    /// @brief Set the debounce window (ms).
    ///
    /// @param ms Required observed stability in milliseconds; zero disables debounce.
    void setDebounceMs(uint16_t ms) noexcept { bank_.setDebounceMs(ms); }

    /// @brief Set the minimum polling interval (ms).
    ///
    /// @param ms Minimum milliseconds between attempts; zero disables throttling.
    void setMinPollMs(uint16_t ms) noexcept { bank_.setMinPollMs(ms); }

    /// @brief Set the latch behavior for changed/edge reporting.
    ///
    /// @param mode ManualClear or legacy ClearOnRead policy for changed(), not edge masks.
    void setLatchMode(typename Bank::LatchMode mode) noexcept { bank_.setLatchMode(mode); }

    /// @brief Inject a time source used by update() and internal timestamps.
    ///
    /// @param fn Optional millisecond clock; null restores the default clock behavior.
    void setTimeSource(typename SwitchBankHandler::TimeFn fn) noexcept { bank_.setTimeSource(fn); }

    /// @brief Set runtime polarity and resynchronize without emitting edges.
    /// @return true when the new polarity was established from a valid GPIO sample.
    ///
    /// @param m New active-low bits in logical packed order; ignored for compile-time polarity.
    bool setActiveLowMask(uint32_t m) noexcept { return bank_.setActiveLowMask(m); }

    /// @brief Get the runtime active-low polarity mask.
    ///
    /// @return Effective polarity mask in logical packed-bit order; bits above N are zero.
    SWITCHBANK_NODISCARD uint32_t activeLowMask() const noexcept { return bank_.activeLowMask(); }

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    /// @brief Set the optional state-transition callback.
    ///
    /// @param cb Callback invoked in the caller context after a stable transition; null disables it.
    void setOnCommit(typename Bank::OnCommitFn cb) noexcept { bank_.setOnCommit(cb); }
#endif

    /// @brief Produce a POD snapshot (same as core).
    ///
    /// @return A copy of stable state, retained edges and health metadata; no acquisition occurs.
    SWITCHBANK_NODISCARD typename Bank::SwitchBankSnapshot snapshot() const noexcept { return bank_.snapshot(); }

    /// @brief Access underlying SwitchBank directly (advanced).
    ///
    /// @return Reference to the owned bank, valid only for this wrapper lifetime.
    SWITCHBANK_NODISCARD Bank &core() noexcept { return bank_; }

    /// @brief Access underlying SwitchBank directly (advanced).
    ///
    /// @return Reference to the owned bank, valid only for this wrapper lifetime.
    SWITCHBANK_NODISCARD const Bank &core() const noexcept { return bank_; }

private:
    /// @brief Arduino time source (milliseconds).
    ///
    /// @return Arduino millis() timestamp, wrapping modulo 2^32.
    static uint32_t now_ms() noexcept { return millis(); }

    /**
     * @brief Arduino GPIO reader returning the electrical level.
     *
     * @param key GPIO pin number stored as the SwitchBank key.
     * @return true If the pin reads HIGH.
     * @return false If the pin reads LOW.
     */
    static bool readArduinoElectricalLevelCtx(void * /*ctx*/, uint8_t key) noexcept
    {
        const int v = digitalRead(static_cast<uint8_t>(key));
        return (v == HIGH);
    }

    /**
     * @brief Map a logical polarity selection to an active-low mask for this bank.
     *
     * ActiveLow  → all bits set (active-low).
     * ActiveHigh → all bits clear (active-high).
     *
     * @param p Logical active-low or active-high interpretation.
     * @return All N bits set for ActiveLow, otherwise zero.
     */
    static uint32_t activeLowMaskFromPolarity(Polarity p) noexcept
    {
        return (p == Polarity::ActiveLow) ? mask_all_active_low<N>() : mask_all_active_high<N>();
    }

    /**
     * @brief Apply Arduino pinMode according to the pin mode configuration.
     *
     * @param pin Arduino GPIO pin.
     * @param m Pin mode configuration.
     */
    static void applyPinMode(uint8_t pin, PinModeCfg m) noexcept
    {
        switch (m)
        {
        case PinModeCfg::Pullup:
            pinMode(pin, INPUT_PULLUP);
            break;
        case PinModeCfg::Pulldown:
#if defined(INPUT_PULLDOWN)
            pinMode(pin, INPUT_PULLDOWN);
#else
            // Fallback when INPUT_PULLDOWN is not supported by this core.
            pinMode(pin, INPUT);
#endif
            break;
        case PinModeCfg::Input:
        default:
            pinMode(pin, INPUT);
            break;
        }
    }

    Bank bank_; ///< Underlying SwitchBank instance.
};

/**
 * @brief Beginner-friendly Arduino factory.
 *
 * @tparam N Number of inputs.
 * @param pins GPIO pins.
 * @param debounce_ms Debounce window (ms).
 * @param pol Logical switch polarity (default: ActiveLow).
 * @param pin Arduino pinMode configuration (default: Pullup).
 * @param skipPinInit If true, does not call pinMode().
 * @return SwitchBankArduino<N> with runtime polarity control.
 *         For compile-time polarity/reverse order, instantiate SwitchBankArduino directly.
 */
template <size_t N>
inline SwitchBankArduino<N> makeSwitchBankArduino(const uint8_t (&pins)[N],
                                                  uint16_t debounce_ms,
                                                  Polarity pol = Polarity::ActiveLow,
                                                  PinModeCfg pin = PinModeCfg::Pullup,
                                                  bool skipPinInit = false) noexcept
{
    return SwitchBankArduino<N>(pins, debounce_ms, pol, pin, skipPinInit);
}
