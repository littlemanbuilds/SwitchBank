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
#include <cstdint>
#include <SwitchBank_Factory.h>

/// @brief Logical switch polarity.
enum class Polarity : std::uint8_t
{
    ActiveLow = 0, ///< Logical ON when pin reads LOW.
    ActiveHigh     ///< Logical ON when pin reads HIGH.
};

/// @brief Arduino pinMode configuration.
enum class PinModeCfg : std::uint8_t
{
    Pullup = 0, ///< pinMode(INPUT_PULLUP).
    Pulldown,   ///< pinMode(INPUT_PULLDOWN).
    Input       ///< pinMode(INPUT) (external resistor).
};

/**
 * @brief Owns per-instance Arduino read configuration + an underlying SwitchBank.
 * @tparam N Number of inputs (1..32).
 * @tparam PolarityMask Compile-time polarity mask passed through to SwitchBank (default: runtime).
 * @tparam ReverseOrder Bit packing order passed through to SwitchBank.
 */
template <std::size_t N, std::int64_t PolarityMask = -1, bool ReverseOrder = false>
class SwitchBankArduino
{
public:
    using Bank = SwitchBank<N, PolarityMask, ReverseOrder>;

    /**
     * @brief Construct a SwitchBankArduino using Arduino GPIO pins.
     * @param pins GPIO pins (keys).
     * @param debounce_ms Debounce window (ms).
     * @param pol Logical switch polarity (ActiveLow or ActiveHigh).
     * @param pin Arduino pinMode configuration.
     * @param skipPinInit If true, does not call pinMode().
     */
    SwitchBankArduino(const std::uint8_t (&pins)[N],
                      std::uint16_t debounce_ms,
                      Polarity pol = Polarity::ActiveLow,
                      PinModeCfg pin = PinModeCfg::Pullup,
                      bool skipPinInit = false) noexcept
        : bank_{makeSwitchBankCtxMasked<N>(
              pins,
              debounce_ms,
              &readArduinoElectricalLevelCtx,
              nullptr,
              activeLowMaskFromPolarity(pol),
              &now_ms)}
    {
        if (!skipPinInit)
        {
            for (std::uint8_t p : pins)
            {
                applyPinMode(p, pin);
            }
        }
    }

    // ---- Update ---- //

    /// @brief Poll hardware using millis() (injected).
    bool update() noexcept { return bank_.update(); }

    /// @brief Poll hardware with explicit timestamp.
    bool update(std::uint32_t now) noexcept { return bank_.update(now); }

    /// @brief Synchronize baseline to current hardware **without emitting edges**.
    void sync() noexcept { bank_.sync(); }

    /// @brief Synchronize baseline with an explicit timestamp.
    void sync(std::uint32_t now) noexcept { bank_.sync(now); }

    /// @brief Commit the current state immediately (may emit edges if different).
    ///        Prefer sync() for the initial baseline.
    void commit() noexcept { bank_.commit(); }

    /// @brief Commit the current state immediately using an explicit timestamp.
    void commit(std::uint32_t now) noexcept { bank_.commit(now); }

    // ---- Readout ---- //

    /// @brief Get the current committed packed switch value.
    [[nodiscard]] std::uint32_t value() const noexcept { return bank_.value(); }

    /// @brief Get the current packed value without committing.
    [[nodiscard]] std::uint32_t peekValue() const noexcept { return bank_.peekValue(); }

    /// @brief Get the previously committed packed switch value.
    [[nodiscard]] std::uint32_t prevValue() const noexcept { return bank_.prevValue(); }

    /// @brief Check whether any switch changed on the last commit.
    [[nodiscard]] bool changed() const noexcept { return bank_.changed(); }

    /// @brief Get the bitmask of switches that changed on the last commit.
    [[nodiscard]] std::uint32_t changedMask() const noexcept { return bank_.changedMask(); }

    /// @brief Get the bitmask of OFF→ON transitions.
    [[nodiscard]] std::uint32_t risingMask() const noexcept { return bank_.risingMask(); }

    /// @brief Get the bitmask of ON→OFF transitions.
    [[nodiscard]] std::uint32_t fallingMask() const noexcept { return bank_.fallingMask(); }

    /// @brief Check whether a switch transitioned from OFF to ON.
    [[nodiscard]] bool rose(std::uint8_t i) const noexcept { return bank_.rose(i); }

    /// @brief Check whether a switch transitioned from ON to OFF.
    [[nodiscard]] bool fell(std::uint8_t i) const noexcept { return bank_.fell(i); }

    /// @brief Check whether a switch is currently ON.
    [[nodiscard]] bool isOn(std::uint8_t i) const noexcept { return bank_.isOn(i); }

    /// @brief Get the timestamp (ms) of the last state commit.
    [[nodiscard]] std::uint32_t lastCommitMs() const noexcept { return bank_.lastCommitMs(); }

    /// @brief Get the total number of committed changes.
    [[nodiscard]] std::uint32_t changeCount() const noexcept { return bank_.changeCount(); }

    /// @brief Get the number of switches managed by this bank.
    [[nodiscard]] constexpr std::uint8_t size() const noexcept { return bank_.size(); }

    // ---- Control ---- //

    /// @brief Clear the changed/edge flags.
    void clearChanged() noexcept { bank_.clearChanged(); }

    /// @brief Set the debounce window (ms).
    void setDebounceMs(std::uint16_t ms) noexcept { bank_.setDebounceMs(ms); }

    /// @brief Set the minimum polling interval (ms).
    void setMinPollMs(std::uint16_t ms) noexcept { bank_.setMinPollMs(ms); }

    /// @brief Set the latch behavior for changed/edge reporting.
    void setLatchMode(typename Bank::LatchMode mode) noexcept { bank_.setLatchMode(mode); }

    /// @brief Inject a time source used by update() and internal timestamps.
    void setTimeSource(typename SwitchBankHandler::TimeFn fn) noexcept { bank_.setTimeSource(fn); }

    /// @brief Set the runtime active-low polarity mask.
    void setActiveLowMask(std::uint32_t m) noexcept { bank_.setActiveLowMask(m); }

    /// @brief Get the runtime active-low polarity mask.
    [[nodiscard]] std::uint32_t activeLowMask() const noexcept { return bank_.activeLowMask(); }

#ifdef SWITCHBANK_ENABLE_COMMIT_CALLBACK
    void setOnCommit(typename Bank::OnCommitFn cb) noexcept { bank_.setOnCommit(cb); }
#endif

    /// @brief Produce a POD snapshot (same as core).
    [[nodiscard]] typename Bank::SwitchBankSnapshot snapshot() const noexcept { return bank_.snapshot(); }

    /// @brief Access underlying SwitchBank directly (advanced).
    [[nodiscard]] Bank &core() noexcept { return bank_; }

    /// @brief Access underlying SwitchBank directly (advanced).
    [[nodiscard]] const Bank &core() const noexcept { return bank_; }

private:
    /// @brief Arduino time source (milliseconds).
    static std::uint32_t now_ms() noexcept { return millis(); }

    /**
     * @brief Arduino GPIO reader returning the electrical level.
     * @param ctx Unused (kept for factory signature compatibility).
     * @param key GPIO pin number stored as the SwitchBank key.
     * @return true If the pin reads HIGH.
     * @return false If the pin reads LOW.
     */
    static bool readArduinoElectricalLevelCtx(void * /*ctx*/, std::uint8_t key) noexcept
    {
        const int v = digitalRead(static_cast<std::uint8_t>(key));
        return (v == HIGH);
    }

    /**
     * @brief Map a logical polarity selection to an active-low mask for this bank.
     *
     * ActiveLow  → all bits set (active-low).
     * ActiveHigh → all bits clear (active-high).
     */
    static std::uint32_t activeLowMaskFromPolarity(Polarity p) noexcept
    {
        return (p == Polarity::ActiveLow) ? mask_all_active_low<N>() : mask_all_active_high<N>();
    }

    /**
     * @brief Apply Arduino pinMode according to the pin mode configuration.
     * @param pin Arduino GPIO pin.
     * @param m Pin mode configuration.
     */
    static void applyPinMode(std::uint8_t pin, PinModeCfg m) noexcept
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
            pinMode(pin, INPUT); ///< Fallback when INPUT_PULLDOWN is not supported by this core.
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
 * @tparam N Number of inputs.
 * @param pins GPIO pins.
 * @param debounce_ms Debounce window (ms).
 * @param pol Logical switch polarity (default: ActiveLow).
 * @param pin Arduino pinMode configuration (default: Pullup).
 * @param skipPinInit If true, does not call pinMode().
 */
template <std::size_t N>
inline SwitchBankArduino<N> makeSwitchBankArduino(const std::uint8_t (&pins)[N],
                                                  std::uint16_t debounce_ms,
                                                  Polarity pol = Polarity::ActiveLow,
                                                  PinModeCfg pin = PinModeCfg::Pullup,
                                                  bool skipPinInit = false) noexcept
{
    return SwitchBankArduino<N>(pins, debounce_ms, pol, pin, skipPinInit);
}
