/**
 * MIT License
 *
 * @brief Portable compile smoke test for the hardware-neutral SwitchBank API.
 *
 * @file portable_compile.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright © 2026 Little Man Builds
 */

#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

#if defined(__AVR__)
static_assert(sizeof(unsigned int) == 2, "AVR coverage must use real 16-bit unsigned int");
#endif
static_assert(sizeof(uint32_t) == 4, "Packed masks require 32 bits");
static_assert(mask_all_active_low<15>() == 0x7fffUL, "15-input boundary");
static_assert(mask_all_active_low<16>() == 0xffffUL, "16-input boundary");
static_assert(mask_all_active_low<17>() == 0x1ffffUL, "17-input boundary");
static_assert(mask_all_active_low<31>() == 0x7fffffffUL, "31-input boundary");
static_assert(mask_all_active_low<32>() == 0xffffffffUL, "32-input boundary without shift by 32");
static_assert(mask_all_active_high<32>() == 0UL, "All active-high");

#if __cplusplus >= 201402L
constexpr uint8_t first[] = {0};
constexpr uint8_t upper[] = {15, 16, 31};
constexpr bool polarities[32] = {
    false, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true};
static_assert(mask_from_active_high_indices<32>(first) == 0xfffffffeUL,
              "Clearing bit zero must retain the upper sixteen bits");
static_assert(mask_from_active_high_indices<32>(upper) == 0x7ffe7fffUL,
              "Mixed polarity at indices 15, 16 and 31");
static_assert(mask_from_active_high_indices<16>(upper) == 0x7fffUL, "Ignore indices outside bank");
static_assert(mask_from_active_high_indices<17>(upper) == 0x7fffUL, "17-input mixed polarity");
static_assert(mask_from_active_high_indices<31>(upper) == 0x7ffe7fffUL, "31-input mixed polarity");
static_assert(mask_from_active_low_array(polarities) == 0xfffffffeUL, "Boolean polarity array");
#endif

#ifndef SWITCHBANK_MASK_PROBE
#include <Arduino.h>

const uint8_t KEYS[32] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
volatile uint32_t fake_levels = 0x80018000UL;
volatile bool sample_valid = true;
volatile uint32_t observed = 0;

static SwitchBankReadResult readResult(void *, uint8_t key)
{
    return sample_valid ? SwitchBankReadResult::success(((fake_levels >> key) & 1u) != 0u)
                        : SwitchBankReadResult::failure();
}

static bool readPin(uint8_t key)
{
    return ((fake_levels >> key) & 1u) != 0u;
}

static SwitchBankPackedReadResult readPacked(void *)
{
    return sample_valid ? SwitchBankPackedReadResult::success(fake_levels)
                        : SwitchBankPackedReadResult::failure();
}

void setup()
{
    // Sequential blocks keep the fixture within small-target RAM limits.
    {
        auto bank = makeSwitchBankResultCtx<32>(KEYS, 0, readResult);
        (void)bank.sync(0);
        fake_levels = 1;
        (void)bank.update(1);
        sample_valid = false;
        (void)bank.setActiveLowMask(0, 2); // Compile failure/rollback path.
        sample_valid = true;
        const uint8_t high[] = {0, 15, 16, 31};
        (void)bank.setActiveLowMask(mask_from_active_high_indices<32>(high), 3);
        observed = bank.peekValue();
    }
    {
        auto bank = makeSwitchBankPinsCT<32, 0, true>(KEYS, 0, readPin);
        (void)bank.sync(4);
        observed = bank.peekValue();
    }
    {
        auto bank = makeSwitchBankPacked<32>(KEYS, 0, readPacked);
        (void)bank.sync(5);
        (void)bank.setActiveLowMask(mask_from_active_high_indices<32>(uint8_t{0}, uint8_t{15}, uint8_t{16}, uint8_t{31}), 6);
        observed = bank.peekValue();
    }
}

void loop() {}
#endif
