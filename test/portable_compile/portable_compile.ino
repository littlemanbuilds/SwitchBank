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

#include <Arduino.h>
#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

static const uint8_t KEYS[3] = {0u, 1u, 2u};
static uint32_t fake_levels = 0x7u;

static uint32_t nowMs()
{
    return static_cast<uint32_t>(millis());
}

static SwitchBankPackedReadResult readPacked(void * /*ctx*/)
{
    return SwitchBankPackedReadResult::success(fake_levels);
}

static auto bank = makeSwitchBankPacked<3>(KEYS, 5u, readPacked, nullptr, nowMs);

void setup()
{
    (void)bank.sync();
}

void loop()
{
    (void)bank.update();
    const SwitchBankStatus st = bank.status();
    if (st.valid)
    {
        (void)bank.peekValue();
    }
}
