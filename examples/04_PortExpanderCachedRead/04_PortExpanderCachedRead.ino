/**
 * MIT License
 *
 * @brief Read eight MCP23017 switches from one coherent GPIO register snapshot.
 *
 * @file 04_PortExpanderCachedRead.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-01-16
 * @copyright Copyright © 2026 Little Man Builds
 */

#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

// Default MCP23017 I2C address.
const uint8_t MCP_ADDR = 0x20;

// Bank input order. Here input 0..7 maps directly to GPA0..GPA7.
const uint8_t EXP_PINS[8] = {0, 1, 2, 3, 4, 5, 6, 7};

Adafruit_MCP23X17 mcp;

// ---- SwitchBank helpers ---- //

uint32_t now_ms()
{
    return millis();
}

// Read the complete MCP23017 register once.
// Bit i returned here represents the electrical level for bank input i.
static SwitchBankPackedReadResult readExpanderPacked(void * /*ctx*/)
{
    const uint16_t gpioAB = mcp.readGPIOAB();
    const uint32_t portA = static_cast<uint32_t>(gpioAB & 0x00FFu);

    // Adafruit_MCP23X17::readGPIOAB() does not expose a definitive transfer-status
    // result. A lower-level adapter that can detect an I2C failure should return
    // SwitchBankPackedReadResult::failure() instead.
    return SwitchBankPackedReadResult::success(portA);
}

// Active-low is the factory default, matching INPUT_PULLUP wiring.
static auto bank = makeSwitchBankPacked<8>(
    EXP_PINS,
    20,
    readExpanderPacked,
    nullptr,
    now_ms);

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Wire.begin() uses the board defaults. ESP32 users may also pass SDA/SCL.
    Wire.begin();

    if (!mcp.begin_I2C(MCP_ADDR))
    {
        Serial.println("MCP23017 not found!");
        while (true)
        {
            delay(1000);
        }
    }

    for (uint8_t i = 0; i < 8; ++i)
    {
        mcp.pinMode(EXP_PINS[i], INPUT_PULLUP);
    }

    bank.setMinPollMs(5);

    // Establish the initial state without generating boot edges.
    if (!bank.sync(now_ms()))
    {
        Serial.println("Initial switch read failed!");
    }
}

void loop()
{
    // update() performs one packed MCP23017 acquisition, then debounces all bits.
    if (bank.update(now_ms()))
    {
        for (uint8_t i = 0; i < bank.size(); ++i)
        {
            if (bank.rose(i))
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" turned ON.");
            }

            if (bank.fell(i))
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" turned OFF.");
            }
        }
    }

    delay(100);
}
