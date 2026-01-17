/**
 * @file 03_Port_Expander_8-bit.ino
 *
 * @brief Print on/off changes from 8 switches on an MCP23017 I2C expander.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <SwitchBank.h>
#include <SwitchBank_Factory.h>

// Default I2C address 0x20.
const uint8_t MCP_ADDR = 0x20;

// Bit order: SwitchBank index 0..7 maps to these expander pins (using GPA0..GPA7).
const uint8_t EXP_PINS[8] = {0, 1, 2, 3, 4, 5, 6, 7};

// MCP23017 I2C GPIO expander instance.
Adafruit_MCP23X17 mcp;

// ---- SwitchBank helpers ----//

// Time source (ms).
uint32_t now_ms()
{
    return millis(); ///< Get current time (ms).
}

// Pin reader for SwitchBank.
static bool readExpander(uint8_t pin)
{
    return mcp.digitalRead(pin) == HIGH; ///< Returns raw level (HIGH=true, LOW=false).
}

// Create the SwitchBank as a normal global object.
static auto bank = makeSwitchBankPins<8>(
    EXP_PINS,     // Pin mapping (index → expander pin)
    20,           // Debounce time (ms)
    readExpander, // Pin reader
    now_ms        // Time source
);

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Initialize I2C.
    Wire.begin();

    // Initialize the MCP23017.
    if (!mcp.begin_I2C(MCP_ADDR))
    {
        Serial.println("MCP23017 not found!");
        while (true)
        {
            delay(100);
        }
    }

    // Configure expander pins as inputs with INPUT_PULLUP.
    for (uint8_t i = 0; i < 8; ++i)
    {
        mcp.pinMode(EXP_PINS[i], INPUT_PULLUP);
    }

    // Optional: prevent excessive I2C polling.
    bank.setMinPollMs(5);

    // Call sync() once after all hardware is initialized.
    // This reads the current switch states and establishes a baseline.
    // Prevent any phantom on/off events at boot.
    bank.sync(now_ms());
}

void loop()
{
    // update() polls inputs, applies debounce, and detects changes.
    if (bank.update(now_ms()))
    {
        // Report switch edges.
        for (uint8_t i = 0; i < bank.size(); ++i)
        {
            if (bank.rose(i)) ///< Off → on
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" turned ON.");
            }

            if (bank.fell(i)) ///< On → off
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" turned OFF.");
            }
        }
    }

    delay(100); ///< Small delay to keep serial output readable.
}