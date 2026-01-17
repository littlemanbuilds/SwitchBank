/**
 * @file 04_Port_Expander_Cached_Read.ino
 *
 * @brief Detect on/off changes from an MCP23017 I2C expander, using a cached
 *        (single read per loop) input scan.
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

// Cached input state for MCP23017 Port A (GPA0..GPA7).
// Each bit represents the raw electrical level of a pin.
static uint8_t g_portA_cache = 0;

// Refresh the cached input state.
// This performs ONE I2C read and should be called once per loop().
static void refreshExpanderCache()
{
    // readGPIOAB() returns 16 bits:
    // - Low byte  = Port A (GPA0..GPA7)
    // - High byte = Port B (GPB0..GPB7)
    const uint16_t gpioAB = mcp.readGPIOAB();
    g_portA_cache = static_cast<uint8_t>(gpioAB);
}

// Pin reader for SwitchBank.
// Returns the RAW electrical level of the pin:
// - True  = HIGH
// - False = LOW
static bool readExpander(uint8_t pin)
{
    return ((g_portA_cache >> pin) & 1) != 0;
}

// Create the SwitchBank as a normal global object.
static auto bank = makeSwitchBankPins<8>(
    EXP_PINS,     // Pin mapping (index → expander pin)
    20,           // Debounce time (ms)
    readExpander, // Cached pin reader
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

    // Establish a clean baseline after hardware initialization.
    // Refresh the cache first so sync() sees valid input data.
    refreshExpanderCache();
    bank.sync(now_ms());
}

void loop()
{
    // Update the cached expander inputs once per loop().
    refreshExpanderCache();

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
