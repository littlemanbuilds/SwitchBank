/**
 * @file 02_Mode_Names_3bit.ino
 *
 * @brief Interpret a 3-bit DIP switch as named modes.
 */

#include <Arduino.h>
#include <SwitchBank_Arduino.h>

// We have 3 DIP switches on GPIO 35, 36, 37.
// One side of each switch goes to GND, the other side to the pin.
// With INPUT_PULLUP, the pin reads HIGH when 'off' and LOW when 'on'.
const uint8_t DIP_PINS[3] = {35, 36, 37};

// Create a 3-bit switch bank with:
// - 20 ms debounce window.
// - Active-low logic (low = on).
// - INPUT_PULLUP pin configuration.
auto dip = makeSwitchBankArduino<3>(
    DIP_PINS,
    20,
    Polarity::ActiveLow,
    PinModeCfg::Pullup);

// Convert a packed DIP value into a human-readable mode name.
const char *modeName(int mode)
{
    switch (mode)
    {
    case 0:
        return "Mode 0 (Default)";
    case 1:
        return "Mode 1";
    case 2:
        return "Mode 2";
    case 3:
        return "Mode 3";
    case 4:
        return "Mode 4";
    case 5:
        return "Mode 5";
    case 6:
        return "Mode 6";
    case 7:
        return "Mode 7 (Max)";
    default:
        return "Invalid";
    }
}

void setup()
{
    Serial.begin(115200);

    // Synchronize the initial switch state.
    dip.sync();
}

void loop()
{
    // update() polls the inputs and commits a new value when it stabilizes.
    if (dip.update())
    {
        int mode = dip.value(); ///< 0..7 for a 3-bit bank.

        Serial.print("DIP value: ");
        Serial.print(mode, BIN); ///< Print in binary (base 2).
        Serial.print(" -> ");
        Serial.println(modeName(mode));
    }

    // Small delay to keep serial output readable.
    delay(100);
}
