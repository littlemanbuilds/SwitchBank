/**
 * MIT License
 *
 * @brief Simple 3-bit DIP switch example using ESP32-S3 GPIO pins.
 *
 * @file 01_SimpleDIP3Bit.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-09-30
 * @copyright Copyright © 2026 Little Man Builds
 */

#include <SwitchBank_Arduino.h>

#include <Arduino.h>

// ESP32-S3 example pins. Change these to suitable GPIOs for your board.
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

void setup()
{
    Serial.begin(115200);

    // Important: synchronize the initial switch state after pin setup.
    // This establishes a baseline so boot positions do not look like edges.
    dip.sync();
}

void loop()
{
    // update() polls the inputs and commits a new value when it stabilizes.
    if (dip.update())
    {
        // Check each switch for 'off' → 'on' or 'on' → 'off' transitions.
        for (uint8_t i = 0; i < dip.size(); ++i)
        {
            if (dip.rose(i)) // Off → on.
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" turned ON");
            }

            if (dip.fell(i)) // On → off.
            {
                Serial.print("Switch ");
                Serial.print(i + 1);
                Serial.println(" turned OFF");
            }
        }
    }

    // Small delay to keep serial output readable.
    delay(100);
}
