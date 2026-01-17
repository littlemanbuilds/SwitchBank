/**
 * @file 01_Simple_DIP_3bit.ino
 *
 * @brief Simple 3-bit DIP switch example using Arduino GPIO pins.
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
        // Check each switch for 'off' → 'on' or 'on' → 'off' transitions.
        for (int i = 0; i < dip.size(); ++i)
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
