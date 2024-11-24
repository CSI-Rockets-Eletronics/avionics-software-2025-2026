#include <Arduino.h>

#include "device.h"

const unsigned long kSerialBaud = 115200;

extern "C" void app_main() {
    initArduino();

    Serial.begin(kSerialBaud);

    device::InitAll();

    while (true) {
        device::LoopAll();
    }
}
