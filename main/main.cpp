#include <Arduino.h>

#include "device.h"

extern "C" void app_main() {
    initArduino();

    Serial.begin(115200);

    device::InitAll();

    while (true) {
        device::LoopAll();
    }
}
