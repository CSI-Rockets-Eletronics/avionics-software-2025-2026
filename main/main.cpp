#include <Arduino.h>

#include "avionics.h"

using namespace avionics;

const unsigned long kSerialBaud = 115200;

Node node{DeviceType::DevDhtImu, DeviceType::DevGps, DeviceType::DevRadio};

extern "C" void app_main() {
    initArduino();

    Serial.begin(kSerialBaud);

    node.Setup();

    while (true) {
        node.Loop();
    }
}
