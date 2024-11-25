#include <Arduino.h>

#include "comms.h"
#include "nodeconfig.h"

const unsigned long kSerialBaud = 115200;

extern "C" void app_main() {
    initArduino();

    Serial.begin(kSerialBaud);

    avionics::EspNowSetup([](auto dato, auto len) {
        // TODO
    });

    while (true) {
        avionics::this_node.Loop();
    }
}
