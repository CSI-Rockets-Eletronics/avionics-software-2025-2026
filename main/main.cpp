#include <Arduino.h>

#include "nodeconfig.h"

const unsigned long kSerialBaud = 115200;

extern "C" void app_main() {
    initArduino();

    Serial.begin(kSerialBaud);

    nodeconfig::this_node.Setup();

    while (true) {
        nodeconfig::this_node.Loop();
    }
}
