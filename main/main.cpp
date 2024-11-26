#include <Arduino.h>

#include "comms.h"
#include "deviceindex.h"
#include "nodeconfig.h"

using namespace avionics;

const unsigned long kSerialBaud = 115200;

extern "C" void app_main() {
    initArduino();

    Serial.begin(kSerialBaud);

    auto this_mac_address =
        EspNowSetup([](auto data, auto len) { Node::OnReceive(data, len); });

    auto maybe_this_node = Node::FindNode(this_mac_address);
    if (!maybe_this_node) {
        Die("Failed to find this node");
    }

    auto& this_node = maybe_this_node->get();

    this_node.Setup();

    while (true) {
        this_node.Loop();
    }
}
