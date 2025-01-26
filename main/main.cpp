#include <Arduino.h>

#include "comms.h"
#include "deviceindex.h"
#include "nodeconfig.h"
#include "ota.h"

using namespace avionics;

const unsigned long kSerialBaud = 115200;

extern "C" void app_main() {
    initArduino();

    Serial.begin(kSerialBaud);

    EspNowSetup([](auto data, auto len) { Node::OnReceive(data, len); });

    auto maybe_this_node = Node::FindNode(GetThisMacAddress());
    if (!maybe_this_node) {
        Die("Failed to find this node");
    }

    auto& this_node = maybe_this_node->get();

    ota::CheckForUpdate(this_node.name);

    this_node.Run();

    vTaskDelete(nullptr);  // this lets other tasks run forever
}
