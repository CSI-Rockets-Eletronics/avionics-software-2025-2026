#include <Arduino.h>
#include <WiFi.h>

#include "comms.h"
#include "deviceindex.h"
#include "nodeconfig.h"
#include "ota.h"

using namespace avionics;

const unsigned long kSerialBaud = 115200;

extern "C" void app_main() {
    initArduino();

    Serial.begin(kSerialBaud);
    WiFi.mode(WIFI_STA);

    MacAddress this_mac_address = GetThisMacAddress();
    auto maybe_this_node = Node::FindNode(this_mac_address);
    if (!maybe_this_node) {
        Die(("Failed to find this node: " + this_mac_address.ToString())
                .c_str());
    }

    auto& this_node = maybe_this_node->get();

    ota::CheckForUpdate(this_node.name);

    if (this_node.use_esp_now) {
        EspNowSetup([](auto data, auto len) { Node::OnReceive(data, len); });
    }

    this_node.Run();

    vTaskDelete(nullptr);  // this lets other tasks run forever
}
