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
    delay(1000);  // Give serial time to initialize

    Serial.println("\n\n=== STARTING APP_MAIN ===");

    // Simple heartbeat to confirm code is running
    for (int i = 0; i < 5; i++) {
        Serial.print("working ");
        Serial.println(i);
        delay(500);
    }

    Serial.println("Setting WiFi mode to STA");
    WiFi.mode(WIFI_STA);

    Serial.println("Getting MAC address");
    MacAddress this_mac_address = GetThisMacAddress();
    Serial.print("This MAC: ");
    Serial.println(this_mac_address.ToString().c_str());

    Serial.println("Finding node configuration");
    auto maybe_this_node = Node::FindNode(this_mac_address);
    if (!maybe_this_node) {
        Die(("Failed to find this node: " + this_mac_address.ToString())
                .c_str());
    }

    auto& this_node = maybe_this_node->get();
    Serial.print("Node found: ");
    Serial.println(this_node.name.c_str());

    // ota::CheckForUpdate(this_node.name);

    if (this_node.use_esp_now) {
        Serial.println("Setting up ESP-NOW");
        EspNowSetup([](auto data, auto len) { Node::OnReceive(data, len); });
        Serial.println("ESP-NOW setup complete");
    }

    Serial.println("Initialization complete - all readings ready");
    Serial.println("Starting device tasks...");

    this_node.Run();

    Serial.println("Device tasks started - main task exiting");
    vTaskDelete(nullptr);  // this lets other tasks run forever
}
