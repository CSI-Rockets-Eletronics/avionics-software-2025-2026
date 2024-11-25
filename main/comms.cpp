#include "comms.h"

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <wifi.h>

namespace avionics {

static void die(const char* msg) {
    Serial.println(msg);
    esp_restart();
}

// don't bother synchronizing -- it's okay to drop messages occasionally
static bool send_in_flight = false;

static void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) { send_in_flight = false; }

EspNow::EspNow(const std::vector<MacAddress>& mac_addresses) {
    WiFi.mode(WIFI_STA);

    MacAddress this_mac_address;

    if (esp_wifi_get_mac(WIFI_IF_STA, this_mac_address.data()) != ESP_OK) {
        die("Failed to get ESP-NOW MAC address");
    }

    if (esp_now_init() != ESP_OK) {
        die("Failed to initialize ESP-NOW");
    }

    if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
        die("Failed to register ESP-NOW send callback");
    }

    for (const auto& mac_address : mac_addresses) {
        if (mac_address == this_mac_address) {
            continue;
        }

        esp_now_peer_info_t peer_info;
        std::copy(mac_address.begin(), mac_address.end(), peer_info.peer_addr);
        peer_info.channel = 0;
        peer_info.encrypt = false;

        if (esp_now_add_peer(&peer_info) != ESP_OK) {
            die("Failed to add ESP-NOW peer");
        }
    }
}

void EspNow::SendBytes(const MacAddress& to_address, const uint8_t* bytes, size_t len) {
    if (send_in_flight) {
        return;
    }

    if (len > ESP_NOW_MAX_DATA_LEN) {
        Serial.println("Warning: ESP-NOW message too large");
        return;
    }

    send_in_flight = true;

    if (esp_now_send(to_address.data(), bytes, len) != ESP_OK) {
        send_in_flight = false;
        Serial.println("Warning: Failed to send ESP-NOW message");
        return;
    }
}

}  // namespace avionics
