#include "comms.h"

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <wifi.h>

#include "nodeconfig.h"

namespace avionics {

MacAddress::MacAddress() {}

MacAddress::MacAddress(std::string str_address) {
    if (str_address.size() != 17) {
        Die("Invalid MAC address");
    }
    for (size_t i = 0; i < 6; i++) {
        bytes_[i] = std::stoi(str_address.substr(i * 3, 2), nullptr, 16);
    }
}

std::string MacAddress::ToString() const {
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", bytes_[0],
             bytes_[1], bytes_[2], bytes_[3], bytes_[4], bytes_[5]);
    return buf;
}

void MacAddress::CopyInto(uint8_t* dest) const {
    memcpy(dest, bytes_.data(), bytes_.size());
}

uint8_t* MacAddress::Data() { return bytes_.data(); }

const uint8_t* MacAddress::ReadData() const { return bytes_.data(); }

bool MacAddress::operator==(const MacAddress& other) const {
    return bytes_ == other.bytes_;
}

// don't bother synchronizing -- it's okay to drop messages occasionally
static bool send_in_flight = false;

static void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    send_in_flight = false;
}

void EspNowSetup() {
    WiFi.mode(WIFI_STA);

    MacAddress this_mac_address;

    if (esp_wifi_get_mac(WIFI_IF_STA, this_mac_address.Data()) != ESP_OK) {
        Die("Failed to get ESP-NOW MAC address");
    }

    Serial.print("Mac address: ");
    Serial.println(this_mac_address.ToString().c_str());

    if (esp_now_init() != ESP_OK) {
        Die("Failed to initialize ESP-NOW");
    }

    if (esp_now_register_send_cb(OnDataSent) != ESP_OK) {
        Die("Failed to register ESP-NOW send callback");
    }

    for (const auto& mac_address : all_mac_addresses) {
        if (mac_address == this_mac_address) {
            continue;
        }

        esp_now_peer_info_t peer_info;
        mac_address.CopyInto(peer_info.peer_addr);
        peer_info.channel = 0;
        peer_info.encrypt = false;

        if (esp_now_add_peer(&peer_info) != ESP_OK) {
            Die("Failed to add ESP-NOW peer");
        }
    }
}

void EspNowSend(const MacAddress& to_address, const uint8_t* bytes,
                size_t len) {
    if (send_in_flight) {
        return;
    }

    if (len > ESP_NOW_MAX_DATA_LEN) {
        Serial.println("Warning: ESP-NOW message too large");
        return;
    }

    send_in_flight = true;

    if (esp_now_send(to_address.ReadData(), bytes, len) != ESP_OK) {
        send_in_flight = false;
        Serial.println("Warning: Failed to send ESP-NOW message");
        return;
    }
}

}  // namespace avionics
