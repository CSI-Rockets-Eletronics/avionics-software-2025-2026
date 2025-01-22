#include "ota.h"

#include <Arduino.h>
#include <WiFi.h>

#include "ota_version.h"

namespace ota {

// try this long to connect to wifi before giving up on OTA update
static const int kWifiTimeoutMs = 5000;
// give up if a request takes longer than this
static const int kHttpTimeoutMs = 10000;

static const char *kWifiSsid = "Columbia University";
static const char *kWifiPassword = nullptr;

static const char *kHost = "csiwiki.me.columbia.edu";
static const int kPort = 4009;

static const std::string kNoUpdateRequired = "No update required!";

static const size_t buffer_size = 4 * 1024;  // TODO make bigger

void Die(const char *msg) {
    Serial.println(msg);
    esp_restart();
}

void HttpConnectAndSkipToBody(WiFiClient &client, std::string path,
                              uint8_t *buffer, size_t buffer_size) {
    client.setTimeout(kHttpTimeoutMs);
    client.connect(kHost, kPort);

    // send headers
    client.print("GET ");
    client.print(path.c_str());
    client.print(" HTTP/1.1\r\n");

    client.print("Host: ");
    client.print(kHost);
    client.print("\r\n");

    client.print("Connection: close\r\n");
    client.print("\r\n");

    // read HTTP status
    int count = client.readBytesUntil('\n', buffer, buffer_size - 1);
    buffer[count] = '\0';  // terminate string
    int status;
    if (sscanf((const char *)buffer, "HTTP/1.1 %d", &status) != 1) {
        Die("Failed to parse HTTP status");
    }
    if (status != 200) {
        Die("HTTP request failed");
    }

    // skip other headers
    while (true) {
        int count = client.readBytesUntil('\n', buffer, buffer_size);
        if (count == 0) Die("Failed to read HTTP header");
        if (count == 1) break;  // found line with just '\r\n'
    }
}

std::string HttpGetBody(std::string path) {
    uint8_t *buffer = new uint8_t[buffer_size];  // won't fit on stack
    WiFiClient client;

    HttpConnectAndSkipToBody(client, path, buffer, buffer_size);
    size_t len = client.readBytes(buffer, buffer_size - 1);
    buffer[len] = '\0';  // terminate string
    std::string body((const char *)buffer);

    delete[] buffer;
    client.stop();

    return body;
}

void ConnectWifi() {
    WiFi.begin(kWifiSsid, kWifiPassword);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < kWifiTimeoutMs) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
}

void CheckForUpdates(std::string name) {
    Serial.print("Connecting to ");
    Serial.print(kWifiSsid);
    Serial.println(" to check for OTA updates");

    ConnectWifi();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to WiFi -- skipping OTA");
    }

    Serial.println("Checking for OTA updates...");

    std::string needUpdateBody = HttpGetBody(
        std::string("/need-update?name=" + name + "&version=" + OTA_VERSION));

    Serial.print("GET /need-update: ");
    Serial.println(needUpdateBody.c_str());

    if (needUpdateBody == kNoUpdateRequired) {
        return;
    }
}

}  // namespace ota
