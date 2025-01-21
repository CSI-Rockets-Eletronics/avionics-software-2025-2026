#include "ota.h"

#include <Arduino.h>
#include <WiFi.h>

namespace ota {

// try this long to connect to wifi before giving up on OTA update
static const int kWifiTimeoutMs = 5000;
// give up if a request takes longer than this
static const int kHttpTimeoutMs = 10000;

static const char *wifiSsid = "Columbia University";
static const char *wifiPassword = nullptr;

static const char *kHost = "csiwiki.me.columbia.edu";
static const int kPort = 4009;

static const size_t buffer_size = 4 * 1024;  // TODO make bigger

void Die(const char *msg) {
    Serial.println(msg);
    esp_restart();
}

/**
 * Stores the response body into the buffer and returns the number of bytes
 * read.
 */
size_t HttpGetBody(std::string path, uint8_t *buffer, size_t buffer_size) {
    // connect to server
    WiFiClient client;
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
    while (client.connected()) {
        int count = client.readBytesUntil('\n', buffer, buffer_size);
        if (count == 1) break;  // found line with just '\r\n'
    }
    if (!client.connected()) {
        Die("Connection lost");
    }

    // read response body
    size_t len = client.readBytes(buffer, buffer_size);

    // clean up
    client.stop();
    return len;
}

void otaTask(void *pvParameters) {
    Serial.println("Checking for OTA updates...");

    // test stuff

    // won't fit on the stack of the task
    uint8_t *buffer = new uint8_t[buffer_size];

    size_t len = HttpGetBody(
        "http://csiwiki.me.columbia.edu:4009/need-update?name=foo&version=bar",
        buffer, buffer_size - 1);

    buffer[len] = '\0';  // terminate string
    Serial.print("Read: ");
    Serial.println((const char *)buffer);

    delete[] buffer;
}

void Start() {
    Serial.print("Connecting to ");
    Serial.print(wifiSsid);
    Serial.println(" to check for OTA updates");

    WiFi.begin(wifiSsid, wifiPassword);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - start > kWifiTimeoutMs) {
            Serial.println("Failed to connect to WiFi -- skipping OTA");
            return;
        }
    }

    // TODO priority?
    xTaskCreate(otaTask, "otaTask", 8192, NULL, 1, NULL);
}

}  // namespace ota
