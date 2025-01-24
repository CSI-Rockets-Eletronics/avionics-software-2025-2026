#include "ota.h"

#include <Arduino.h>
#include <WiFi.h>

#include "esp_ota_ops.h"
#include "nvs_flash.h"
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

static const size_t buffer_size = 16 * 1024;  // 16 KB

void Die(const char *msg) {
    Serial.println(msg);
    esp_restart();
}

bool Diagnostic() {
    // TODO
    // https://github.com/espressif/esp-idf/blob/v5.4/examples/system/ota/native_ota_example/main/native_ota_example.c#L257
    return true;
}

void CheckRunningPartition() {
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        Serial.println(
            "Note: OTA running partition is different from configured");
    }

    esp_ota_img_states_t ota_state;

    esp_err_t err = esp_ota_get_state_partition(running, &ota_state);
    if (err == ESP_ERR_NOT_SUPPORTED) {
        Serial.println("Note: not currently booting from OTA");
        return;
    }
    if (err != ESP_OK) {
        Die("Failed to get OTA state");
    }

    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        bool diagnostic_is_ok = Diagnostic();
        if (diagnostic_is_ok) {
            Serial.println("OTA diagnostics passed!");
            esp_ota_mark_app_valid_cancel_rollback();
        } else {
            Serial.println("OTA diagnostics failed! Rolling back...");
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }
}

void InitNvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // OTA app partition table has a smaller NVS partition size than the
        // non-OTA partition table. This size mismatch may cause NVS
        // initialization to fail. If this happens, we erase NVS partition and
        // initialize NVS again.
        if (nvs_flash_erase() != ESP_OK) {
            Die("Failed to erase NVS");
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        Die("Failed to initialize NVS");
    }
}

// returns the parsed Content-Length header
size_t HttpConnectAndSkipToBody(WiFiClient &client, std::string path,
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

    // look for Content-Length header, skipping all headers

    size_t content_length = 0;

    while (true) {
        int count = client.readBytesUntil('\n', buffer, buffer_size - 1);
        buffer[count] = '\0';  // terminate string

        if (count == 0) Die("Failed to read HTTP header");
        if (count == 1) break;  // found line with just '\r\n'

        // writes to content_length if the header matches
        sscanf((const char *)buffer, "Content-Length: %zu", &content_length);
    }

    if (content_length == 0) {
        Die("Failed to find Content-Length header");
    }

    return content_length;
}

std::string HttpGetBody(std::string path) {
    uint8_t *buffer = new uint8_t[buffer_size];  // won't fit on stack
    WiFiClient client;

    size_t content_length =
        HttpConnectAndSkipToBody(client, path, buffer, buffer_size);

    if (content_length > buffer_size - 1) {
        Die("Content-Length too large");
    }

    size_t len = client.readBytes(buffer, content_length);
    buffer[len] = '\0';  // terminate string

    if (len != content_length) {
        Die("Failed to read all HTTP body bytes");
    }

    std::string body((const char *)buffer);

    delete[] buffer;
    client.stop();

    return body;
}

void DownloadAndFlash(std::string path) {
    InitNvs();

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL) {
        Die("Failed to get OTA partition");
    }

    esp_ota_handle_t update_handle = 0;
    if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES,
                      &update_handle) != ESP_OK) {
        Die("Failed to begin OTA");
    }

    uint8_t *buffer = new uint8_t[buffer_size];  // won't fit on stack
    WiFiClient client;

    size_t content_length =
        HttpConnectAndSkipToBody(client, path, buffer, buffer_size);

    size_t processed_length = 0;

    while (true) {
        size_t remaining = content_length - processed_length;
        size_t len = client.readBytes(buffer, min(remaining, buffer_size));

        if (len == 0) {
            Die("Failed to read OTA bytes");
        }

        if (esp_ota_write(update_handle, buffer, len) != ESP_OK) {
            Die("Failed to write OTA");
        }

        processed_length += len;

        float percentage = (float)processed_length / content_length * 100;
        Serial.print("OTA progress: ");
        Serial.print(percentage, 2);
        Serial.println("%");

        if (processed_length > content_length) {
            Die("Read more OTA bytes than expected");
        }
        if (processed_length == content_length) {
            break;
        }
    }

    delete[] buffer;
    client.stop();

    if (esp_ota_end(update_handle) != ESP_OK) {
        Die("Failed to end OTA");
    }

    if (esp_ota_set_boot_partition(update_partition) != ESP_OK) {
        Die("Failed to set boot partition");
    }

    Serial.println("OTA complete! Rebooting...");
    esp_restart();
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

void CheckAndPerformUpdate(std::string name) {
    Serial.println("Checking for OTA updates...");

    std::string needUpdateBody = HttpGetBody(
        std::string("/need-update?name=" + name + "&version=" + OTA_VERSION));

    Serial.print("GET /need-update: ");
    Serial.println(needUpdateBody.c_str());

    if (needUpdateBody == kNoUpdateRequired) {
        return;
    }

    Serial.println("Downloading and flashing OTA update...");

    DownloadAndFlash("/program.bin");
}

void CheckForUpdate(std::string name) {
    CheckRunningPartition();

    Serial.print("Connecting to ");
    Serial.print(kWifiSsid);
    Serial.println(" to check for OTA updates");

    ConnectWifi();

    if (WiFi.status() == WL_CONNECTED) {
        CheckAndPerformUpdate(name);
    } else {
        Serial.println("Failed to connect to WiFi -- skipping OTA");
    }

    if (!WiFi.disconnect()) {
        Die("Failed to disconnect from WiFi");
    }
}

}  // namespace ota
