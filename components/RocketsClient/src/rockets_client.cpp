#include "rockets_client.h"

#include <WiFi.h>

#include "frequency_logger.h"

namespace rockets_client {

// wifi presets

WifiConfigPresets wifiConfigPresets = {
    .GROUND =
        {
            .ssid = "Darknet",
            .password = "Blueberries",
        },
    .ROCKET =
        {
            .ssid = "Darknet Rocket",
            .password = "Blueberries",
        },
};

// server presets

ServerConfigPresets serverConfigPresets = {
    .FS_PI =
        {
            .host = "fs-pi.local",
            .port = 3000,
            .pathPrefix = "",
        },
    .ROCKET_PI =
        {
            .host = "rocket-pi.local",
            .port = 3000,
            .pathPrefix = "",
        },
    .MECHE =
        {
            .host = "csiwiki.me.columbia.edu",
            .port = 3001,
            .pathPrefix = "/rocketsdata2",
        },
    .ALEX_LAPTOP =
        {
            .host = "csi-alex-laptop-data-server.ngrok.io",
            .port = 80,
            .pathPrefix = "",
        },
    .ALEX_HOME_DESKTOP =
        {
            .host = "xdxdxdxdxd.mynetgear.com",
            .port = 3000,
            .pathPrefix = "",
        },
};

// general parameters

const int64_t MAX_TS_DELAY_MS = 200;  // max allowed delay w.r.t. server time

// task parameters

const int CORE_ID = 0;
const int PRIORITY = 0;
const int STACK_DEPTH = 64 * 1000;  // 64 kB

// constants specific to this client

String WIFI_SSID;
String WIFI_PASSWORD;

String HOST;
int PORT;
String PATH_PREFIX;

String ENVIRONMENT_KEY;
String DEVICE;

bool POLL_MESSAGES;
String POLL_RECORD_DEVICES;

// for triggering ts sync
bool syncTsRequested = false;

// buffers for queued record and latest message

// length 0 if no record is queued
Buffer queuedRecord = "";
// empty if there is no new message since the last retrieval
StaticJsonDoc latestMessage;
// empty only at the beginning, before the first poll completes
StaticJsonDoc latestRecords;

// for uploading records

// add this to esp_timer_get_time() to get the absolute timestamp
int64_t absoluteTsOffset = 0;

// for fetching new messages

int64_t lastMessageTs = 0;

// mutexes for the buffers

SemaphoreHandle_t queuedRecordMutex;
SemaphoreHandle_t latestMessageMutex;
SemaphoreHandle_t latestRecordsMutex;

// for logging

FrequencyLogger frequencyLogger = FrequencyLogger("rockets client", 1000);

// private functions

// returns true if successful and queued record is non-empty
bool getQueuedRecord(Buffer dest) {
    if (xSemaphoreTake(queuedRecordMutex, portMAX_DELAY)) {
        strncpy(dest, queuedRecord, sizeof(Buffer));
        queuedRecord[0] = '\0';
        xSemaphoreGive(queuedRecordMutex);
        return strlen(dest) > 0;
    } else {
        return false;
    }
}

// returns true if successful
bool setLatestMessage(const Buffer src) {
    StaticJsonDoc doc;
    auto err = deserializeJson(doc, src);

    if (err != DeserializationError::Ok) {
        Serial.print("Message response deserialization failed: ");
        Serial.println(err.f_str());
        return false;
    }

    if (!doc.containsKey("ts")) {
        Serial.println("Message response missing ts");
        return false;
    }

    // no mutex needed; lastMessageTs is only accessed by this task
    lastMessageTs = doc["ts"];

    if (xSemaphoreTake(latestMessageMutex, portMAX_DELAY)) {
        latestMessage = doc;
        xSemaphoreGive(latestMessageMutex);
        return true;
    } else {
        return false;
    }
}

// returns true if successful
bool setLatestRecords(const Buffer src) {
    StaticJsonDoc doc;
    auto err = deserializeJson(doc, src);

    if (err != DeserializationError::Ok) {
        Serial.print("Poll records response deserialization failed: ");
        Serial.println(err.f_str());
        return false;
    }

    if (xSemaphoreTake(latestRecordsMutex, portMAX_DELAY)) {
        latestRecords = doc;
        xSemaphoreGive(latestRecordsMutex);
        return true;
    } else {
        return false;
    }
}

void printHeartbeat() {
    frequencyLogger.tick();

    // Serial.print("Free heap memory (bytes): ");
    // Serial.println(esp_get_free_heap_size());

    // Serial.print("Network heartbeat: ");
    if (WiFi.status() == WL_CONNECTED) {
        // Serial.println("WiFi connected");
    } else {
        Serial.println("WiFi not connected");
    }
}

// body is ignored for GET requests
void postReq(String method, WiFiClient& client, String path, const char* body) {
    String fullPath = PATH_PREFIX + path;

    client.setTimeout(3);

    client.print(method);
    client.print(" ");
    client.print(fullPath);
    client.println(" HTTP/1.1");

    client.print("Host: ");
    client.println(HOST);

    client.println("Connection: close");

    if (method == "GET") {
        client.println();
    } else {
        client.println("Content-Type: application/json");

        client.print("Content-Length: ");
        client.println(strlen(body));

        client.println();
        client.print(body);
    }
}

// returns true if successful
bool readRes(WiFiClient& client, Buffer dest) {
    int TIMEOUT = 3000;

    int i = 0;
    int startTime = millis();

    while (client.connected()) {
        if (millis() - startTime > TIMEOUT) {
            return false;
        }

        while (client.available()) {
            if (millis() - startTime > TIMEOUT) {
                return false;
            }

            char c = client.read();
            if (i < sizeof(Buffer) - 1) {
                dest[i] = c;
                i++;
            }
        }

        yield();
    }

    dest[i] = '\0';
    return true;
}

// returns HTTP status, or 0 if error
int parseResBody(const Buffer res, Buffer dest) {
    int status;

    bool readStatus = sscanf(res, "HTTP/1.1 %d", &status);
    if (!readStatus) {
        return 0;
    }

    // read until empty line (end of headers)
    const char* lineStart = res;
    while (true) {
        const char* lineEnd = strstr(lineStart, "\r\n");
        if (lineEnd == NULL) {
            return 0;
        }
        if (lineEnd == lineStart) {
            lineStart += 2;
            break;
        }
        lineStart = lineEnd + 2;
    }

    strncpy(dest, lineStart, sizeof(Buffer));
    return status;
}

// Returns true if successful.
bool trySyncTs(WiFiClient& client) {
    bool success = false;

    String path = "/ts";

    if (client.connect(HOST.c_str(), PORT)) {
        Serial.println("Syncing ts");

        postReq("GET", client, path, NULL);

        Buffer res;
        if (readRes(client, res)) {
            Buffer resBody;
            int status = parseResBody(res, resBody);

            if (status == 200) {
                Serial.println("syncTs success");
                int64_t syncedTs = strtoll(resBody, NULL, 10);

                absoluteTsOffset = syncedTs - esp_timer_get_time();
                lastMessageTs = syncedTs;

                success = true;
            } else {
                Serial.print("syncTs failed, status: ");
                Serial.println(status);
                Serial.println(resBody);
            }
        } else {
            Serial.println("syncTs failed, network timeout");
        }
    } else {
        Serial.println("syncTs connect failed");
    }

    client.stop();
    return success;
}

void syncTs(WiFiClient& client) {
    while (true) {
        int64_t start = esp_timer_get_time();
        bool success = trySyncTs(client);
        int64_t end = esp_timer_get_time();

        delay(5);

        if (!success) {
            Serial.println("trySyncTs failed, retrying...");
            continue;
        }

        int64_t elapsed_ms = (end - start) / 1000;

        Serial.print("trySyncTs took (ms): ");
        Serial.println(elapsed_ms);

        if (elapsed_ms > MAX_TS_DELAY_MS) {
            Serial.println("trySyncTs took too long, retrying...");
            continue;
        }

        break;
    }
}

void sendQueuedRecord(WiFiClient& client) {
    Buffer record;
    if (!getQueuedRecord(record)) {
        return;
    }

    if (client.connect(HOST.c_str(), PORT)) {
        // Serial.println("Sending queued record");

        postReq("POST", client, "/records", record);

        Buffer res;
        if (readRes(client, res)) {
            Buffer resBody;
            int status = parseResBody(res, resBody);

            if (status == 200) {
                // Serial.println("sendQueuedRecord success");
            } else {
                Serial.print("sendQueuedRecord failed, status: ");
                Serial.println(status);
                Serial.println(resBody);
            }
        } else {
            Serial.println("sendQueuedRecord failed, network timeout");
        }
    } else {
        Serial.println("sendQueuedRecord connect failed");
    }

    client.stop();
}

void pollLatestMessage(WiFiClient& client) {
    String path = "/messages/next?environmentKey=" + ENVIRONMENT_KEY +
                  "&device=" + DEVICE + "&afterTs=" + lastMessageTs;

    if (client.connect(HOST.c_str(), PORT)) {
        // Serial.println("Polling latest message");

        postReq("GET", client, path, NULL);

        Buffer res;
        if (readRes(client, res)) {
            Buffer resBody;
            int status = parseResBody(res, resBody);

            if (status == 200) {
                // Serial.println("pollLatestMessage success");

                if (strcmp(resBody, "NONE") != 0) {
                    setLatestMessage(resBody);
                }
            } else {
                Serial.print("pollLatestMessage failed, status: ");
                Serial.println(status);
                Serial.println(resBody);
            }
        } else {
            Serial.println("pollLatestMessage failed, network timeout");
        }
    } else {
        Serial.println("pollLatestMessage connect failed");
    }

    client.stop();
}

void pollLatestRecords(WiFiClient& client) {
    String path = "/records/multiDevice?environmentKey=" + ENVIRONMENT_KEY +
                  "&devices=" + POLL_RECORD_DEVICES;

    if (client.connect(HOST.c_str(), PORT)) {
        // Serial.println("Polling latest records");

        postReq("GET", client, path, NULL);

        Buffer res;
        if (readRes(client, res)) {
            Buffer resBody;
            int status = parseResBody(res, resBody);

            if (status == 200) {
                // Serial.println("pollLatestRecords success");

                setLatestRecords(resBody);
            } else {
                Serial.print("pollLatestRecords failed, status: ");
                Serial.println(status);
                Serial.println(resBody);
            }
        } else {
            Serial.println("pollLatestRecords failed, network timeout");
        }
    } else {
        Serial.println("pollLatestRecords connect failed");
    }

    client.stop();
}

void runTask(void* pvParameters) {
    WiFiClient client;

    syncTs(client);

    while (true) {
        printHeartbeat();

        if (syncTsRequested) {
            syncTs(client);
            syncTsRequested = false;
        }

        sendQueuedRecord(client);
        delay(5);

        if (POLL_MESSAGES) {
            pollLatestMessage(client);
            delay(5);
        }

        if (POLL_RECORD_DEVICES.length() > 0) {
            pollLatestRecords(client);
            delay(5);
        }
    }
}

void initTask() {
    queuedRecordMutex = xSemaphoreCreateMutex();
    latestMessageMutex = xSemaphoreCreateMutex();
    latestRecordsMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(runTask, "network", STACK_DEPTH, NULL, PRIORITY,
                            NULL, CORE_ID);
}

void initWifi() {
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // block until connected
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// implementation of the interface

void syncTimestamp() { syncTsRequested = true; }

bool queueRecord(const StaticJsonDoc& recordData) {
    StaticJsonDoc record;

    record["environmentKey"] = ENVIRONMENT_KEY;
    record["device"] = DEVICE;
    record["ts"] = esp_timer_get_time() + absoluteTsOffset;

    JsonObject data = record.createNestedObject("data");
    data.set(recordData.as<JsonObjectConst>());

    if (xSemaphoreTake(queuedRecordMutex, portMAX_DELAY)) {
        serializeJson(record, queuedRecord, sizeof(Buffer));
        xSemaphoreGive(queuedRecordMutex);
        return true;
    } else {
        return false;
    }
}

StaticJsonDoc getLatestMessage() {
    StaticJsonDoc doc;

    if (xSemaphoreTake(latestMessageMutex, portMAX_DELAY)) {
        doc = latestMessage;
        latestMessage = StaticJsonDoc();
        xSemaphoreGive(latestMessageMutex);
        return doc;
    } else {
        // return empty json object
        return doc;
    }
}

StaticJsonDoc getLatestRecords() {
    StaticJsonDoc doc;

    if (xSemaphoreTake(latestRecordsMutex, portMAX_DELAY)) {
        doc = latestRecords;
        xSemaphoreGive(latestRecordsMutex);
        return doc;
    } else {
        // return empty json object
        return doc;
    }
}

// `pollRecordDevices` should be a comma-separated list of devices, e.g.
// "deviceA,deviceB". Empty string for `pollRecordDevices` means don't poll.
void init(WifiConfig wifiConfig, ServerConfig serverConfig,
          String environmentKey, String device, bool pollMessages,
          String pollRecordDevices) {
    WIFI_SSID = wifiConfig.ssid;
    WIFI_PASSWORD = wifiConfig.password;

    HOST = serverConfig.host;
    PORT = serverConfig.port;
    PATH_PREFIX = serverConfig.pathPrefix;

    ENVIRONMENT_KEY = environmentKey;
    DEVICE = device;

    POLL_MESSAGES = pollMessages;
    POLL_RECORD_DEVICES = pollRecordDevices;

    initWifi();
    initTask();
}

}  // namespace rockets_client