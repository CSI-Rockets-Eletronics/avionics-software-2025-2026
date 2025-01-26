#include "avionics.h"

#include <Arduino.h>
#include <esp_now.h>

#include <unordered_map>

namespace avionics {

static const int kQueueNumEntries = 32;

// entry format: [len][data], where len is a uint8_t
static const int kQueueEntrySize = sizeof(uint8_t) + ESP_NOW_MAX_DATA_LEN;

static const int kDeviceLoopTaskStackSize = 4 * 1024;  // 4KB
static const int kDeviceLoopTaskCore = 0;

void Die(const char* msg) {
    Serial.println(msg);
    esp_restart();
}

Device::Device() {
    recv_queue_ = xQueueCreate(kQueueNumEntries, kQueueEntrySize);

    if (recv_queue_ == nullptr) {
        Die("Failed to create receive queue");
    }
}

void Device::QueueReceive(const uint8_t* bytes, size_t len) {
    uint8_t buf[kQueueEntrySize];

    if (uxQueueSpacesAvailable(recv_queue_) == 0) {
        // queue is full, drop oldest message
        xQueueReceive(recv_queue_, buf, 0);
    }

    memset(buf, 0, kQueueEntrySize);
    buf[0] = len;
    memcpy(buf + 1, bytes, len);

    if (xQueueSend(recv_queue_, buf, 0) != pdTRUE) {
        Serial.println("Failed to queue received message");
    }
}

void Device::Die(const char* msg) { avionics::Die(msg); }

void Device::Send(DeviceType to_device, const uint8_t* bytes, size_t len) {
    auto maybe_to_node = Node::FindNode(to_device);
    if (!maybe_to_node) {
        Serial.println("Send: Node not found, ignoring...");
        return;
    }
    auto& to_node = maybe_to_node->get();

    // packet format: [DeviceType][data]
    size_t dev_header_len = sizeof(DeviceType);
    uint8_t buf[dev_header_len + len];
    memcpy(buf, &to_device, dev_header_len);
    memcpy(buf + dev_header_len, bytes, len);

    EspNowSend(to_node.mac_address, buf, sizeof(buf));
}

bool Device::Receive(uint8_t* out, size_t len, bool put_back_if_len_mismatch) {
    uint8_t buf[kQueueEntrySize];
    if (xQueueReceive(recv_queue_, buf, 0) != pdTRUE) {
        return false;
    }

    if (buf[0] != len) {
        if (put_back_if_len_mismatch) {
            xQueueSendToFront(recv_queue_, buf, 0);  // drop if queue is full
        } else {
            Serial.println("Invalid message length, ignoring...");
        }
        return false;
    }

    memcpy(out, buf + 1, len);
    return true;
}

static std::unordered_map<DeviceType, std::function<std::unique_ptr<Device>()>>
    device_factories;

static std::vector<Node*> all_nodes;

void _register::RegisterDeviceFactory(
    DeviceType type, std::function<std::unique_ptr<Device>()> factory) {
    device_factories[type] = factory;
}

Node::Node(std::string name, MacAddress mac_address,
           std::vector<DeviceType> device_types, bool use_esp_now)
    : name(name),
      mac_address(mac_address),
      device_types(device_types),
      use_esp_now(use_esp_now) {
    all_nodes.push_back(this);
}

void Node::Run() {
    for (auto type : device_types) {
        auto factory = device_factories[type];
        if (!factory) {
            Die("Device factory not found");
        }
        devices_.push_back(factory());
    }

    for (auto& dev : devices_) {
        dev->Setup();

        // certain libraries crash if the task isn't pinned to core 0
        auto res = xTaskCreatePinnedToCore(
            [](void* param) {
                Device* device = static_cast<Device*>(param);
                while (true) {
                    device->Loop();
                }
            },
            "DeviceLoopTask", kDeviceLoopTaskStackSize, dev.get(),
            tskIDLE_PRIORITY, nullptr, kDeviceLoopTaskCore);

        if (res != pdPASS) {
            Die("Failed to create device loop task");
        }
    }
}

std::optional<std::reference_wrapper<Node>> Node::FindNode(MacAddress mac) {
    for (auto node : all_nodes) {
        if (node->mac_address == mac) {
            return *node;
        }
    }
    return std::nullopt;
}

std::optional<std::reference_wrapper<Node>> Node::FindNode(DeviceType device) {
    for (auto node : all_nodes) {
        for (auto node_dev : node->device_types) {
            if (node_dev == device) {
                return *node;
            }
        }
    }
    return std::nullopt;
}

std::optional<std::reference_wrapper<Device>> Node::FindDevice(
    DeviceType device) {
    auto maybe_node = FindNode(device);
    if (!maybe_node) {
        return std::nullopt;
    }
    auto& node = maybe_node->get();

    // is node initialized?
    if (node.devices_.size() != node.device_types.size()) {
        return std::nullopt;
    }

    for (int i = 0; i < node.device_types.size(); i++) {
        if (node.device_types[i] == device) {
            return *node.devices_[i];
        }
    }

    return std::nullopt;
}

std::vector<MacAddress> Node::AllMacAddresses() {
    std::vector<MacAddress> macs;
    for (auto node : all_nodes) {
        macs.push_back(node->mac_address);
    }
    return macs;
}

void Node::OnReceive(uint8_t* bytes, size_t len) {
    if (len < sizeof(DeviceType)) {
        Serial.println("Invalid message length, ignoring...");
        return;
    }

    DeviceType from_device = *reinterpret_cast<const DeviceType*>(bytes);
    auto maybe_dev = FindDevice(from_device);
    if (!maybe_dev) {
        Serial.println("Device not found, ignoring...");
        return;
    }
    auto& dev = maybe_dev->get();

    dev.QueueReceive(bytes + sizeof(DeviceType), len - sizeof(DeviceType));
}

}  // namespace avionics
