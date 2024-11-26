#include "avionics.h"

#include <Arduino.h>
#include <esp_now.h>

#include <unordered_map>

namespace avionics {

static const int kQueueNumEntries = 32;

// entry format: [len][data], where len is a uint8_t
static const int kQueueEntrySize = sizeof(uint8_t) + ESP_NOW_MAX_DATA_LEN;

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

bool Device::Receive(uint8_t* out, size_t len) {
    uint8_t buf[kQueueEntrySize];
    if (xQueueReceive(recv_queue_, buf, 0) != pdTRUE) {
        return false;
    }

    if (buf[0] != len) {
        Serial.println("Invalid message length, ignoring...");
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

Node::Node(const MacAddress mac_address,
           const std::vector<DeviceType> device_types)
    : mac_address(mac_address), device_types(device_types) {
    all_nodes.push_back(this);
}

void Node::Setup() {
    for (auto type : device_types) {
        auto factory = device_factories[type];
        if (!factory) {
            Die("Device factory not found");
        }
        devices_.push_back(factory());
    }

    for (auto& dev : devices_) {
        dev->Setup();
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

void Node::Loop() {
    for (auto& dev : devices_) {
        dev->Loop();
    }
}

}  // namespace avionics
