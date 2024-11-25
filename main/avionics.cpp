#include "avionics.h"

#include <Arduino.h>

#include <unordered_map>

namespace avionics {

void Die(const char* msg) {
    Serial.println(msg);
    esp_restart();
}

void Device::Die(const char* msg) { avionics::Die(msg); }

static std::unordered_map<DeviceType, std::function<std::unique_ptr<Device>()>>
    device_factories;

static std::vector<Node*> all_nodes;

Node& FindNode(DeviceType device) {
    for (auto node : all_nodes) {
        for (auto node_dev : node->device_types) {
            if (node_dev == device) {
                return *node;
            }
        }
    }
    Die("Node not found");
    return *all_nodes[0];  // unreachable
}

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
        devices_.push_back(factory());
    }

    for (auto& dev : devices_) {
        dev->Setup();
    }
}

void Node::Loop() {
    for (auto& dev : devices_) {
        dev->Loop();
    }
}

}  // namespace avionics
