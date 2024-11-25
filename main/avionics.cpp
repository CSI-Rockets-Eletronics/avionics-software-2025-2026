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

void _register::RegisterDeviceFactory(
    DeviceType type, std::function<std::unique_ptr<Device>()> factory) {
    device_factories[type] = factory;
}

void Node::Setup() {
    for (auto type : device_types_) {
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
