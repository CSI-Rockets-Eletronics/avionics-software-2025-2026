#include "device.h"

#include <Arduino.h>

#include <vector>

namespace device {

std::vector<Device*> devices;

Device::Device() { devices.push_back(this); }

Device::~Device() {
    for (auto it = devices.begin(); it != devices.end(); ++it) {
        if (*it == this) {
            devices.erase(it);
            break;
        }
    }
}

void Device::LoopIfSetupSucceeded() {
    if (!setupFailed) {
        Loop();
    }
}

void Device::SetupDie(std::string msg) {
    setupFailed = true;
    Serial.println(msg.c_str());
}

void SetupAll() {
    for (auto dev : devices) {
        dev->Setup();
    }
}

void LoopAll() {
    for (auto dev : devices) {
        dev->LoopIfSetupSucceeded();
    }
}

}  // namespace device
