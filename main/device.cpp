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

void Device::LoopIfInit() {
    if (!init_failed_) {
        Loop();
    }
}

void Device::InitDie(std::string msg) {
    init_failed_ = true;
    Serial.println(msg.c_str());
}

void InitAll() {
    for (auto dev : devices) {
        dev->Init();
    }
}

void LoopAll() {
    for (auto dev : devices) {
        dev->LoopIfInit();
    }
}

}  // namespace device
