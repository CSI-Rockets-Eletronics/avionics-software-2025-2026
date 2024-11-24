#include "device.h"

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

void InitAll() {
    for (auto dev : devices) {
        dev->init();
    }
}

void LoopAll() {
    for (auto dev : devices) {
        dev->loop();
    }
}

}  // namespace device