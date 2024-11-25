#ifndef AVIONICS_H_
#define AVIONICS_H_

#include <functional>
#include <memory>
#include <vector>

#include "comms.h"

namespace avionics {

void Die(const char* msg);

enum class DeviceType {
    // sort alphabetically
    DevDhtImu,
    DevGps,
    DevRadio,
};

class Device {
   public:
    virtual void Setup() = 0;
    virtual void Loop() = 0;

   protected:
    void Die(const char* msg);
};

class Node {
   public:
    const MacAddress mac_address;

    Node(const MacAddress mac_address,
         const std::vector<DeviceType> device_types)
        : mac_address(mac_address), device_types_(device_types) {}
    void Setup();
    void Loop();

   private:
    const std::vector<DeviceType> device_types_;
    // uninitialized unless Setup() is called
    std::vector<std::unique_ptr<Device>> devices_;
};

namespace _register {

template <typename T>
concept IsDevice = std::is_base_of_v<Device, T>;

void RegisterDeviceFactory(DeviceType type,
                           std::function<std::unique_ptr<Device>()> factory);

template <IsDevice Dev, DeviceType type>
class RegisterDevice {
   public:
    RegisterDevice() {
        RegisterDeviceFactory(type, []() { return std::make_unique<Dev>(); });
    }
};

}  // namespace _register

}  // namespace avionics

#define REGISTER_AVIONICS_DEVICE(Dev)                                          \
    static avionics::_register::RegisterDevice<Dev, avionics::DeviceType::Dev> \
        reg

#endif  // AVIONICS_H_
