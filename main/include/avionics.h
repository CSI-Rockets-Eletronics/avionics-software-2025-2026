#ifndef AVIONICS_H_
#define AVIONICS_H_

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace avionics {

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
    // reboots the device
    void SetupDie(std::string msg);
};

class Node {
   public:
    Node(std::initializer_list<DeviceType> device_types) : device_types_{device_types} {}
    void Setup();
    void Loop();

   private:
    const std::vector<DeviceType> device_types_;
    std::vector<std::unique_ptr<Device>> devices_;  // uninitialized unless Setup() is called
};

namespace _register {

template <typename T>
concept IsDevice = std::is_base_of_v<Device, T>;

void RegisterDeviceFactory(DeviceType type, std::function<std::unique_ptr<Device>()> factory);

template <IsDevice Dev, DeviceType type>
class RegisterDevice {
   public:
    RegisterDevice() {
        RegisterDeviceFactory(type, []() { return std::make_unique<Dev>(); });
    }
};

}  // namespace _register

#define REGISTER_DEVICE(Dev) static _register::RegisterDevice<Dev, DeviceType::Dev> reg

}  // namespace avionics

#endif  // AVIONICS_H_
