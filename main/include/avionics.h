#ifndef AVIONICS_H_
#define AVIONICS_H_

#include <Arduino.h>

#include <functional>
#include <memory>
#include <optional>
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

    virtual void OnReceive(uint8_t* bytes, size_t len) {}

   protected:
    void Die(const char* msg);

    template <typename T>
    inline void Send(DeviceType to_device, const T& data) {
        Send(to_device, reinterpret_cast<const uint8_t*>(&data), sizeof(data));
    }

    template <typename T>
    inline T& Parse(const uint8_t* bytes, size_t len) {
        if (len != sizeof(T)) {
            Serial.println("Invalid message length, ignoring...");
        }
        return *reinterpret_cast<T*>(bytes);
    }

   private:
    void Send(DeviceType to_device, const uint8_t* bytes, size_t len);
};

class Node {
   public:
    const MacAddress mac_address;
    const std::vector<DeviceType> device_types;

    Node(const MacAddress mac_address,
         const std::vector<DeviceType> device_types);

    Node(const Node&) = delete;

    void Setup();
    void Loop();

    static std::optional<std::reference_wrapper<Node>> FindNode(
        DeviceType type);
    static std::optional<std::reference_wrapper<Device>> FindDevice(
        DeviceType type);

    static void OnReceive(uint8_t* bytes, size_t len);

   private:
    // uninitialized unless Setup() is called
    // if initialized, then devices correspond to device_types
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
