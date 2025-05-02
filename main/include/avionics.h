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
    DevCopvFan,
    DevDht,
    DevFsInjectorTransducers,
    DevFsLoxGn2Transducers,
    DevFsPiPacketBroadcaster,
    DevFsRelays,
    DevFsThermocouples,
    DevGps,
    DevGroundRadio,
    DevImu,
    DevPiSerial,
    DevRocketRadio,
};

class Device {
   public:
    Device();

    void QueueReceive(const uint8_t* bytes, size_t len);

    virtual void Setup() = 0;
    virtual void Loop() = 0;

   protected:
    void Die(const char* msg);

    template <typename T>
    inline void Send(DeviceType to_device, const T& data) {
        Send(to_device, reinterpret_cast<const uint8_t*>(&data), sizeof(data));
    }

    // writes into the first type that matches the size of the message;
    // returns the index of the argument that was written to, or
    // -1 if no argument was written to
    template <typename T, typename... Others>
    inline int Receive(T* out, Others*... others) {
        // sizes must be different to disambiguate
        static_assert(((sizeof(T) != sizeof(Others)) && ...),
                      "message sizes must be different");

        constexpr bool has_others = sizeof...(Others) > 0;
        if (Receive(reinterpret_cast<uint8_t*>(out), sizeof(T), has_others)) {
            // zero non-matching args to help catch bugs
            (memset(others, 0, sizeof(Others)), ...);
            return 0;
        } else if constexpr (has_others) {
            // zero non-matching args to help catch bugs
            memset(out, 0, sizeof(T));
            auto result = Receive(others...);
            return result >= 0 ? result + 1 : -1;
        } else {
            return -1;
        }
    }

   private:
    QueueHandle_t recv_queue_;

    void Send(DeviceType to_device, const uint8_t* bytes, size_t len);

    // returns true if a message was received, false otherwise
    bool Receive(uint8_t* bytes, size_t len, bool put_back_if_len_mismatch);
};

class Node {
   public:
    const std::string name;
    const MacAddress mac_address;
    const std::vector<DeviceType> device_types;
    const bool use_esp_now;

    Node(std::string name, MacAddress mac_address,
         std::vector<DeviceType> device_types, bool use_esp_now = true);

    Node(const Node&) = delete;

    void Run();

    static std::optional<std::reference_wrapper<Node>> FindNode(
        MacAddress mac_address);

    static std::optional<std::reference_wrapper<Node>> FindNode(
        DeviceType type);

    static std::optional<std::reference_wrapper<Device>> FindDevice(
        DeviceType type);

    static std::vector<MacAddress> AllMacAddresses();

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
        _register_##Dev;

#endif  // AVIONICS_H_
