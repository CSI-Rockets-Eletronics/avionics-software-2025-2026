#ifndef COMMS_H_
#define COMMS_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace avionics {

class MacAddress {
   public:
    MacAddress();
    MacAddress(std::string str_address);
    std::string ToString() const;
    void CopyInto(uint8_t* dest) const;
    uint8_t* Data();
    const uint8_t* ReadData() const;

    bool operator==(const MacAddress& other) const;

   private:
    std::array<uint8_t, 6> bytes_;
};

void EspNowSetup();

void EspNowSend(const MacAddress& to_address, const uint8_t* bytes, size_t len);

}  // namespace avionics

#endif  // COMMS_H_
