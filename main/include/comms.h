#ifndef COMMS_H_
#define COMMS_H_

#include <cstddef>
#include <cstdint>

#include "avionics.h"

namespace avionics {

// only ever construct one of these
class EspNow {
   public:
    EspNow(const std::vector<MacAddress>& mac_addresses);

    // no-op if there is already a message in flight
    template <typename Msg>
    void Send(const MacAddress& to_address, const Msg& msg) {
        SendBytes(to_address, reinterpret_cast<const uint8_t*>(&msg), sizeof(Msg));
    }

   private:
    void SendBytes(const MacAddress& to_address, const uint8_t* bytes, size_t len);
};

}  // namespace avionics

#endif  // COMMS_H_
