#ifndef COMMS_H_
#define COMMS_H_

#include <cstddef>
#include <cstdint>

#include "avionics.h"

namespace avionics {

void EspNowSetup();

void EspNowSend(const MacAddress& to_address, const uint8_t* bytes, size_t len);

}  // namespace avionics

#endif  // COMMS_H_
