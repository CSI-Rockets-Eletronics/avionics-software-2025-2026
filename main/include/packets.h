#ifndef PACKETS_H_
#define PACKETS_H_

namespace avionics {

struct PiSerialPacket {
    char msg[64];
};

}  // namespace avionics

#endif  // PACKETS_H_
