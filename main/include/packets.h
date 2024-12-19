#ifndef PACKETS_H_
#define PACKETS_H_

namespace avionics {

#pragma pack(push, 1)

// size: 23 bytes
struct GpsPacket {
    uint64_t ts;              // 8 bytes
    uint8_t fix;              // 1 byte
    uint8_t fixquality;       // 1 byte
    uint8_t satellites;       // 1 byte
    int32_t latitude_fixed;   // 4 bytes
    int32_t longitude_fixed;  // 4 bytes
    float altitude;           // 4 bytes
};

// size: 20 bytes
struct ImuPacket {
    uint64_t ts;  // 8 bytes
    int16_t ax;   // 2 bytes
    int16_t ay;   // 2 bytes
    int16_t az;   // 2 bytes
    int16_t gx;   // 2 bytes
    int16_t gy;   // 2 bytes
    int16_t gz;   // 2 bytes
};

// size: 16 bytes
struct DhtPacket {
    uint64_t ts;        // 8 bytes
    float temperature;  // 4 bytes
    float humidity;     // 4 bytes
};

#pragma pack(pop)

}  // namespace avionics

#endif  // PACKETS_H_
