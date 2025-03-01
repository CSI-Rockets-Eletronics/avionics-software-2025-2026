#ifndef PACKETS_H_
#define PACKETS_H_

namespace avionics {

#pragma pack(push, 1)

// size: 17 bytes
struct FsLoxGn2TransducersPacket {
    float lox_upper;       // 4 bytes
    float lox_lower;       // 4 bytes
    float gn2_manifold_1;  // 4 bytes
    float gn2_manifold_2;  // 4 bytes
    uint8_t _dummy;        // 1 byte
};

// size: 8 bytes
struct FsInjectorTransducersPacket {
    float injector_manifold_1;  // 4 bytes
    float injector_manifold_2;  // 4 bytes
};

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

// size: 18 bytes
struct RadioPacket {
    // last byte of ts, to detect fresh data
    uint8_t gps_ts_tail;          // 1 byte
    uint8_t gps_fix;              // 1 byte
    uint8_t gps_fixquality;       // 1 byte
    uint8_t gps_satellites;       // 1 byte
    int32_t gps_latitude_fixed;   // 4 bytes
    int32_t gps_longitude_fixed;  // 4 bytes
    float gps_altitude;           // 4 bytes
    int16_t imu_az;               // 2 bytes
};

#pragma pack(pop)

}  // namespace avionics

#endif  // PACKETS_H_
