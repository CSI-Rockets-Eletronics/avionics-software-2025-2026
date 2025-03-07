#ifndef PACKETS_H_
#define PACKETS_H_

namespace avionics {

#pragma pack(push, 1)

// NOTE: packet sizes must be unique within ground packets and avionics packets

// ===== GROUND PACKETS =====

enum class FsCommand : uint8_t {
    STATE_CUSTOM = 0,
    STATE_ABORT = 1,
    STATE_STANDBY = 2,
    STATE_GN2_STANDBY = 3,
    STATE_GN2_FILL = 10,
    STATE_GN2_PULSE_FILL_A = 11,
    STATE_GN2_PULSE_FILL_B = 12,
    STATE_GN2_PULSE_FILL_C = 13,
    STATE_FIRE = 20,
    STATE_FIRE_MANUAL_DOME_PILOT_OPEN = 21,
    STATE_FIRE_MANUAL_DOME_PILOT_CLOSE = 22,
    STATE_FIRE_MANUAL_IGNITER = 23,
    STATE_FIRE_MANUAL_RUN = 24,
    RECALIBRATE_TRANSDUCERS = 100,
    RESTART = 110,
};

// size: 9 bytes
struct FsCommandPacket {
    FsCommand command;  // 1 byte

    // the solenoid state fields below are only used if command is CUSTOM
    bool gn2_abort;          // 1 byte
    bool gn2_fill;           // 1 byte
    bool pilot_vent;         // 1 byte
    bool dome_pilot_open;    // 1 byte
    bool run;                // 1 byte
    bool water_suppression;  // 1 byte
    bool igniter;            // 1 byte

    uint8_t _dummy;  // 1 byte
};

#define FROM_FS_COMMAND(COMMAND) COMMAND = (uint8_t)FsCommand::STATE_##COMMAND

// a subset of FsCommand values are state values
enum class FsState : uint8_t {
    // expands to:
    // CUSTOM = (uint8_t)FsCommand::STATE_CUSTOM,
    FROM_FS_COMMAND(CUSTOM),
    FROM_FS_COMMAND(ABORT),
    FROM_FS_COMMAND(STANDBY),
    FROM_FS_COMMAND(GN2_STANDBY),
    FROM_FS_COMMAND(GN2_FILL),
    FROM_FS_COMMAND(GN2_PULSE_FILL_A),
    FROM_FS_COMMAND(GN2_PULSE_FILL_B),
    FROM_FS_COMMAND(GN2_PULSE_FILL_C),
    FROM_FS_COMMAND(FIRE),
    FROM_FS_COMMAND(FIRE_MANUAL_DOME_PILOT_OPEN),
    FROM_FS_COMMAND(FIRE_MANUAL_DOME_PILOT_CLOSE),
    FROM_FS_COMMAND(FIRE_MANUAL_IGNITER),
    FROM_FS_COMMAND(FIRE_MANUAL_RUN),
};

// size: 8 bytes
struct FsStatePacket {
    FsState state;           // 1 byte
    bool gn2_abort;          // 1 byte
    bool gn2_fill;           // 1 byte
    bool pilot_vent;         // 1 byte
    bool dome_pilot_open;    // 1 byte
    bool run;                // 1 byte
    bool water_suppression;  // 1 byte
    bool igniter;            // 1 byte
};

// size: 24 bytes
struct FsLoxGn2TransducersPacket {
    uint64_t ts;           // 8 bytes
    float lox_upper;       // 4 bytes
    float lox_lower;       // 4 bytes
    float gn2_manifold_1;  // 4 bytes
    float gn2_manifold_2;  // 4 bytes
};

// size: 16 bytes
struct FsInjectorTransducersPacket {
    uint64_t ts;                // 8 bytes
    float injector_manifold_1;  // 4 bytes
    float injector_manifold_2;  // 4 bytes
};

// size: 17 bytes
struct FsThermocouplesPacket {
    uint64_t ts;        // 8 bytes
    float lox_celsius;  // 4 bytes
    float gn2_celsius;  // 4 bytes
    uint8_t _dummy;     // 1 byte
};

// ===== AVIONICS PACKETS =====

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
