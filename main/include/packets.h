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
    STATE_FIRE_MANUAL_PRESS_PILOT = 21,
    STATE_FIRE_MANUAL_DOME_PILOT_CLOSE = 22,
    STATE_FIRE_MANUAL_IGNITER = 23,
    STATE_FIRE_MANUAL_RUN = 24,
    EREG_CLOSED = 30,
    EREG_STAGE_1 = 31,
    EREG_STAGE_2 = 32,
    RECALIBRATE_TRANSDUCERS = 100,
    RESTART = 110,
};

// size: 10 bytes
struct FsCommandPacket {
    FsCommand command;  // 1 byte

    // the solenoid state fields below are only used if command is CUSTOM
    bool gn2_drain;          // 1 byte
    bool gn2_fill;           // 1 byte
    bool depress;            // 1 byte
    bool press_pilot;        // 1 byte
    bool run;                // 1 byte
    bool lox_fill;           // 1 byte
    bool lox_disconnect;     // 1 byte
    bool igniter;            // 1 byte
    bool ereg_power;         // 1 byte
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
    FROM_FS_COMMAND(FIRE_MANUAL_PRESS_PILOT),
    FROM_FS_COMMAND(FIRE_MANUAL_DOME_PILOT_CLOSE),
    FROM_FS_COMMAND(FIRE_MANUAL_IGNITER),
    FROM_FS_COMMAND(FIRE_MANUAL_RUN),
};

// size: 14 bytes
struct FsStatePacket {
    uint32_t ms_since_boot;  // 4 bytes
    FsState state;           // 1 byte
    bool gn2_drain;          // 1 byte
    bool gn2_fill;           // 1 byte
    bool depress;         // 1 byte
    bool press_pilot;    // 1 byte
    bool run;                // 1 byte
    bool lox_fill;           // 1 byte
    bool lox_disconnect;     // 1 byte
    bool igniter;            // 1 byte
    bool ereg_power;         // 1 byte
};

// size: 35 bytes
struct FsLoxGn2TransducersPacket {
    uint64_t ts;           // 8 bytes
    float oxtank_1;        // 4 bytes
    float oxtank_2;        // 4 bytes
    float copv_1;          // 4 bytes
    float copv_2;          // 4 bytes
    float pilot_pres;      // 4 bytes
    float qd_pres;         // 4 bytes
    bool ereg_closed;      // 1 byte
    bool ereg_stage_1;     // 1 byte
    bool ereg_stage_2;     // 1 byte
};

// size: 20 bytes
struct FsInjectorTransducersPacket {
    uint64_t ts;                // 8 bytes
    float injector_1;           // 4 bytes
    float injector_2;           // 4 bytes
    float upper_cc;             // 4 bytes
};

// size: 25 bytes
struct FsThermocouplesPacket {
    uint64_t ts;                   // 8 bytes
    float gn2_internal_celsius;    // 4 bytes
    float gn2_external_celsius;    // 4 bytes
    float lox_upper_celsius;       // 4 bytes
    float lox_lower_celsius;       // 4 bytes
    uint8_t dummy;                 // 1 byte (for unique packet size)
};

// size: 17 bytes
struct CapFillPacket {
    uint64_t ts;            // 8 bytes
    float cap_fill_base;    // 4 bytes
    float cap_fill_actual;  // 4 bytes
    int8_t board_temp;      // 1 byte
};

// size: 26 bytes
struct RelayCurrentMonitorPacket {
    uint64_t ts;                // 8 bytes
    int16_t gn2_drain_ma;       // 2 bytes
    int16_t gn2_fill_ma;        // 2 bytes
    int16_t depress_ma;         // 2 bytes
    int16_t press_pilot_ma;     // 2 bytes
    int16_t run_ma;             // 2 bytes
    int16_t lox_fill_ma;        // 2 bytes
    int16_t lox_disconnect_ma;  // 2 bytes
    int16_t igniter_ma;         // 2 bytes
    int16_t ereg_power_ma;      // 2 bytes
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

// size: 21 bytes
struct ImuPacket {
    uint64_t ts;  // 8 bytes
    int16_t ax;   // 2 bytes
    int16_t ay;   // 2 bytes
    int16_t az;   // 2 bytes
    int16_t gx;   // 2 bytes
    int16_t gy;   // 2 bytes
    int16_t gz;   // 2 bytes
    uint8_t dummy;                 // 1 byte (for unique packet size)
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
