#ifndef NODE_CONFIG_H_
#define NODE_CONFIG_H_

#include "avionics.h"

namespace avionics {

static Node fs_scientific1{ //i2c3/4
    "FsScientific1",
    MacAddress{"50:78:7d:35:20:48"},
    {
        DeviceType::DevFsLoxGn2Transducers,
        DeviceType::DevFsPiPacketBroadcaster,
        DeviceType::DevEregControl,
    },
};


static Node fs_scientific2{ //i2c1/2
    "FsScientific2",
    MacAddress{"50:78:7d:35:20:58"},
    {
        DeviceType::DevFsInjectorTransducers,
    },
};


static Node fs_relays{ //i2c5/6
    "FsRelays",
    MacAddress{"50:78:7d:35:1f:8c"},
    {
        DeviceType::DevFsRelays,
        DeviceType::DevFsThermocouples,  // Commented out - no power yet
        DeviceType::DevRelayImon,
    },
};

static Node cap_fill{
    "CapFill",
    MacAddress{"34:85:18:a5:87:f0"},
    {
        // DeviceType::DevCapFill,  // Commented out - no power yet
    },
};

static Node rocket_radio{
    "RocketRadio",
    MacAddress{"34:85:18:a5:88:50"},
    {DeviceType::DevRocketRadio},
};

static Node pos_tracking{
    "PosTracking",
    MacAddress{"34:85:18:a5:ef:24"},
    {
        //   DeviceType::DevDht,
        DeviceType::DevGps,
        DeviceType::DevImu,
        DeviceType::DevPiSerial,
    },
};

static Node ground_radio{
    "GroundRadio",
    MacAddress{"68:b6:b3:3f:07:5c"},
    {DeviceType::DevGroundRadio},
    false,  // don't use ESP-NOW (leaves WiFi available)
};

}  // namespace avionics

#endif  // NODE_CONFIG_H_
