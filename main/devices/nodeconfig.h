#ifndef NODE_CONFIG_H_
#define NODE_CONFIG_H_

#include "avionics.h"

namespace avionics {

static Node fs_scientific1{ //i2c3/4
    "FsScientific1",
    MacAddress{"f4:12:fa:5b:42:c8"},
    {
        DeviceType::DevFsLoxGn2Transducers,
        DeviceType::DevFsPiPacketBroadcaster,
    },
};


static Node fs_scientific2{ //i2c1/2
    "FsScientific2",
    MacAddress{"f4:12:fa:83:de:cc"},
    {
        DeviceType::DevFsInjectorTransducers,
    },
};


static Node fs_relays{ //i2c5/6
    "FsRelays",
    MacAddress{"34:85:18:a5:80:4c"},
    {
        DeviceType::DevFsRelays,
        DeviceType::DevFsThermocouples,
        DeviceType::DevRelayImon,
    },
};

static Node cap_fill{
    "CapFill",
    MacAddress{"34:85:18:a5:87:f0"},
    {
        DeviceType::DevCapFill,
    },
}

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
