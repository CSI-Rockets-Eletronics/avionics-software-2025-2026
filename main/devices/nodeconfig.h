#ifndef NODE_CONFIG_H_
#define NODE_CONFIG_H_

#include "avionics.h"

namespace avionics {

static Node rocket_radio_node{"RocketRadio",
                              MacAddress{"34:85:18:a5:88:50"},
                              {DeviceType::DevRocketRadio}};

static Node pos_tracking_node{"PosTracking",
                              MacAddress{"34:85:18:a5:ef:24"},
                              {DeviceType::DevDht, DeviceType::DevGps,
                               DeviceType::DevImu, DeviceType::DevPiSerial}};

static Node ground_radio_node{"GroundRadio",
                              MacAddress{"00:00:00:00:00:00"},  // TODO
                              {DeviceType::DevGroundRadio}};

}  // namespace avionics

#endif  // NODE_CONFIG_H_
