#ifndef NODE_CONFIG_H_
#define NODE_CONFIG_H_

#include "avionics.h"

namespace avionics {

static Node radio_node{MacAddress{"00:00:00:00:00:00"}, {DeviceType::DevRadio}};

static Node pos_tracking_node{MacAddress{"34:85:18:a5:ef:24"},
                              {DeviceType::DevDht, DeviceType::DevGps,
                               DeviceType::DevImu, DeviceType::DevPiSerial}};

// static Node pi_serial_node{MacAddress{"34:85:18:a5:88:50"},
//                            {DeviceType::DevPiSerial}};

}  // namespace avionics

#endif  // NODE_CONFIG_H_
