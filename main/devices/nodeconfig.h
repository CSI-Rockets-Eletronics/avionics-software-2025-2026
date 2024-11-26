#ifndef NODE_CONFIG_H_
#define NODE_CONFIG_H_

#include "avionics.h"

namespace avionics {

static Node radio_node{MacAddress{"d8:3b:da:a0:d1:6c"}, {DeviceType::DevRadio}};

static Node pos_tracking_node{MacAddress{"00:00:00:00:00:01"},
                              {DeviceType::DevDhtImu, DeviceType::DevGps}};

static Node pi_serial_node{MacAddress{"34:85:18:A4:55:24"},
                           {DeviceType::DevPiSerial}};

}  // namespace avionics

#endif  // NODE_CONFIG_H_
