#include "avionics.h"

using namespace avionics;

static Node radio_node{MacAddress{"00:00:00:00:00:00"}, {DeviceType::DevRadio}};

static Node pos_tracking_node{MacAddress{"00:00:00:00:00:00"},
                              {DeviceType::DevDhtImu, DeviceType::DevGps}};
