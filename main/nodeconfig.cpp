#include "nodeconfig.h"

namespace avionics {

static Node radio_node{MacAddress{"00:00:00:00:00:00"}, {DeviceType::DevRadio}};
static Node pos_tracking_node{MacAddress{"00:00:00:00:00:00"},
                              {DeviceType::DevDhtImu, DeviceType::DevGps}};

std::vector<MacAddress> all_mac_addresses = {radio_node.mac_address,
                                             pos_tracking_node.mac_address};

Node& this_node = radio_node;

}  // namespace avionics
