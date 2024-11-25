#include "nodeconfig.h"

namespace nodeconfig {

using namespace avionics;

static Node radio_node{"node1", {DeviceType::DevRadio}};
static Node pos_tracking_node{"node1", {DeviceType::DevDhtImu, DeviceType::DevGps}};

avionics::Node& this_node = radio_node;

}  // namespace nodeconfig
