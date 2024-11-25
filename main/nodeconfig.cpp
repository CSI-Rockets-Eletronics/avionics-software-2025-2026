#include "nodeconfig.h"

namespace nodeconfig {

using namespace avionics;

static Node radio_node{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, {DeviceType::DevRadio}};
static Node pos_tracking_node{{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, {DeviceType::DevDhtImu, DeviceType::DevGps}};

avionics::Node& this_node = radio_node;

}  // namespace nodeconfig
