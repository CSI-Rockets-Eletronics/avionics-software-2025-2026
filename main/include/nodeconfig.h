#ifndef NODECONFIG_H_
#define NODECONFIG_H_

#include "avionics.h"
#include "comms.h"

namespace nodeconfig {

extern avionics::Node& this_node;
extern avionics::EspNow esp_now;

}  // namespace nodeconfig

#endif  // NODECONFIG_H_
