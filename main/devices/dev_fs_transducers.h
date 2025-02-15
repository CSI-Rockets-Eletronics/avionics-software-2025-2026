#include "avionics.h"

using namespace avionics;

class DevFsTransducers : public Device {
   public:
    void Setup() override {}

    void Loop() override {}

   private:
};

REGISTER_AVIONICS_DEVICE(DevFsTransducers);
