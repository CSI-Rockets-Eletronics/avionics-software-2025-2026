#include <RH_RF95.h>

#include "avionics.h"

static const int kInterruptPin = 99;  // TODO

static const int kFrequency = 433;
static const int kTxPower = 23;

class DevRadio : public avionics::Device {
   public:
    void Setup() override {
        if (!rf95.init()) {
            return SetupDie("RF95 init failed");
        }
        if (!rf95.setFrequency(kFrequency)) {
            return SetupDie("RF95 setFrequency failed");
        }
        rf95.setTxPower(kTxPower, false);
    }

    void Loop() override {
        // TODO
    }

   private:
    RH_RF95 rf95{SS, kInterruptPin};  // uses default SPI pins
};

REGISTER_AVIONICS_DEVICE(DevRadio);
