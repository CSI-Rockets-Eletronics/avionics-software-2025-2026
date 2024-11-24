#include <Arduino.h>
#include <RH_RF95.h>

#include "device.h"

static const uint8_t kInterrupt = 99;  // TODO
static const uint32_t kFrequency = 433;
static const int8_t kTxPower = 23;

class DevRadio : public device::Device {
   public:
    void Init() override {
        if (!rf95.init()) {
            return InitDie("RF95 init failed");
        }
        if (!rf95.setFrequency(kFrequency)) {
            return InitDie("RF95 setFrequency failed");
        }
        rf95.setTxPower(kTxPower, false);
    }

    void Loop() override {
        // TODO
    }

   private:
    RH_RF95 rf95{SS, kInterrupt};
};

DevRadio devRadio;