#include <Arduino.h>

#include "DHT.h"
#include "device.h"

static const uint8_t kDhtPin = 99;  // TODO

class DevDhtImu : public device::Device {
   public:
    void Init() override {
        // TODO
    }

    void Loop() override {
        // TODO
    }

   private:
    DHT dht{kDhtPin, DHT11};
};

DevDhtImu devDhtImu;
