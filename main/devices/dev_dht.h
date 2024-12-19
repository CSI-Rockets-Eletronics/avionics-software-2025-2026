#include <DHT.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

static const int kDhtPin = 99;  // TODO

class DevDht : public Device {
   public:
    void Setup() override {
        // dht.begin();
    }

    void Loop() override {
        DhtPacket dht_packet{
            .ts = micros(),
            .temperature = 0.0,
            .humidity = 0.0,
        };
        Send(DeviceType::DevPiSerial, dht_packet);

        delay(1000);
    }

   private:
    DHT dht{kDhtPin, DHT11};
};

REGISTER_AVIONICS_DEVICE(DevDht);
