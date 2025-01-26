#include <DHT.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

static const int kDhtPin = 6;

class DevDht : public Device {
   public:
    void Setup() override { dht.begin(); }

    void Loop() override {
        delay(1000);  // DHT11 needs at least 1s between reads

        // force read once for both temperature and humidity
        bool success = dht.read(true);

        if (!success) {
            Serial.println("Failed to read from DHT sensor!");
            return;
        }

        DhtPacket dht_packet{
            .ts = micros(),
            // these will use the values from the last read()
            .temperature = dht.readTemperature(),
            .humidity = dht.readHumidity(),
        };
        Send(DeviceType::DevPiSerial, dht_packet);
    }

   private:
    DHT dht{kDhtPin, DHT11};
};

REGISTER_AVIONICS_DEVICE(DevDht);
