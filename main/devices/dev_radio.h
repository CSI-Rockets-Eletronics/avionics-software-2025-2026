#include <RH_RF95.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

static const int kInterruptPin = 99;  // TODO

static const int kFrequency = 433;
static const int kTxPower = 23;

class DevRadio : public Device {
   public:
    void Setup() override {
        // if (!rf95.init()) {
        //     return Die("RF95 init failed");
        // }
        // if (!rf95.setFrequency(kFrequency)) {
        //     return Die("RF95 setFrequency failed");
        // }
        // rf95.setTxPower(kTxPower, false);
    }

    void Loop() override {
        GpsPacket gps_packet;

        if (Receive(&gps_packet) == 0) {
            Serial.print("Received GPS packet (ts = ");
            Serial.print(gps_packet.ts);
            Serial.println(")");
        }
    }

   private:
    RH_RF95 rf95{SS, kInterruptPin};  // uses default SPI pins
};

REGISTER_AVIONICS_DEVICE(DevRadio);
