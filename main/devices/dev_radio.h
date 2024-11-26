#include <RH_RF95.h>

#include "avionics.h"
#include "packets.h"

static const int kInterruptPin = 99;  // TODO

static const int kFrequency = 433;
static const int kTxPower = 23;

class DevRadio : public avionics::Device {
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
        // TODO
        avionics::PiSerialPacket packet{.msg = "Ping from Radio!"};
        Send(avionics::DeviceType::DevPiSerial, packet);
        // Serial.println("Sent message");
    }

   private:
    RH_RF95 rf95{SS, kInterruptPin};  // uses default SPI pins
};

REGISTER_AVIONICS_DEVICE(DevRadio);
