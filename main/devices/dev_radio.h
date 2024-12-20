#include <RH_RF95.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

static const int kInterruptPin = 99;  // TODO

static const int kFrequency = 433;
static const int kTxPower = 20;  // max power

class DevRadio : public Device {
   public:
    void Setup() override {
        if (!rf95.init()) {
            return Die("RF95 init failed");
        }
        if (!rf95.setFrequency(kFrequency)) {
            return Die("RF95 setFrequency failed");
        }
        rf95.setTxPower(kTxPower, false);
    }

    void Loop() override {
        RadioPacket radio_packet;

        if (Receive(&radio_packet) == 0) {
            Serial.print("Received radio packet (gps_ts_tail = ");
            Serial.print(radio_packet.gps_ts_tail);
            Serial.println(")");

            rf95.send((uint8_t*)&radio_packet, sizeof(radio_packet));
            rf95.waitPacketSent();
        } else {
            // yield and wait for more data
            delay(10);
        }
    }

   private:
    RH_RF95 rf95{SS, kInterruptPin};  // uses default SPI pins
};

REGISTER_AVIONICS_DEVICE(DevRadio);
