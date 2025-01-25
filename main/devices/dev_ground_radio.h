#include <RHSoftwareSPI.h>
#include <RH_RF95.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

class DevGroundRadio : public Device {
   private:
    static const int kMisoPin = 13;
    static const int kMosiPin = 11;
    static const int kSckPin = 12;
    static const int kSsPin = 10;
    static const int kInterruptPin = 2;

    static const int kFrequency = 433;
    static const int kTxPower = 20;  // max power

    RHSoftwareSPI spi;  // for custom pins
    RH_RF95 rf95{SS, kInterruptPin, spi};

   public:
    void Setup() override {
        spi.setPins(kMisoPin, kMosiPin, kSckPin);

        if (!rf95.init()) {
            return Die("RF95 init failed");
        }
        if (!rf95.setFrequency(kFrequency)) {
            return Die("RF95 setFrequency failed");
        }
        rf95.setTxPower(kTxPower, false);
    }

    void Loop() override {
        rf95.waitAvailable();

        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (!rf95.recv(buf, &len)) {
            Serial.println("RF95 recv failed");
            return;
        }

        if (len != sizeof(RadioPacket)) {
            Serial.println("RF95 recv len mismatch");
            return;
        }

        RadioPacket* radio_packet = (RadioPacket*)buf;

        Serial.println("Received radio packet");
        Serial.print("gps_ts_tail: ");
        Serial.print(radio_packet->gps_ts_tail);
        Serial.print("\taz: ");
        Serial.println(radio_packet->imu_az);
    }
};

REGISTER_AVIONICS_DEVICE(DevGroundRadio);
