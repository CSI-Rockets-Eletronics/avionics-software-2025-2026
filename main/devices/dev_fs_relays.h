#include "avionics.h"

using namespace avionics;

class DevFsRelays : public Device {
   public:
    void Setup() override {
        // set pin 5 as output
        pinMode(5, OUTPUT);
    }

    void Loop() override {
        Serial.println("Test");

        // toggle pin 5 on/off for 1s
        digitalWrite(5, HIGH);
        delay(1000);
        digitalWrite(5, LOW);
        delay(1000);
    }
   private:
};

REGISTER_AVIONICS_DEVICE(DevFsRelays);
