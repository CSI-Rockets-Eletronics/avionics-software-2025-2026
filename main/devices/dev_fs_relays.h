#include "avionics.h"

using namespace avionics;

class DevFsRelays : public Device {
   private:
    const int kRelayPins[12] = {5, 1, 6, 7, 15, 16, 17, 18, 8, 2, 12, 9};
    const int kDelayMs = 1000;

   public:
    void Setup() override {
        for (int i = 0; i < 12; i++) {
            pinMode(kRelayPins[i], OUTPUT);
        }
    }

    void Loop() override {
        for (int i = 0; i < 12; i++) {
            digitalWrite(kRelayPins[i], HIGH);
        }
        delay(kDelayMs);

        for (int i = 0; i < 12; i++) {
            digitalWrite(kRelayPins[i], LOW);
        }
        delay(kDelayMs);
    }
};

REGISTER_AVIONICS_DEVICE(DevFsRelays);
