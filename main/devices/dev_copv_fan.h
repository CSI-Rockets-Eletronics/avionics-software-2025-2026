#include <Arduino.h>
#include <ESP32Servo.h>

#include "avionics.h"

using namespace avionics;

// ——— Pin Configuration ———
static const int kServoPin = 17;  // GPIO pin connected to the servo signal line

// ——— Servo/ESC Pulse Widths ———
static const uint32_t kPulseMax = 2000;  // µs (on position / maximum throttle)
static const uint32_t kPulseMedium =
    1500;                                // µs (medium position / mid throttle)
static const uint32_t kPulseMin = 1000;  // µs (off position / minimum throttle)

class DevCopvFan : public Device {
   public:
    void Setup() override {
        Serial.begin(115200);

        servo.attach(kServoPin);
        phase = 0;
        phaseStartMillis = millis();
        servo.writeMicroseconds(kPulseMax);  // Start at high power
        Serial.println("Phase 0: High power (5s)");
    }

    void Loop() override {
        unsigned long now = millis();
        if (phase == 0 && now - phaseStartMillis >= 5000) {
            servo.writeMicroseconds(kPulseMin);  // Switch to low power
            phase = 1;
            phaseStartMillis = now;
            Serial.println("Phase 1: Low power (5s)");
        } else if (phase == 1 && now - phaseStartMillis >= 5000) {
            servo.writeMicroseconds(kPulseMedium);  // Switch to medium power
            phase = 2;
            Serial.println("Phase 2: Medium power (indefinite)");
        }
        // phase 2: stay at medium power indefinitely
    }

   private:
    Servo servo;
    int phase = 0;
    unsigned long phaseStartMillis = 0;
};

REGISTER_AVIONICS_DEVICE(DevCopvFan);
