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
        servo.writeMicroseconds(kPulseMax);  // Default to max
        Serial.println("Ready for L/M/H commands");
    }

    void Loop() override {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'L') {
                servo.writeMicroseconds(kPulseMin);
                Serial.println("Manual: L -> Min throttle");
            } else if (c == 'M') {
                servo.writeMicroseconds(kPulseMedium);
                Serial.println("Manual: M -> Medium throttle");
            } else if (c == 'H') {
                servo.writeMicroseconds(kPulseMax);
                Serial.println("Manual: H -> Max throttle");
            }
        }
    }

   private:
    Servo servo;
};

REGISTER_AVIONICS_DEVICE(DevCopvFan);
