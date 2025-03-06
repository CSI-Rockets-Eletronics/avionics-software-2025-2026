#include <Adafruit_MAX31855.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

class DevFsThermocouples : public Device {
   private:
    static const int kClkPin = 40;
    static const int kCsPin = 39;
    static const int kDoPin = 37;  // aka MISO

    Adafruit_MAX31855 thermocouple{kClkPin, kCsPin, kDoPin};

   public:
    void Setup() override {
        if (!thermocouple.begin()) {
            Die("MAX31855 init failed");
        }
    }

    void Loop() override {
        double celsius = thermocouple.readCelsius();

        if (isnan(celsius)) {
            HandleFault();
        } else {
            FsThermocouplesPacket thermo_packet{
                .ts = micros(),
                .lox_celsius = (float)celsius,
                .gn2_celsius = 0,  // TODO
                ._dummy = 0,
            };
            Send(DeviceType::DevFsInjectorTransducers, thermo_packet);
        }

        delay(1000);
    }

    void HandleFault() {
        Serial.println("Thermocouple fault(s) detected!");
        uint8_t error = thermocouple.readError();
        if (error & MAX31855_FAULT_OPEN)
            Serial.println("FAULT: Thermocouple is open - no connections.");
        if (error & MAX31855_FAULT_SHORT_GND)
            Serial.println("FAULT: Thermocouple is short-circuited to GND.");
        if (error & MAX31855_FAULT_SHORT_VCC)
            Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
    }
};

REGISTER_AVIONICS_DEVICE(DevFsThermocouples);
