#include <Adafruit_MAX31855.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

class DevFsThermocouples : public Device {
   private:
    static const int kClkPin = 37;
    static const int kMisoPin = 39;

    static const int kCs4Pin = 13;
    static const int kCs5Pin = 12;

    Adafruit_MAX31855 tc4{kClkPin, kCs4Pin, kMisoPin};
    Adafruit_MAX31855 tc5{kClkPin, kCs5Pin, kMisoPin};

    utils::FrequencyLogger thermocouples_freq_logger{"Thermocouples"};

   public:
    void Setup() override {
        if (!tc4.begin()) Die("tc4 MAX31855 init failed");
        if (!tc5.begin()) Die("tc5 MAX31855 init failed");
    }

    void Loop() override {
        double tc4_celsius = tc4.readCelsius();
        double tc5_celsius = tc5.readCelsius();

        if (isnan(tc4_celsius)) HandleFault("tc4", tc4);
        if (isnan(tc5_celsius)) HandleFault("tc5", tc5);

        FsThermocouplesPacket thermo_packet{
            .ts = micros(),
            // if there are faults, values will be NaN
            .lox_celsius = CoalesceNaN(tc4_celsius),
            .gn2_celsius = CoalesceNaN(tc5_celsius),
            ._dummy = 0,
        };
        Send(DeviceType::DevFsInjectorTransducers, thermo_packet);

        thermocouples_freq_logger.Tick();

        // PrintCelsius("tc4", tc4_celsius);
        // PrintCelsius("tc5", tc5_celsius);

        delay(100);
    }

    void PrintCelsius(const char *label, double celsius) {
        Serial.print(label);
        Serial.print(": ");
        Serial.print(celsius);
        Serial.println(" *C");
    }

    void HandleFault(const char *label, Adafruit_MAX31855 &tc) {
        Serial.print(label);
        Serial.println(": Thermocouple fault(s) detected!");

        uint8_t error = tc.readError();

        if (error & MAX31855_FAULT_OPEN)
            Serial.println("FAULT: Thermocouple is open - no connections.");
        if (error & MAX31855_FAULT_SHORT_GND)
            Serial.println("FAULT: Thermocouple is short-circuited to GND.");
        if (error & MAX31855_FAULT_SHORT_VCC)
            Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
    }

    // JSON doesn't allow NaN; also convert double to float
    float CoalesceNaN(double value) {
        return isnan(value) ? 0.0f : (float)value;
    }
};

REGISTER_AVIONICS_DEVICE(DevFsThermocouples);
