#include <Adafruit_MAX31855.h>
#include <Adafruit_MAX31856.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

class DevFsThermocouples : public Device {
   private:
    static const int kClkPin = 37;
    static const int kMosiPin = 40;  // TODO
    static const int kMisoPin = 39;

    static const int kCs3Pin = 14;  // TODO
    static const int kCs4Pin = 13;

    Adafruit_MAX31856 tc3{kCs3Pin, kMosiPin, kMisoPin, kClkPin};
    Adafruit_MAX31855 tc4{kClkPin, kCs4Pin, kMisoPin};

    utils::FrequencyLogger thermocouples_freq_logger{"Thermocouples"};

   public:
    void Setup() override {
        if (!tc3.begin()) Die("tc3 init failed");
        tc3.setThermocoupleType(MAX31856_TCTYPE_K);

        if (!tc4.begin()) Die("tc4 init failed");
        // if (!tc5.begin()) Die("tc5 init failed");
    }

    void Loop() override {
        double tc3_celsius = tc3.readThermocoupleTemperature();
        double tc4_celsius = tc4.readCelsius();
        // double tc5_celsius = tc5.readCelsius();

        Handle31856Fault("tc3", tc3, tc3_celsius);
        Handle31855Fault("tc4", tc4, tc4_celsius);
        // Handle31855Fault("tc5", tc5, tc5_celsius);

        FsThermocouplesPacket thermo_packet{
            .ts = micros(),
            // if there are faults, values will be NaN
            .lox_celsius = CoalesceNaN(tc4_celsius),
            // .gn2_celsius = CoalesceNaN(tc5_celsius),
            .gn2_celsius = CoalesceNaN(tc3_celsius),
            ._dummy = 0,
        };
        Send(DeviceType::DevFsInjectorTransducers, thermo_packet);

        thermocouples_freq_logger.Tick();

        // PrintCelsius("tc3", tc3_celsius);
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

    void Handle31855Fault(const char *label, Adafruit_MAX31855 &tc,
                          double &celsius) {
        if (!isnan(celsius)) return;

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

    void Handle31856Fault(const char *label, Adafruit_MAX31856 &tc,
                          double &celsius) {
        uint8_t fault = tc.readFault();
        if (!fault) return;

        celsius = NAN;

        Serial.print(label);
        Serial.println(": Thermocouple fault(s) detected!");

        if (fault & MAX31856_FAULT_CJRANGE)
            Serial.println("Cold Junction Range Fault");
        if (fault & MAX31856_FAULT_TCRANGE)
            Serial.println("Thermocouple Range Fault");
        if (fault & MAX31856_FAULT_CJHIGH)
            Serial.println("Cold Junction High Fault");
        if (fault & MAX31856_FAULT_CJLOW)
            Serial.println("Cold Junction Low Fault");
        if (fault & MAX31856_FAULT_TCHIGH)
            Serial.println("Thermocouple High Fault");
        if (fault & MAX31856_FAULT_TCLOW)
            Serial.println("Thermocouple Low Fault");
        if (fault & MAX31856_FAULT_OVUV)
            Serial.println("Over/Under Voltage Fault");
        if (fault & MAX31856_FAULT_OPEN)
            Serial.println("Thermocouple Open Fault");
    } 

    // JSON doesn't allow NaN; also convert double to float
    float CoalesceNaN(double value) {
        return isnan(value) ? 0.0f : (float)value;
    }
};

REGISTER_AVIONICS_DEVICE(DevFsThermocouples);
