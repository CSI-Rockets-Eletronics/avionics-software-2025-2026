#include <SparkFun_MCP9600.h>
#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsThermocouples : public Device {
   private:
    // I2C addresses for the 4 MCP9600 thermocouples on I2C bus 0
    // will need to change I2C addresses once fs board is assembled
    static const uint8_t kGn2InternalAddress = 0x60;
    static const uint8_t kGn2ExternalAddress = 0x61;
    static const uint8_t kLoxUpperAddress = 0x62;
    static const uint8_t kLoxLowerAddress = 0x63;

    I2CWire i2c0{0, 42, 37};  //I2C bus "5"

    MCP9600 gn2_internal;
    MCP9600 gn2_external;
    MCP9600 lox_upper;
    MCP9600 lox_lower;

    utils::FrequencyLogger thermocouples_freq_logger{"Thermocouples"};

   public:
    void Setup() override {
        // Initialize all 4 MCP9600 thermocouples on I2C bus 0
        if (!gn2_internal.begin(kGn2InternalAddress, i2c0.wire))
            Die("gn2_internal init failed");
        gn2_internal.setThermocoupleType(TYPE_E);

        if (!gn2_external.begin(kGn2ExternalAddress, i2c0.wire))
            Die("gn2_external init failed");
        gn2_external.setThermocoupleType(TYPE_E);

        if (!lox_upper.begin(kLoxUpperAddress, i2c0.wire))
            Die("lox_upper init failed");
        lox_upper.setThermocoupleType(TYPE_E);

        if (!lox_lower.begin(kLoxLowerAddress, i2c0.wire))
            Die("lox_lower init failed");
        lox_lower.setThermocoupleType(TYPE_E);
    }

    void Loop() override {
        // Read temperatures from all 4 MCP9600 thermocouples
        float gn2_internal_celsius = gn2_internal.getThermocoupleTemp();
        float gn2_external_celsius = gn2_external.getThermocoupleTemp();
        float lox_upper_celsius = lox_upper.getThermocoupleTemp();
        float lox_lower_celsius = lox_lower.getThermocoupleTemp();

        // Check for sensor errors
        HandleMCP9600Fault("gn2_internal", gn2_internal, gn2_internal_celsius);
        HandleMCP9600Fault("gn2_external", gn2_external, gn2_external_celsius);
        HandleMCP9600Fault("lox_upper", lox_upper, lox_upper_celsius);
        HandleMCP9600Fault("lox_lower", lox_lower, lox_lower_celsius);

        FsThermocouplesPacket thermo_packet{
            .ts = micros(),
            .gn2_internal_celsius = CoalesceNaN(gn2_internal_celsius),
            .gn2_external_celsius = CoalesceNaN(gn2_external_celsius),
            .lox_upper_celsius = CoalesceNaN(lox_upper_celsius),
            .lox_lower_celsius = CoalesceNaN(lox_lower_celsius),
            .dummy = 0,
        };
        Send(DeviceType::DevFsInjectorTransducers, thermo_packet);

        thermocouples_freq_logger.Tick();

        // PrintCelsius("gn2_internal", gn2_internal_celsius);
        // PrintCelsius("gn2_external", gn2_external_celsius);
        // PrintCelsius("lox_upper", lox_upper_celsius);
        // PrintCelsius("lox_lower", lox_lower_celsius);

        delay(100);
    }

    void PrintCelsius(const char* label, float celsius) {
        Serial.print(label);
        Serial.print(": ");
        Serial.print(celsius);
        Serial.println(" *C");
    }

    void HandleMCP9600Fault(const char* label, MCP9600& tc, float& celsius) {
        // Check if input range is exceeded
        if (tc.isInputRangeExceeded()) {
            celsius = NAN;
            Serial.print(label);
            Serial.println(": Input range exceeded!");
        }

        // Check if sensor is connected
        if (!tc.isConnected()) {
            celsius = NAN;
            Serial.print(label);
            Serial.println(": Sensor not connected!");
        }
    }

    // JSON doesn't allow NaN
    float CoalesceNaN(float value) { return isnan(value) ? 0.0f : value; }
};

REGISTER_AVIONICS_DEVICE(DevFsThermocouples);
