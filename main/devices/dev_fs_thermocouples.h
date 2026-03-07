#include <SparkFun_MCP9600.h>
#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsThermocouples : public Device {
   private:
    // I2C addresses for the 3 MCP9600 thermocouples on i2c0
    // Devices found on i2c0 (SDA=42, SCL=37): 0x64, 0x65, 0x66
    static const uint8_t kGn2InternalAddress = 0x64;
    static const uint8_t kLoxLowerAddress = 0x65;
    static const uint8_t kLoxUpperAddress = 0x66;

    I2CWire i2c0{0, 42, 37};  // I2C bus 0

    MCP9600 gn2_internal;
    MCP9600 lox_lower;
    MCP9600 lox_upper;

    utils::FrequencyLogger thermocouples_freq_logger{"Thermocouples"};

   public:
    void Setup() override {
        // Initialize the 3 MCP9600 thermocouples on I2C bus 0
        if (!gn2_internal.begin(kGn2InternalAddress, i2c0.wire))
            Die("gn2_internal init failed");
        gn2_internal.setThermocoupleType(TYPE_E);

        if (!lox_lower.begin(kLoxLowerAddress, i2c0.wire))
            Die("lox_lower init failed");
        lox_lower.setThermocoupleType(TYPE_E);

        if (!lox_upper.begin(kLoxUpperAddress, i2c0.wire))
            Die("lox_upper init failed");
        lox_upper.setThermocoupleType(TYPE_E);
    }

    void Loop() override {
        // Read temperatures from the 3 MCP9600 thermocouples
        float gn2_internal_celsius = gn2_internal.getThermocoupleTemp();
        float lox_lower_celsius = lox_lower.getThermocoupleTemp();
        float lox_upper_celsius = lox_upper.getThermocoupleTemp();

        // Check for sensor errors
        HandleMCP9600Fault("gn2_internal", gn2_internal, gn2_internal_celsius);
        HandleMCP9600Fault("lox_lower", lox_lower, lox_lower_celsius);
        HandleMCP9600Fault("lox_upper", lox_upper, lox_upper_celsius);

        FsThermocouplesPacket thermo_packet{
            .ts = micros(),
            .gn2_internal_celsius = CoalesceNaN(gn2_internal_celsius),
            .gn2_external_celsius = 0.0f,  // Not connected
            .lox_upper_celsius = CoalesceNaN(lox_upper_celsius),
            .lox_lower_celsius = CoalesceNaN(lox_lower_celsius),
            .dummy = 0,
        };
        Send(DeviceType::DevFsLoxGn2Transducers, thermo_packet);

        thermocouples_freq_logger.Tick();

        // PrintCelsius("gn2_internal", gn2_internal_celsius);
        // PrintCelsius("lox_lower", lox_lower_celsius);
        // PrintCelsius("lox_upper", lox_upper_celsius);

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
