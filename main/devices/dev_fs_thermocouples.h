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

    I2CWire i2c0{0, 42, 37, 400000};  // I2C bus 0, 400kHz (same as working scanner)

    MCP9600 gn2_internal;
    MCP9600 lox_lower;
    MCP9600 lox_upper;

    utils::FrequencyLogger thermocouples_freq_logger{"Thermocouples"};

   public:
    void Setup() override {
        // Initialize the 3 MCP9600 thermocouples on I2C bus 0
        // MCP9600 is known to be "fussy" with I2C communication
        // Multiple retries and delays are needed for reliable initialization

        // Add initial delay for I2C bus to stabilize
        delay(200);

        // Initialize with retries - MCP9600 often fails on first attempt
        if (!InitMCP9600WithRetries(gn2_internal, kGn2InternalAddress, "gn2_internal"))
            Die("gn2_internal init failed after retries");
        delay(200);

        if (!InitMCP9600WithRetries(lox_lower, kLoxLowerAddress, "lox_lower"))
            Die("lox_lower init failed after retries");
        delay(200);

        if (!InitMCP9600WithRetries(lox_upper, kLoxUpperAddress, "lox_upper"))
            Die("lox_upper init failed after retries");
        delay(200);

        Serial.println("All MCP9600 thermocouples initialized successfully");
    }

    bool InitMCP9600WithRetries(MCP9600& tc, uint8_t address, const char* name) {
        const int maxRetries = 5;
        for (int attempt = 1; attempt <= maxRetries; attempt++) {
            Serial.print("Initializing ");
            Serial.print(name);
            Serial.print(" at 0x");
            Serial.print(address, HEX);
            Serial.print(" (attempt ");
            Serial.print(attempt);
            Serial.print("/");
            Serial.print(maxRetries);
            Serial.println(")");

            // Per MCP9600 library source: begin() calls checkDeviceID() which reads
            // the device ID register twice (first read often fails, second succeeds)
            // Do NOT call isConnected() after begin() as it will likely fail
            if (tc.begin(address, i2c0.wire)) {
                Serial.print(name);
                Serial.println(" begin() succeeded");

                // Wait for device to be ready
                delay(100);

                // Set thermocouple type
                uint8_t result = tc.setThermocoupleType(TYPE_E);
                if (result == 0) {
                    Serial.print(name);
                    Serial.println(" type set to TYPE_E");

                    // Wait before verification
                    delay(100);

                    // Verify the type was set correctly
                    Thermocouple_Type readType = tc.getThermocoupleType();
                    if (readType == TYPE_E) {
                        Serial.print(name);
                        Serial.println(" INITIALIZATION SUCCESSFUL");
                        return true;
                    } else {
                        Serial.print(name);
                        Serial.print(" type verification failed - got ");
                        Serial.println(readType);
                    }
                } else {
                    Serial.print(name);
                    Serial.print(" setThermocoupleType failed with code ");
                    Serial.println(result);
                }
            } else {
                Serial.print(name);
                Serial.println(" begin() returned false - device ID check failed");
            }

            // Wait before retry, increasing delay with each attempt
            Serial.print("Waiting before retry...");
            delay(300 * attempt);
            Serial.println("retrying");
        }

        Serial.print(name);
        Serial.println(" FAILED AFTER ALL RETRIES");
        return false;
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
