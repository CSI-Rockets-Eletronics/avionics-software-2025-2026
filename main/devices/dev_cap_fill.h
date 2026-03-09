#include <FDC2214.h>
#include <Wire.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;

class DevCapFill : public Device {
   public:
    void Setup() override {
        // for serial to Scientific2 ESP32
        Serial1.begin(kOtherEsp32SerialBaud, SERIAL_8N1, kOtherEsp32SerialRxPin,
                      kOtherEsp32SerialTxPin);

        // Initialize I2C
        Wire.begin(kI2cSdaPin, kI2cSclPin);

        // Initialize FDC2214 sensor
        // Channel mask 0x02 = channel 1 only (INA1/INB1 single-ended)
        // Autoscan 0x00 = single channel mode
        // Deglitch 0x001 = 1 MHz
        // Use external oscillator = false
        if (!fdc.begin(kChannelMask, kAutoscanSeq, kDeglitchValue, kUseIntOsc)) {
            Die("FDC2214 initialization failed");
        }

        // Debug: Print device ID and status
        // Serial.println("FDC2214 initialized successfully");
        // PrintDebugInfo();

        delay(100);
    }

    void Loop() override {
        // Read frequency from channel 1 (INA1/INB1 single-ended operation)
        // Channel 1: actual capacitance measurement
        unsigned long freq_actual = fdc.getReading28(1);

        // For baseline, we'll use a reference reading or calibration value
        // TODO: Implement proper baseline calibration
        unsigned long freq_base = kBaselineFrequency;

        // Convert frequency to capacitance
        // TODO: Replace with actual calibration equation
        // Placeholder conversion: C = k / f^2 (simplified lumped element model)
        float cap_base = FrequencyToCapacitance(freq_base);
        float cap_actual = FrequencyToCapacitance(freq_actual);

        // Create packet
        CapFillPacket cap_fill_packet{
            .ts = micros(),
            .cap_fill_base = cap_base,
            .cap_fill_actual = cap_actual,
            .board_temp = 0,  // TODO: Implement temperature reading from FDC2214
        };

        SendToOtherEsp32(cap_fill_packet);

        freq_logger.Tick();

        // Control loop rate for high-speed mass flow data
        // 100 Hz update rate suitable for rocket thrust mass flow measurements
        delay(kLoopDelayMs);
    }

   private:
    // ===== Helper Methods =====

    // Print debug information about FDC2214 status
    /*
    void PrintDebugInfo() {
        uint16_t deviceId = fdc.read16FDC(FDC2214_DEVICE_ID);
        uint16_t status = fdc.read16FDC(FDC2214_STATUS);
        uint16_t config = fdc.read16FDC(FDC2214_CONFIG);
        uint16_t muxConfig = fdc.read16FDC(FDC2214_MUX_CONFIG);

        Serial.print("Device ID: 0x");
        Serial.println(deviceId, HEX);
        Serial.print("Status: 0x");
        Serial.println(status, HEX);
        Serial.print("Config: 0x");
        Serial.println(config, HEX);
        Serial.print("MUX Config: 0x");
        Serial.println(muxConfig, HEX);
    }
    */

    // Convert frequency reading to capacitance
    // TODO: Replace with actual calibration equation based on LC tank parameters
    float FrequencyToCapacitance(unsigned long frequency) {
        if (frequency == 0) {
            Serial.println("Warning: FDC2214 returned zero frequency");
            return 0.0f;
        }

        // Placeholder equation: C = k / f^2
        // where k is a calibration constant
        // FDC2214 measures frequency of LC oscillator: f = 1/(2*pi*sqrt(L*C))
        // Therefore: C = 1 / (4*pi^2*L*f^2)
        // Assuming L = 18 uH (typical for FDC2214 eval board)
        const float kInductance_uH = 18.0f;
        const float kPi = 3.14159265f;

        // Convert frequency reading to actual frequency in Hz
        // FDC2214 reading is raw counts, need to convert based on reference frequency
        // For external oscillator at 40 MHz: freq_Hz = (reading * fref) / 2^28
        const float kRefFreq_MHz = 40.0f;
        float freq_MHz = (frequency * kRefFreq_MHz) / 268435456.0f;  // 2^28
        float freq_Hz = freq_MHz * 1000000.0f;

        // Calculate capacitance in pF
        // C = 1 / (4 * pi^2 * L * f^2)
        float capacitance_pF = 1000000000000.0f / (4.0f * kPi * kPi * kInductance_uH * freq_Hz * freq_Hz);

        return capacitance_pF;
    }

    template <typename T>
    void SendToOtherEsp32(const T& data) {
        Serial1.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
        Serial1.write(kPacketDelimeter1);
        Serial1.write(kPacketDelimeter2);
    }

    // ===== Constants =====

    // I2C Configuration
    static const int kI2cSdaPin = 2;
    static const int kI2cSclPin = 1;

    // FDC2214 I2C address (ADDR pin low = 0x2A)
    static const uint8_t kFdcI2cAddress = FDC2214_I2C_ADDR_0;  // 0x2A

    // FDC2214 Configuration
    // Channel 1 only (INA1/INB1 single-ended operation)
    static const uint8_t kChannelMask = 0x02;      // Channel 1 only
    static const uint8_t kAutoscanSeq = 0x00;      // Single channel mode
    static const uint8_t kDeglitchValue = 0x001;   // 1 MHz deglitch
    static const bool kUseIntOsc = false;          // External oscillator

    // Baseline frequency for reference (to be calibrated)
    // TODO: Calibrate this value during initialization
    static const unsigned long kBaselineFrequency = 5000000;

    // Serial to Scientific2 ESP32
    // RX pin 38 connects to Scientific2's TX pin 37
    // TX pin 37 connects to Scientific2's RX pin 38
    static const int kOtherEsp32SerialRxPin = 38;  // Our RX <- Other's TX
    static const int kOtherEsp32SerialTxPin = 37;  // Our TX -> Other's RX

    static const unsigned long kOtherEsp32SerialBaud = 230400;

    // Packet delimiters
    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    // Loop timing - 100 Hz for high-speed mass flow measurements
    static const int kLoopDelayMs = 10;  // 100 Hz update rate

    // ===== Member Variables =====

    FDC2214 fdc{kFdcI2cAddress};
    utils::FrequencyLogger freq_logger{"CapFill"};
};

REGISTER_AVIONICS_DEVICE(DevCapFill);
