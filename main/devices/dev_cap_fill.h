#include <FDC2214.h>
#include <Wire.h>
#include <cmath>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;

class DevCapFill : public Device {
   public:
    void Setup() override {
        // for serial to Scientific2 ESP32
        // Serial1.begin(kOtherEsp32SerialBaud, SERIAL_8N1, kOtherEsp32SerialRxPin,
        //               kOtherEsp32SerialTxPin);

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
        Serial.println("FDC2214 initialized successfully");
        PrintDebugInfo();

        // Initialize MCP9700 temperature sensor ADC pin
        pinMode(kBoardTempPin, INPUT);
        analogSetAttenuation(ADC_11db);  // 0-3.3V range

        delay(100);
    }

    void Loop() override {
        // DEBUG: Read and print status register first
        uint16_t status = fdc.read16FDC(FDC2214_STATUS);
        Serial.print("STATUS Reg: 0x");
        Serial.print(status, HEX);
        Serial.print(" - DRDY_CH1: ");
        Serial.print((status & FDC2214_CH1_UNREADCONV) ? "YES" : "NO");
        Serial.print(", ERR_CHAN: ");
        Serial.print((status >> 14) & 0x03);
        Serial.print(", ERR_AHW: ");
        Serial.print((status >> 13) & 0x01);
        Serial.print(", ERR_AEW: ");
        Serial.println((status >> 12) & 0x01);

        // Read frequency from channel 1 (INA1/INB1 single-ended operation)
        // Channel 1: actual capacitance measurement
        unsigned long raw_reading = fdc.getReading28(1);

        // Convert raw reading to frequency in Hz
        float freq_actual_hz = RawReadingToFrequency(raw_reading);

        // For baseline, we'll use a reference reading or calibration value
        // TODO: Implement proper baseline calibration
        // float freq_base_hz = RawReadingToFrequency(kBaselineFrequency);

        // Convert frequency to capacitance
        // TODO: Replace with actual calibration equation
        // Placeholder conversion: C = k / f^2 (simplified lumped element model)
        float cap_base = FrequencyToCapacitance(freq_actual_hz);
        float cap_actual = FrequencyToHeight(freq_actual_hz);

        // Read board temperature from MCP9700
        float board_temp_c = ReadBoardTemperature();

        // Print capacitance and temperature
        Serial.print("Capacitance: ");
        Serial.print(cap_actual * 1e12, 2);  // Convert to pF for readability
        Serial.print(" pF | Frequency: ");
        Serial.print(freq_actual_hz / 1000000.0f, 3);  // Convert to MHz
        Serial.println(" MHz");
        Serial.print("Board Temp (C): ");
        Serial.println(board_temp_c);

        // Check for error flags in data register
        uint16_t dataMSB = fdc.read16FDC(FDC2214_DATA_CH1_MSB);
        uint16_t dataLSB = fdc.read16FDC(FDC2214_DATA_CH1_LSB);
        Serial.print("Data MSB: 0x");
        Serial.print(dataMSB, HEX);
        Serial.print(", LSB: 0x");
        Serial.println(dataLSB, HEX);

        if (dataMSB & FDC2214_DATA_CHx_MASK_ERRAW) {
            Serial.println("ERROR: Amplitude too high/low!");
        }
        if (dataMSB & FDC2214_DATA_CHx_MASK_ERRWD) {
            Serial.println("ERROR: Watchdog timeout - no oscillation!");
        }

        // Create packet
        CapFillPacket cap_fill_packet{
            .ts = micros(),
            .cap_fill_base = cap_base * 1e12f,
            .cap_fill_actual = cap_actual,
            .board_temp = static_cast<int8_t>(board_temp_c),
        };

        // Send via ESP-NOW to gn2transducers node
        Send(DeviceType::DevFsLoxGn2Transducers, cap_fill_packet);

        freq_logger.Tick();

        // Control loop rate for high-speed mass flow data
        // 100 Hz update rate suitable for rocket thrust mass flow measurements
        delay(kLoopDelayMs);
    }

   private:
    // ===== Helper Methods =====

    // Read temperature from MCP9700 sensor
    // MCP9700: Vout = 500mV @ 0°C, 10mV/°C
    // Temperature(°C) = (Vout - 500mV) / 10mV
    float ReadBoardTemperature() {
        // Read ADC value (12-bit: 0-4095)
        int adc_reading = analogRead(kBoardTempPin);

        // Convert ADC reading to voltage in millivolts
        // ESP32 ADC: 0-4095 maps to 0-3300mV with 11dB attenuation
        float voltage_mv = (adc_reading / 4095.0f) * 3300.0f;

        // Convert voltage to temperature using MCP9700 formula
        // Temp(°C) = (Vout - 500mV) / 10mV/°C
        float temperature_c = (voltage_mv - 500.0f) / 10.0f;

        return temperature_c;
    }

    // Print debug information about FDC2214 status
    void PrintDebugInfo() {
        uint16_t deviceId = fdc.read16FDC(FDC2214_DEVICE_ID);
        uint16_t status = fdc.read16FDC(FDC2214_STATUS);
        uint16_t config = fdc.read16FDC(FDC2214_CONFIG);
        uint16_t muxConfig = fdc.read16FDC(FDC2214_MUX_CONFIG);
        uint16_t clockDiv = fdc.read16FDC(FDC2214_CLOCK_DIVIDERS_CH1);
        uint16_t drive = fdc.read16FDC(FDC2214_DRIVE_CH1);
        uint16_t dataMSB = fdc.read16FDC(FDC2214_DATA_CH1_MSB);
        uint16_t dataLSB = fdc.read16FDC(FDC2214_DATA_CH1_LSB);

        Serial.print("Device ID: 0x");
        Serial.println(deviceId, HEX);
        Serial.print("Status: 0x");
        Serial.println(status, HEX);
        Serial.print("  ERR_CHAN: ");
        Serial.println((status >> 14) & 0x03, BIN);
        Serial.print("  ERR_AHW: ");
        Serial.println((status >> 13) & 0x01);
        Serial.print("  ERR_AEW: ");
        Serial.println((status >> 12) & 0x01);
        Serial.print("Config: 0x");
        Serial.println(config, HEX);
        Serial.print("MUX Config: 0x");
        Serial.println(muxConfig, HEX);
        Serial.print("Clock Dividers CH1: 0x");
        Serial.println(clockDiv, HEX);
        Serial.print("Drive CH1: 0x");
        Serial.println(drive, HEX);
        Serial.print("Data CH1 MSB: 0x");
        Serial.println(dataMSB, HEX);
        Serial.print("Data CH1 LSB: 0x");
        Serial.println(dataLSB, HEX);
    }

    // Convert raw 28-bit reading to frequency in Hz
    // FDC2214 formula: f_sensor = (raw_reading * f_ref) / 2^28
    float RawReadingToFrequency(unsigned long raw_reading) {
        if (raw_reading == 0) {
            Serial.println("Warning: FDC2214 returned zero reading");
            return 0.0f;
        }

        // External oscillator: 40 MHz
        // With FREF_DIVIDER = 2: f_ref = 40 MHz / 2 = 20 MHz
        const float kRefFreq_MHz = 20.0f;  // Reference frequency in MHz (40 MHz / 2)
        const float k2pow28 = 268435456.0f;  // 2^28

        // Convert 28-bit reading to frequency in Hz
        float freq_MHz = (raw_reading * kRefFreq_MHz) / k2pow28;
        float freq_Hz = freq_MHz * 1000000.0f;

        return freq_Hz;
    }

    // Convert frequency (in Hz) to capacitance
    // Using LC tank formula: f = 1 / (2π√(LC))
    // Solving for C: C = 1 / (4π²f²L)
    float FrequencyToCapacitance(float freq_Hz) {
        if (freq_Hz == 0) {
            Serial.println("Warning: FDC2214 returned zero frequency");
            return 0.0f;
        }
        // Fixed LC tank values on the board
        const float kInductance = 10e-6f;     // L0 = 10 uH
        const float kCapacitance = 10e-12f;   // C0 = 10 pF
        const float kPi = 3.14159265f;

        // Optional extra fixed capacitance offset
        // Keep at zero unless you determine a known constant offset
        // that should be removed before calibration/use.
        const float kParasiticCap = 0.0f;

        float rootTerm = freq_Hz * kPi * sqrtf(kInductance * kCapacitance);

        if (rootTerm <= 0.0f) {
            Serial.println("Error: invalid rootTerm in FrequencyToSensorCapacitance");
            return 0.0f;
        }

        float alpha = (1.0f / rootTerm) - 1.0f;
        float cap_sensor = kCapacitance * (alpha * alpha - 1.0f);

        // Optional constant parasitic subtraction
        cap_sensor -= kParasiticCap;

        // Prevent small negative values from numerical issues
        if (cap_sensor < 0.0f) {
            cap_sensor = 0.0f;
        }
        return cap_sensor;
    }

    // Convert frequency reading to height using calibrated capacitance values.
    // This assumes capacitance varies linearly with height after converting
    // frequency to sensor capacitance via the Berkeley model.
    float FrequencyToHeight(unsigned long frequency) {
        if (frequency == 0) {
            Serial.println("Warning: FDC2214 returned zero frequency");
            return 0.0f;
        }

        // Choose full-scale height convention.
        // This matches your previous "overall height" convention:
        const float kFullHeight = 1.285f;  // meters

        // TODO: Replace these with measured values from actual calibration:
        // 1. Measure empty-tank frequency
        // 2. Convert it using FrequencyToSensorCapacitance(...)
        // 3. Store as kCapEmpty
        //
        // 1. Measure full-tank frequency
        // 2. Convert it using FrequencyToSensorCapacitance(...)
        // 3. Store as kCapFull
        const float kCapEmpty = 0.0f;
        const float kCapFull = 100e-12f;  // placeholder: 100 pF

        float cap_sensor = FrequencyToCapacitance(frequency);

        float delta_cap = kCapFull - kCapEmpty;
        if (fabsf(delta_cap) < 1e-18f) {
            Serial.println("Error: invalid capacitance calibration span");
            return 0.0f;
        }

        float h = kFullHeight * (cap_sensor - kCapEmpty) / delta_cap;

        // Clamp to physical range
        if (h < 0.0f) {
            h = 0.0f;
        } else if (h > kFullHeight) {
            h = kFullHeight;
        }

        // calculate h_tank
        float h_tank = h + 0.0254 + 0.01; 

        // calculate h_percent
        float h_percent = (h_tank/1.3204) * 100;

        return h_percent;
    }

    // template <typename T>
    // void SendToOtherEsp32(const T& data) {
    //     Serial1.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
    //     Serial1.write(kPacketDelimeter1);
    //     Serial1.write(kPacketDelimeter2);
    // }

    // ===== Constants =====

    // MCP9700 Temperature Sensor Configuration
    static const int kBoardTempPin = 10;  // GPIO 10 for analog temp sensor

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
    static const bool kUseIntOsc = false;          // Use external 40 MHz oscillator

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
