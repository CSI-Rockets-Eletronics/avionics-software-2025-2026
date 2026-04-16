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
        // Read frequency from channel 1 (INA1/INB1 single-ended operation)
        // Channel 1: actual capacitance measurement
        unsigned long raw_reading = fdc.getReading28(1);

        // Check for FDC error flags
        uint16_t dataMSB = fdc.read16FDC(FDC2214_DATA_CH1_MSB);
        bool err_amplitude_low = (dataMSB >> 12) & 0x1;   // ERRAW
        bool err_amplitude_high = (dataMSB >> 13) & 0x1;  // ERRWD

        if (err_amplitude_low) {
            Serial.println("WARNING: Amplitude too low - increase DRIVE_CURRENT_CH1");
        }
        if (err_amplitude_high) {
            Serial.println("WARNING: Amplitude too high - decrease DRIVE_CURRENT_CH1");
        }

        // Convert raw reading to frequency in Hz
        float freq_actual_hz = RawReadingToFrequency(raw_reading);

        // Calculate probe capacitance using simple LC formula
        float probe_cap_pf = CalculateProbeCapacitance(freq_actual_hz);

        // Calculate fill height percentage
        float height_percent = CapacitanceToHeightPercent(probe_cap_pf);

        // Read board temperature from MCP9700
        float board_temp_c = ReadBoardTemperature();

        // Print raw reading, frequency, probe capacitance, height, and board temp with high precision
        Serial.print("Raw: ");
        Serial.print(raw_reading);
        Serial.print(" | Freq: ");
        Serial.print(freq_actual_hz, 2);  // Hz with 2 decimals
        Serial.print(" Hz (");
        Serial.print(freq_actual_hz / 1000000.0f, 6);  // MHz with 6 decimals
        Serial.print(" MHz) | Probe Cap: ");
        Serial.print(probe_cap_pf, 6);  // pF with 6 decimals
        Serial.print(" pF | Height: ");
        Serial.print(height_percent, 2);
        Serial.print(" % | Board Temp: ");
        Serial.print(board_temp_c, 2);
        Serial.println(" C");

        // Create packet
        // cap_fill_base: fill height percentage (0-100%)
        // cap_fill_actual: probe capacitance in pF (range: ~0-500 pF fits in float)
        CapFillPacket cap_fill_packet{
            .ts = micros(),
            .cap_fill_base = height_percent,        // Fill height percentage
            .cap_fill_actual = probe_cap_pf,        // Probe capacitance in pF
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

        // Decode CLOCK_DIVIDERS_CH1
        uint8_t fin_sel = (clockDiv >> 12) & 0x3;
        uint16_t fref_div = clockDiv & 0x3FF;
        float f_clk = (config & 0x0200) ? 40.0e6f : 43.4e6f;  // bit 9: 1=ext(40MHz), 0=int(43.4MHz)
        float f_ref = f_clk / fref_div;

        // Decode deglitch from MUX_CONFIG
        uint8_t deglitch = muxConfig & 0x7;
        const char* deglitch_str[] = {"Reserved", "1 MHz", "Reserved", "Reserved", "3.3 MHz", "10 MHz", "Reserved", "33 MHz"};

        // Decode drive current
        uint8_t idrive = (drive >> 11) & 0x1F;  // bits 15:11
        float drive_current_ma = 0.016f + (idrive * 0.096f);  // Per datasheet

        Serial.println("========== FDC2214 DEBUG INFO ==========");
        Serial.print("Device ID: 0x");
        Serial.println(deviceId, HEX);

        Serial.print("Status: 0x");
        Serial.print(status, HEX);
        Serial.print(" | ERR_CHAN: ");
        Serial.print((status >> 14) & 0x03, BIN);
        Serial.print(" | ERR_AHW: ");
        Serial.print((status >> 13) & 0x01);
        Serial.print(" | ERR_AEW: ");
        Serial.println((status >> 12) & 0x01);

        Serial.print("Config: 0x");
        Serial.print(config, HEX);
        Serial.print(" | Active Chan: ");
        Serial.print((config >> 14) & 0x3);
        Serial.print(" | Clock: ");
        Serial.println((config & 0x0200) ? "External 40MHz" : "Internal 43.4MHz");

        Serial.print("MUX Config: 0x");
        Serial.print(muxConfig, HEX);
        Serial.print(" | Deglitch: ");
        Serial.println(deglitch_str[deglitch]);

        Serial.print("Clock Dividers CH1: 0x");
        Serial.print(clockDiv, HEX);
        Serial.print(" | FIN_SEL: /");
        Serial.print(fin_sel == 1 ? 1 : (fin_sel == 2 ? 2 : 4));
        Serial.print(" | FREF_DIV: ");
        Serial.print(fref_div);
        Serial.print(" | f_ref: ");
        Serial.print(f_ref / 1e6f, 3);
        Serial.println(" MHz");

        Serial.print("Drive CH1: 0x");
        Serial.print(drive, HEX);
        Serial.print(" | IDRIVE: ");
        Serial.print(idrive);
        Serial.print(" (");
        Serial.print(drive_current_ma, 3);
        Serial.println(" mA)");

        Serial.print("Data CH1 MSB: 0x");
        Serial.print(dataMSB, HEX);
        Serial.print(" | ERRAW: ");
        Serial.print((dataMSB >> 12) & 0x1);
        Serial.print(" | ERRWD: ");
        Serial.println((dataMSB >> 13) & 0x1);

        Serial.print("Data CH1 LSB: 0x");
        Serial.println(dataLSB, HEX);
        Serial.println("========================================");
    }

    // Convert raw 28-bit reading to frequency in Hz
    // FDC2214 formula: f_sensor = (CH_FIN_SEL * raw_reading * f_ref) / 2^28
    float RawReadingToFrequency(unsigned long raw_reading) {
        if (raw_reading == 0) {
            Serial.println("Warning: FDC2214 returned zero reading");
            return 0.0f;
        }

        // Read actual register values instead of hardcoding
        uint16_t config = fdc.read16FDC(FDC2214_CONFIG);
        uint16_t clockDiv = fdc.read16FDC(FDC2214_CLOCK_DIVIDERS_CH1);

        // Decode FIN_SEL and FREF_DIVIDER
        uint8_t fin_sel = (clockDiv >> 12) & 0x3;
        uint16_t fref_div = clockDiv & 0x3FF;

        // Validate FREF_DIVIDER
        if (fref_div < 1) {
            Serial.println("ERROR: FREF_DIVIDER must be > 0");
            return 0.0f;
        }

        // Determine clock source: bit 9 of CONFIG (1 = external, 0 = internal)
        float f_clk = (config & 0x0200) ? 40.0e6f : 43.4e6f;

        // Calculate reference frequency: f_ref = f_clk / FREF_DIVIDER
        float f_ref = f_clk / fref_div;

        // Determine FIN_SEL scaling factor
        // FIN_SEL: 01 = /1, 10 = /2, others = /4 (per datasheet)
        float fin_sel_factor;
        if (fin_sel == 1) {
            fin_sel_factor = 1.0f;
        } else if (fin_sel == 2) {
            fin_sel_factor = 2.0f;
        } else {
            fin_sel_factor = 4.0f;
        }

        const float k2pow28 = 268435456.0f;  // 2^28

        // FDC2214 formula: f_sensor = (CH_FIN_SEL * raw_reading * f_ref) / 2^28
        float freq_Hz = (fin_sel_factor * raw_reading * f_ref) / k2pow28;

        return freq_Hz;
    }

    // Calculate probe capacitance using simple LC formula
    // LC resonant formula: f = 1 / (2π√(L × C_total))
    // Where C_total = C_board + C_probe
    // Solving for C_probe: C_probe = C_total - C_board
    //                              = [1 / (4π²f²L)] - C_board
    float CalculateProbeCapacitance(float freq_Hz) {
        if (freq_Hz == 0) {
            Serial.println("Warning: Zero frequency in CalculateProbeCapacitance");
            return 0.0f;
        }

        const float kInductance = 10e-6f;      // L = 10 µH
        const float kBoardCap = 10e-12f;       // C_board = 10 pF (fixed capacitor on PCB)
        const float kPi = 3.14159265358979f;

        // Calculate total capacitance from frequency
        // C_total = 1 / (4π²f²L)
        float c_total = 1.0f / (4.0f * kPi * kPi * freq_Hz * freq_Hz * kInductance);

        // Subtract board capacitance to get probe capacitance
        float c_probe = c_total - kBoardCap;

        // Convert to pF for output
        float c_probe_pf = c_probe * 1e12f;

        // Prevent negative values from numerical issues
        if (c_probe_pf < 0.0f) {
            c_probe_pf = 0.0f;
        }

        return c_probe_pf;
    }

    // Convert frequency (in Hz) to capacitance
    // Using LC tank formula: f = 1 / (2π√(LC))
    // Solving for C: C = 1 / (4π²f²L)
    // NOTE: This is the Berkeley model - kept for backward compatibility
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

    // Calculate fill height percentage based on capacitance
    // Continuously tracks max capacitance and uses linear interpolation
    // between min (119.1474pF empty) and max (tracked full) capacitance
    float CapacitanceToHeightPercent(float probe_cap_pf) {
        const float kMinCapacitance = 119.1474;  // Empty tank capacitance in pF
        
        // Update max capacitance if current reading is higher
        if (probe_cap_pf > max_capacitance_pf) {
            max_capacitance_pf = probe_cap_pf;
        }

        // Calculate capacitance range
        float cap_range = max_capacitance_pf - kMinCapacitance;

        // If we haven't filled yet (max == min), return 100%
        if (cap_range == 0.0f) {  // Small threshold to avoid division by zero
            return 0.0f;
        }

        // Linear interpolation: percentage = (current - min) / (max - min) * 100
        float height_percent = ((probe_cap_pf - kMinCapacitance) / cap_range) * 100.0f;

        return height_percent;
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
    static const uint8_t kDeglitchValue = 0x005;   // 10 MHz deglitch (was 0x001 = 1 MHz, too low for 5-6 MHz sensor)
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

    // Track maximum capacitance observed for height percentage calculation
    float max_capacitance_pf = 119.1474;  // Initialize to empty tank capacitance
};

REGISTER_AVIONICS_DEVICE(DevCapFill);
