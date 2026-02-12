#include "avionics.h"
#include "packets.h"

using namespace avionics;

// Current monitoring pins for each relay eFuse
// These are ADC-capable GPIO pins on ESP32
enum class ImonPin : int {
    GN2_DRAIN = 11,
    GN2_FILL = 2,
    DEPRESS = 16,
    PRESS_PILOT = 13,
    RUN = 9,
    LOX_FILL = 14,
    LOX_DISCONNECT = 10,
    IGNITER = 1,
    EREG_POWER = 3,
};

class DevRelayImon : public Device {
   private:
    // eFuse current sense amplifier gain: 246 µA/A
    // Shunt resistor: 1000 Ω
    // Output voltage = Current × 246µA/A × 1000Ω = Current × 0.246 mV/A
    // So: Current (A) = Voltage (mV) / 0.246
    // Or: Current (mA) = Voltage (mV) / 0.000246

    // ESP32 ADC: 12-bit (0-4095) for 0-3.3V
    // ADC reading to voltage: V = (reading / 4095) × 3300 mV
    // Voltage to current: I_mA = V_mV / 0.246
    // Combined: I_mA = (reading / 4095) × 3300 / 0.246
    //          I_mA = reading × 3.272

    static constexpr float kAdcToMilliamps = 3300.0f / 4095.0f / 0.246f;  // ≈ 3.272

    // Maximum expected current (for sanity checking)
    static constexpr int16_t kMaxCurrentMa = 10000;  // 10A max

   public:
    void Setup() override {
        // Configure ADC pins as input
        pinMode(static_cast<int>(ImonPin::GN2_DRAIN), INPUT);
        pinMode(static_cast<int>(ImonPin::GN2_FILL), INPUT);
        pinMode(static_cast<int>(ImonPin::DEPRESS), INPUT);
        pinMode(static_cast<int>(ImonPin::PRESS_PILOT), INPUT);
        pinMode(static_cast<int>(ImonPin::RUN), INPUT);
        pinMode(static_cast<int>(ImonPin::LOX_FILL), INPUT);
        pinMode(static_cast<int>(ImonPin::LOX_DISCONNECT), INPUT);
        pinMode(static_cast<int>(ImonPin::IGNITER), INPUT);
        pinMode(static_cast<int>(ImonPin::EREG_POWER), INPUT);

        // Set ADC attenuation to 11dB (0-3.3V range)
        analogSetAttenuation(ADC_11db);
    }

    void Loop() override {
        // Read all current monitor channels
        RelayCurrentMonitorPacket packet{
            .ts = micros(),
            .gn2_drain_ma = ReadCurrent(ImonPin::GN2_DRAIN),
            .gn2_fill_ma = ReadCurrent(ImonPin::GN2_FILL),
            .depress_ma = ReadCurrent(ImonPin::DEPRESS),
            .press_pilot_ma = ReadCurrent(ImonPin::PRESS_PILOT),
            .run_ma = ReadCurrent(ImonPin::RUN),
            .lox_fill_ma = ReadCurrent(ImonPin::LOX_FILL),
            .lox_disconnect_ma = ReadCurrent(ImonPin::LOX_DISCONNECT),
            .igniter_ma = ReadCurrent(ImonPin::IGNITER),
            .ereg_power_ma = ReadCurrent(ImonPin::EREG_POWER),
        };

        // Send to other ESP32s and Raspberry Pi
        Send(DeviceType::DevFsLoxGn2Transducers, packet);

        // Optional: Add a small delay to control sampling rate
        delay(10);  // 100 Hz sampling rate
    }

   private:
    int16_t ReadCurrent(ImonPin pin) {
        // Read ADC value
        int adc_reading = analogRead(static_cast<int>(pin));

        // Convert to milliamps
        float current_ma = adc_reading * kAdcToMilliamps;

        // Clamp to reasonable range and convert to int16_t
        if (current_ma > kMaxCurrentMa) {
            current_ma = kMaxCurrentMa;
        } else if (current_ma < 0) {
            current_ma = 0;
        }

        return static_cast<int16_t>(current_ma);
    }
};

REGISTER_AVIONICS_DEVICE(DevRelayImon);
