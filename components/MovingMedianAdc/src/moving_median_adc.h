#ifndef MOVING_MEDIAN_ADC_H_
#define MOVING_MEDIAN_ADC_H_

#include <Adafruit_ADS1X15.h>

namespace moving_median_adc {

// returns 0 if the vector is empty
template <typename T>
T Median(const std::vector<T>& a) {
    if (a.empty()) {
        return 0;
    }

    std::vector<T> b = a;
    int index = b.size() / 2;
    std::nth_element(b.begin(), b.begin() + index, b.end());
    return b[index];
}

template <typename T>
class MovingMedian {
   public:
    MovingMedian(int window_size, T initial_value = 0)
        : window_size{window_size} {
        for (int i = 0; i < window_size; i++) {
            values.push_back(initial_value);
        }
    }

    void Reset(T initial_value = 0) {
        for (int i = 0; i < window_size; i++) {
            values[i] = initial_value;
        }
    }

    void Add(T value) {
        // `values` is always kept at `window_size` elements
        values.erase(values.begin());
        values.push_back(value);
    }

    T GetMedian() { return Median(values); }

    T GetLatest() { return values.back(); }

   private:
    const int window_size;
    std::vector<T> values;
};

// https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/assembly-and-wiring#i2c-addressing-2974117
enum class ADCAddress {
    GND = 0x48,
    VIN = 0x49,
    SDA = 0x4A,
    SCL = 0x4B,
};

enum class ADCMode {
    SingleEnded_0,
    SingleEnded_1,
    SingleEnded_2,
    SingleEnded_3,
    Differential_0_1,
    Differential_2_3,
};

class I2CWire {
   public:
    TwoWire wire;

    I2CWire(uint8_t bus_num) : wire{bus_num} { wire.begin(); }
    I2CWire(uint8_t bus_num, int sda, int scl, uint32_t frequency = 0UL)
        : wire{bus_num} {
        wire.begin(sda, scl, frequency);
    }
};

// if continuous is true, the ADC must only be used for one channel
template <typename ADCType>
class MovingMedianADC {
   public:
    MovingMedianADC(const char* debug_name, I2CWire& i2c_wire,
                    ADCAddress address, ADCMode mode, uint16_t rate,
                    adsGain_t gain, bool continuous, int window_size,
                    float psi_per_volt)
        : debug_name{debug_name},
          mode{mode},
          continuous{continuous},
          psi_per_volt{psi_per_volt},
          median_volts{window_size} {
        adc.setDataRate(rate);
        adc.setGain(gain);

        if (!adc.begin(static_cast<uint8_t>(address), &i2c_wire.wire)) {
            PrintDebugTag();
            Die("ADS1115 begin failed");
        }

        if (continuous) {
            uint16_t mux = 0;

            switch (mode) {
                case ADCMode::SingleEnded_0:
                    mux = ADS1X15_REG_CONFIG_MUX_SINGLE_0;
                    break;
                case ADCMode::SingleEnded_1:
                    mux = ADS1X15_REG_CONFIG_MUX_SINGLE_1;
                    break;
                case ADCMode::SingleEnded_2:
                    mux = ADS1X15_REG_CONFIG_MUX_SINGLE_2;
                    break;
                case ADCMode::SingleEnded_3:
                    mux = ADS1X15_REG_CONFIG_MUX_SINGLE_3;
                    break;
                case ADCMode::Differential_0_1:
                    mux = ADS1X15_REG_CONFIG_MUX_DIFF_0_1;
                    break;
                case ADCMode::Differential_2_3:
                    mux = ADS1X15_REG_CONFIG_MUX_DIFF_2_3;
                    break;
            }

            adc.startADCReading(mux, true);
        }
    }

    // returns new zero volts
    float Recalibrate(int sample_count) {
        std::vector<float> samples;
        for (int i = 0; i < sample_count; i++) {
            samples.push_back(ReadVolts());
        }
        float newZeroVolts = Median(samples);
        SetZero(newZeroVolts);
        return newZeroVolts;
    }

    void ResetZero() { SetZero(0); }

    float GetLatestVolts() { return median_volts.GetLatest() - zero_volts; }
    float GetMedianVolts() { return median_volts.GetMedian() - zero_volts; }
    float GetLatestPsi() { return GetLatestVolts() * psi_per_volt; }
    float GetMedianPsi() { return GetMedianVolts() * psi_per_volt; }

    // polls ADC for a new reading and saves it
    void Tick() { median_volts.Add(ReadVolts()); }

    void PrintLatestPsi() {
        PrintDebugTag();
        Serial.print(GetLatestPsi());
        Serial.println(" PSI");
    }

   private:
    const char* debug_name;
    const ADCMode mode;
    const bool continuous;
    const float psi_per_volt;

    ADCType adc;
    MovingMedian<float> median_volts;
    float zero_volts = 0;

    void SetZero(float new_zero_volts) {
        zero_volts = new_zero_volts;
        median_volts.Reset(new_zero_volts);

        PrintDebugTag();
        Serial.print("Zero set to ");
        Serial.print(zero_volts);
        Serial.println(" volts");
    }

    float ReadVolts() {
        int16_t counts = 0;

        if (continuous) {
            counts = adc.getLastConversionResults();
        } else {
            switch (mode) {
                case ADCMode::SingleEnded_0:
                    counts = adc.readADC_SingleEnded(0);
                    break;
                case ADCMode::SingleEnded_1:
                    counts = adc.readADC_SingleEnded(1);
                    break;
                case ADCMode::SingleEnded_2:
                    counts = adc.readADC_SingleEnded(2);
                    break;
                case ADCMode::SingleEnded_3:
                    counts = adc.readADC_SingleEnded(3);
                    break;
                case ADCMode::Differential_0_1:
                    counts = adc.readADC_Differential_0_1();
                    break;
                case ADCMode::Differential_2_3:
                    counts = adc.readADC_Differential_2_3();
                    break;
            }
        }

        return adc.computeVolts(counts);
    }

    void PrintDebugTag() {
        Serial.print(debug_name);
        Serial.print(": ");
    }
};

}  // namespace moving_median_adc

#endif  // MOVING_MEDIAN_ADC_H_
