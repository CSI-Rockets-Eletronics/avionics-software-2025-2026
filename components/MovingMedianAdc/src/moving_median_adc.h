#ifndef MOVING_MEDIAN_ADC_H_
#define MOVING_MEDIAN_ADC_H_

#include <Adafruit_ADS1X15.h>

namespace moving_median_adc {

enum class ADCMode {
    SingleEnded_0,
    SingleEnded_1,
    SingleEnded_2,
    SingleEnded_3,
    Differential_0_1,
    Differential_2_3
};

// if continuous is true, the ADC must only be used for one channel
template <typename ADCType>
class MovingMedianADC {
   public:
    MovingMedianADC(const char* debug_name, ADCMode mode, uint16_t rate,
                    adsGain_t gain, bool continuous, int window_size)
        : debug_name{debug_name},
          mode{mode},
          continuous{continuous},
          median_volts{window_size} {
        adc.setDataRate(rate);
        adc.setGain(gain);

        if (!adc.begin()) {
            PrintDebugTag();
            Die("ADS1115 begin failed");
        }

        if (continuous) {
            uint16_t mux;

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
        setZero(newZeroVolts);
        return newZeroVolts;
    }

    void ResetZero() { SetZero(0); }

    float GetLatestVolts() { return median_volts.GetLatest() - zero_volts; }

    float GetMedianVolts() { return median_volts.GetMedian() - zero_volts; }

    // polls ADC for a new reading and saves it
    void Tick() { median_volts.Add(ReadVolts()); }

   private:
    const char* debug_name;
    const ADCMode mode;
    const bool continuous;

    ADCType adc;
    MovingMedian<float> median_volts;
    float zero_volts = 0;

    void SetZero(float new_zero_volts) {
        zero_volts = new_zero_volts;
        median_volts.Reset(new_zero_volts);

        PrintDebugTag();
        print("Zero set to ");
        print(zero_volts);
        println(" volts");
    }

    float ReadVolts() {
        int16_t counts;

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

}  // namespace moving_median_adc

#endif  // MOVING_MEDIAN_ADC_H_
