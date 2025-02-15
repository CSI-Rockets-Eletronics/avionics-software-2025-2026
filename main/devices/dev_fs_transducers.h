#include <moving_median_adc.h>

#include "avionics.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsTransducers : public Device {
   public:
    void Setup() override {}

    void Loop() override {
        gn2_manifold_1_adc.Tick();
        gn2_manifold_2_adc.Tick();

        Serial.print("gn2_manifold_1_adc: ");
        Serial.print(gn2_manifold_1_adc.GetMedianVolts());
        Serial.println(" volts");

        Serial.print("gn2_manifold_2_adc: ");
        Serial.print(gn2_manifold_2_adc.GetMedianVolts());
        Serial.println(" volts");
    }

   private:
    const bool kContinuous = true;
    const int kWindowSize = 50;

    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_1_adc{
        "gn2_manifold_1_adc", ADCMode::SingleEnded_1,
        RATE_ADS1115_860SPS,  GAIN_TWOTHIRDS,
        kContinuous,          kWindowSize,
    };
    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_2_adc{
        "gn2_manifold_2_adc", ADCMode::SingleEnded_1,
        RATE_ADS1115_860SPS,  GAIN_TWOTHIRDS,
        kContinuous,          kWindowSize,
    };
};

REGISTER_AVIONICS_DEVICE(DevFsTransducers);
