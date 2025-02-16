#include <moving_median_adc.h>

#include "avionics.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsTransducers : public Device {
   public:
    void Setup() override {}

    void Loop() override {
        gn2_manifold_1.Tick();
        gn2_manifold_2.Tick();

        Serial.print("gn2_manifold_1: ");
        Serial.print(gn2_manifold_1.GetMedianPsi());
        Serial.println(" PSI");

        Serial.print("gn2_manifold_2: ");
        Serial.print(gn2_manifold_2.GetMedianPsi());
        Serial.println(" PSI");
    }

   private:
    const bool kContinuous = true;
    const int kWindowSize = 50;

    // https://www.dataq.com/resources/pdfs/datasheets/WNK81MA.pdf
    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_1{
        "gn2_manifold_1",    ADCMode::SingleEnded_1,
        RATE_ADS1115_860SPS, GAIN_TWOTHIRDS,
        kContinuous,         kWindowSize,
        10000.0 / 4.5  // TODO calibrate
    };

    // https://www.dataq.com/resources/pdfs/datasheets/WNK81MA.pdf
    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_2{
        "gn2_manifold_2",    ADCMode::SingleEnded_1,
        RATE_ADS1115_860SPS, GAIN_TWOTHIRDS,
        kContinuous,         kWindowSize,
        10000.0 / 4.5  // TODO calibrate
    };
};

REGISTER_AVIONICS_DEVICE(DevFsTransducers);
