#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsTransducers : public Device {
   public:
    void Setup() override {}

    void Loop() override {
        lox_upper.Tick();
        lox_lower.Tick();
        injector_manifold_1.Tick();
        injector_manifold_2.Tick();
        gn2_manifold_1.Tick();
        gn2_manifold_2.Tick();

        // raw values (not medians)
        FsTransducers fs_transducers_packet{
            .lox_upper = lox_upper.GetLatestPsi(),
            .lox_lower = lox_lower.GetLatestPsi(),
            .injector_manifold_1 = injector_manifold_1.GetLatestPsi(),
            .injector_manifold_2 = injector_manifold_2.GetLatestPsi(),
            .gn2_manifold_1 = gn2_manifold_1.GetLatestPsi(),
            .gn2_manifold_2 = gn2_manifold_2.GetLatestPsi(),
        };

        // TODO do something with the packet...

        lox_upper.PrintLatestPsi();
        lox_lower.PrintLatestPsi();
        injector_manifold_1.PrintLatestPsi();
        injector_manifold_2.PrintLatestPsi();
        gn2_manifold_1.PrintLatestPsi();
        gn2_manifold_2.PrintLatestPsi();
    }

   private:
    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;

    // https://kulite.com//assets/media/2017/06/CTL-312.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> lox_upper{
        "lox_upper", ADCMode::SingleEnded_0, kRate, GAIN_ONE, kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://kulite.com//assets/media/2017/06/CTL-190.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> lox_lower{
        "lox_lower", ADCMode::SingleEnded_0, kRate, GAIN_ONE, kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://kulite.com//assets/media/2017/06/CTL-312.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> injector_manifold_1{
        "injector_manifold_1",
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://kulite.com//assets/media/2017/06/CTL-190.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> injector_manifold_2{
        "injector_manifold_2",
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://www.dataq.com/resources/pdfs/datasheets/WNK81MA.pdf
    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_1{
        "gn2_manifold_1",
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_TWOTHIRDS,
        kContinuous,
        kWindowSize,
        10000.0 / 4.5,  // TODO calibrate
    };

    // https://www.dataq.com/resources/pdfs/datasheets/WNK81MA.pdf
    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_2{
        "gn2_manifold_2",
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_TWOTHIRDS,
        kContinuous,
        kWindowSize,
        10000.0 / 4.5,  // TODO calibrate
    };
};

REGISTER_AVIONICS_DEVICE(DevFsTransducers);
