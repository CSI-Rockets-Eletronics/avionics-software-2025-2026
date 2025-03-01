#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsChamberInjectorTransducers : public Device {
   public:
    void Setup() override {}

    void Loop() override {
        chamber.Tick();
        injector.Tick();

        // raw values (not medians)
        FsChamberInjectorTransducersPacket fs_transducers_packet{
            .chamber = chamber.GetLatestPsi(),
            .injector = injector.GetLatestPsi(),
        };

        // TODO do something with the packet...

        chamber.PrintLatestPsi();
        injector.PrintLatestPsi();

        delay(500);
    }

   private:
    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;

    I2CWire i2c2{0, 5, 6};
    I2CWire i2c3{1, 7, 15};

    // https://kulite.com//assets/media/2017/06/CTL-312.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> chamber{
        "chamber", i2c2,     ADCAddress::GND, ADCMode::SingleEnded_0,
        kRate,     GAIN_ONE, kContinuous,     kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://kulite.com//assets/media/2017/06/CTL-190.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> injector{
        "injector", i2c3,     ADCAddress::GND, ADCMode::SingleEnded_0,
        kRate,      GAIN_ONE, kContinuous,     kWindowSize,
        1.0,  // TODO calibrate
    };
};

REGISTER_AVIONICS_DEVICE(DevFsChamberInjectorTransducers);
