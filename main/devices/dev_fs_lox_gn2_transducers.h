#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsLoxGn2Transducers : public Device {
   public:
    void Setup() override {}

    void Loop() override {
        lox_upper.Tick();
        lox_lower.Tick();
        gn2_manifold_1.Tick();
        gn2_manifold_2.Tick();

        // raw values (not medians)
        FsLoxGn2TransducersPacket fs_transducers_packet{
            .lox_upper = lox_upper.GetLatestPsi(),
            .lox_lower = lox_lower.GetLatestPsi(),
            .gn2_manifold_1 = gn2_manifold_1.GetLatestPsi(),
            .gn2_manifold_2 = gn2_manifold_2.GetLatestPsi(),
        };

        // TODO do something with the packet...

        lox_upper.PrintLatestPsi();
        lox_lower.PrintLatestPsi();
        gn2_manifold_1.PrintLatestPsi();
        gn2_manifold_2.PrintLatestPsi();

        delay(500);
    }

   private:
    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;

    I2CWire i2c0{0, 2, 1};
    I2CWire i2c1{1, 6, 7};

    // https://kulite.com//assets/media/2017/06/CTL-312.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> lox_upper{
        "lox_upper", i2c0,     ADCAddress::GND, ADCMode::SingleEnded_0,
        kRate,       GAIN_ONE, kContinuous,     kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://kulite.com//assets/media/2017/06/CTL-190.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> lox_lower{
        "lox_lower", i2c1,     ADCAddress::GND, ADCMode::SingleEnded_0,
        kRate,       GAIN_ONE, kContinuous,     kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://www.dataq.com/resources/pdfs/datasheets/WNK81MA.pdf
    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_1{
        "gn2_manifold_1",
        i2c0,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,  // 10k PSI = 4.5V; we read up to 5k PSI
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
    };

    // https://www.dataq.com/resources/pdfs/datasheets/WNK81MA.pdf
    MovingMedianADC<Adafruit_ADS1115> gn2_manifold_2{
        "gn2_manifold_2",
        i2c1,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,  // 10k PSI = 4.5V; we read up to 5k PSI
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
    };
};

REGISTER_AVIONICS_DEVICE(DevFsLoxGn2Transducers);
