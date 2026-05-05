#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsInjectorTransducers : public Device {
   public:
    void Setup() override {
        Recalibrate();
    }

    void Loop() override {
        injector_1.Tick();
        injector_2.Tick();
        upper_cc.Tick();

        // raw values (not medians)
        FsInjectorTransducersPacket fs_transducers_packet{
            .ts = micros(),
            .injector_1 = injector_1.GetLatestPsi(),
            .injector_2 = injector_2.GetLatestPsi(),
            .upper_cc = upper_cc.GetLatestPsi(),
        };

        // Send to PacketForwarder on fs_scientific1 via ESP-NOW
        Send(DeviceType::DevFsPacketForwarder, fs_transducers_packet);

        transducers_freq_logger.Tick();

        FsThermocouplesPacket thermo_packet;
        FsCommandPacket command_packet;

        switch (Receive(&thermo_packet, &command_packet)) {
            case 0:
                // Forward thermocouple packet to PacketForwarder
                Send(DeviceType::DevFsPacketForwarder, thermo_packet);
                break;
            case 1:
                if (command_packet.command == FsCommand::RESTART) {
                    Die("Restarting by command");
                }
                if (command_packet.command ==
                    FsCommand::RECALIBRATE_TRANSDUCERS) {
                    Recalibrate();
                }
                break;
        }

        // injector_1.PrintLatestPsi();
        // injector_2.PrintLatestPsi();
        // upper_cc.PrintLatestPsi();

        // delay(500);
    }

    void Recalibrate() {
        injector_1.Recalibrate(kCalibrateSamples);
        injector_2.Recalibrate(kCalibrateSamples);
        upper_cc.Recalibrate(kCalibrateSamples);
    }

   private:
    // ===== misc =====

    utils::FrequencyLogger transducers_freq_logger{"Transducers"};

    // ===== for transducers =====

    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;
    const int kCalibrateSamples = 500;

    I2CWire i2c1{1, 47, 21};  // Changed from bus 0 to bus 1
    I2CWire i2c2{2, 14, 13};  // Changed from bus 1 to bus 2

    // dataq - using AIN0 (sensors not connected, values set to zero)
    MovingMedianADC<Adafruit_ADS1115> injector_1{
        "injector_1",
        i2c1,
        ADCAddress::GND,
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
        true,  // debug_skip_init - ignore I2C failures
    };

    // dataq - using AIN0 (sensors not connected, values set to zero)
    MovingMedianADC<Adafruit_ADS1115> injector_2{
        "injector_2",
        i2c2,
        ADCAddress::VIN,
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
        true,  // debug_skip_init - ignore I2C failures
    };

    // dataq - using AIN1 (sensors not connected, values set to zero)
    MovingMedianADC<Adafruit_ADS1115> upper_cc{
        "upper_cc",
        i2c1,
        ADCAddress::GND,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
        true,  // debug_skip_init - ignore I2C failures
    };
};

REGISTER_AVIONICS_DEVICE(DevFsInjectorTransducers);
