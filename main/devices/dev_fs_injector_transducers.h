#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsInjectorTransducers : public Device {
   public:
    void Setup() override {
        // for serial to other ESP32
        Serial1.begin(kOtherEsp32SerialBaud, SERIAL_8N1, kOtherEsp32SerialRxPin,
                      kOtherEsp32SerialTxPin);

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

        SendToOtherEsp32(fs_transducers_packet);

        transducers_freq_logger.Tick();

        FsThermocouplesPacket thermo_packet;
        FsCommandPacket command_packet;
        CapFillPacket cap_fill_packet;

        switch (Receive(&thermo_packet, &command_packet, &cap_fill_packet)) {
            case 0:
                SendToOtherEsp32(thermo_packet);
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
            case 2:
                SendToOtherEsp32(cap_fill_packet);
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

    template <typename T>
    void SendToOtherEsp32(const T& data) {
        Serial1.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
        Serial1.write(kPacketDelimeter1);
        Serial1.write(kPacketDelimeter2);
    }

   private:
    // ===== misc =====

    utils::FrequencyLogger transducers_freq_logger{"Transducers"};

    // ===== for serial to other ESP32 =====

    static const int kOtherEsp32SerialRxPin = 38;
    static const int kOtherEsp32SerialTxPin = 37;

    static const unsigned long kOtherEsp32SerialBaud = 230400;

    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    // ===== for transducers =====

    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;
    const int kCalibrateSamples = 500;

    I2CWire i2c1{0, 47, 21};
    I2CWire i2c2{1, 14, 13};

    // dataq
    MovingMedianADC<Adafruit_ADS1115> injector_1{
        "injector_1",
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

    // dataq
    MovingMedianADC<Adafruit_ADS1115> injector_2{
        "injector_2",
        i2c2,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0,  // TODO calibrate
        true,  // debug_skip_init - ignore I2C failures
    };

    // dataq
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
