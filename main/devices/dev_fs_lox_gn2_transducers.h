#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;
using namespace moving_median_adc;

class DevFsLoxGn2Transducers : public Device {
   public:
    void Setup() override {
        // for serial forwarding
        Serial1.begin(kForwardSerialBaud, SERIAL_8N1, kForwardSerialRxPin,
                      kForwardSerialTxPin);

        // for raspberry pi
        Serial2.begin(kPiSerialBaud, SERIAL_8N1, kPiSerialRxPin,
                      kPiSerialTxPin);

        Recalibrate();
    }

    void Loop() override {
        oxtank_1.Tick();
        oxtank_2.Tick();
        oxtank_3.Tick();
        copv_1.Tick();
        copv_2.Tick();
        pilot_pres.Tick();
        qd_pres.Tick();

        // raw values (not medians)
        FsLoxGn2TransducersPacket fs_transducers_packet{
            .ts = micros(),
            .oxtank_1 = oxtank_1.GetLatestPsi(),
            .oxtank_2 = oxtank_2.GetLatestPsi(),
            .oxtank_3 = oxtank_3.GetLatestPsi(),
            .copv_1 = copv_1.GetLatestPsi(),
            .copv_2 = copv_2.GetLatestPsi(),
            .pilot_pres = pilot_pres.GetLatestPsi(),
            .qd_pres = qd_pres.GetLatestPsi(),
        };

        SendToPi(fs_transducers_packet);

        transducers_freq_logger.Tick();

        serial_forwarder.Tick();

        // oxtank_1.PrintLatestPsi();
        // oxtank_2.PrintLatestPsi();
        // oxtank_3.PrintLatestPsi();
        // copv_1.PrintLatestPsi();
        // copv_2.PrintLatestPsi();
        // pilot_pres.PrintLatestPsi();
        // qd_pres.PrintLatestPsi();

        // delay(500);

        FsCommandPacket command_packet;
        FsStatePacket state_packet;

        switch (Receive(&command_packet, &state_packet)) {
            case 0:
                if (command_packet.command == FsCommand::RESTART) {
                    Die("Restarting by command");
                }
                if (command_packet.command ==
                    FsCommand::RECALIBRATE_TRANSDUCERS) {
                    Recalibrate();
                }
                break;
            case 1:
                SendToPi(state_packet);
                break;
        }
    }

    void Recalibrate() {
        oxtank_1.Recalibrate(kCalibrateSamples);
        oxtank_2.Recalibrate(kCalibrateSamples);
        oxtank_3.Recalibrate(kCalibrateSamples);
        copv_1.Recalibrate(kCalibrateSamples);
        copv_2.Recalibrate(kCalibrateSamples);
        pilot_pres.Recalibrate(kCalibrateSamples);
        qd_pres.Recalibrate(kCalibrateSamples);
    }

    template <typename T>
    void SendToPi(const T& data) {
        Serial2.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
        Serial2.write(kPacketDelimeter1);
        Serial2.write(kPacketDelimeter2);
    }

   private:
    // ===== misc =====

    // just VS code intellisense being dumb; Serial2 is accessible globally
    HardwareSerial Serial2{2};
    utils::FrequencyLogger transducers_freq_logger{"Transducers"};

    // ===== for serial forwarding =====

    static const int kForwardSerialRxPin = 37;
    static const int kForwardSerialTxPin = 36;

    static const unsigned long kForwardSerialBaud = 230400;

    utils::SerialForwarder serial_forwarder{"Serial Forwarder", Serial1,
                                            Serial2};

    // ===== for raspberry pi =====

    static const int kPiSerialRxPin = 8;
    static const int kPiSerialTxPin = 18;

    static const unsigned long kPiSerialBaud = 230400;

    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    // ===== for transducers =====

    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;
    const int kCalibrateSamples = 500;

    I2CWire i2c3{0, 47, 21};
    I2CWire i2c4{1, 14, 13};

    // i2c3 transducers
    MovingMedianADC<Adafruit_ADS1115> oxtank_1{
        "oxtank_1",
        i2c3,
        ADCAddress::GND,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> oxtank_2{
        "oxtank_2",
        i2c3,
        ADCAddress::GND,
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> oxtank_3{
        "oxtank_3",
        i2c3,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0, //Todo
    };

    // i2c4 transducers
    MovingMedianADC<Adafruit_ADS1115> copv_1{
        "copv_1",
        i2c4,
        ADCAddress::GND,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> copv_2{
        "copv_2",
        i2c4,
        ADCAddress::GND,
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> pilot_pres{
        "pilot_pres",
        i2c4,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> qd_pres{
        "qd_pres",
        i2c4,
        ADCAddress::VIN,
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        1.0, //Todo
    };
};

REGISTER_AVIONICS_DEVICE(DevFsLoxGn2Transducers);
