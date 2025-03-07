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
        lox_upper.Tick();
        lox_lower.Tick();
        gn2_manifold_1.Tick();
        gn2_manifold_2.Tick();

        // raw values (not medians)
        FsLoxGn2TransducersPacket fs_transducers_packet{
            .ts = micros(),
            .lox_upper = lox_upper.GetLatestPsi(),
            .lox_lower = lox_lower.GetLatestPsi(),
            .gn2_manifold_1 = gn2_manifold_1.GetLatestPsi(),
            .gn2_manifold_2 = gn2_manifold_2.GetLatestPsi(),
        };

        SendToPi(fs_transducers_packet);

        transducers_freq_logger.Tick();

        serial_forwarder.Tick();

        // lox_upper.PrintLatestPsi();
        // lox_lower.PrintLatestPsi();
        // gn2_manifold_1.PrintLatestPsi();
        // gn2_manifold_2.PrintLatestPsi();

        // delay(500);

        FsCommandPacket command_packet;
        if (Receive(&command_packet) == 0 &&
            command_packet.command == FsCommand::RECALIBRATE_TRANSDUCERS) {
            Recalibrate();
        }
    }

    void Recalibrate() {
        lox_upper.Recalibrate(kCalibrateSamples);
        lox_lower.Recalibrate(kCalibrateSamples);
        gn2_manifold_1.Recalibrate(kCalibrateSamples);
        gn2_manifold_2.Recalibrate(kCalibrateSamples);
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

    static const int kForwardSerialRxPin = 15;
    static const int kForwardSerialTxPin = 16;

    static const unsigned long kForwardSerialBaud = 230400;

    utils::SerialForwarder serial_forwarder{"Serial Forwarder", Serial1,
                                            Serial2};

    // ===== for raspberry pi =====

    static const int kPiSerialRxPin = 40;
    static const int kPiSerialTxPin = 39;

    static const unsigned long kPiSerialBaud = 230400;

    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    // ===== for transducers =====

    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;
    const int kCalibrateSamples = 500;

    I2CWire i2c0{0, 2, 1};
    I2CWire i2c1{1, 6, 7};

    // https://kulite.com//assets/media/2017/06/CTL-312.pdf; with AD620
    MovingMedianADC<Adafruit_ADS1115> lox_upper{
        "lox_upper", i2c0,     ADCAddress::GND, ADCMode::SingleEnded_0,
        kRate,       GAIN_ONE, kContinuous,     kWindowSize,
        400.0,
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
        1260.0,
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
        1260.0,
    };
};

REGISTER_AVIONICS_DEVICE(DevFsLoxGn2Transducers);
