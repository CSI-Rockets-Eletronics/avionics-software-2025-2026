#pragma once

#include <moving_median_adc.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;
using namespace moving_median_adc;

// Shared struct used by both DevFsLoxGn2Transducers and DevEregControl.
// Must be defined before either class so both sides of Send/Receive
// are guaranteed to use the identical type.
struct EregStateData {
    bool ereg_closed;
    bool ereg_stage_1;
    bool ereg_stage_2;
};

class DevFsLoxGn2Transducers : public Device {
   public:
    // I2C buses (needed by transducers)
    I2CWire i2c3{3, 47, 21};  // Changed from bus 0 to bus 3
    I2CWire i2c4{4, 14, 13};  // Changed from bus 1 to bus 4

    // Public transducers - accessed by DevEregControl for PID loop
    // i2c4 transducers - oxtank readings (ADC @ GND address)
    MovingMedianADC<Adafruit_ADS1115> oxtank_1{
        "oxtank_1",
        i2c4,
        ADCAddress::GND,
        ADCMode::SingleEnded_0,
        RATE_ADS1115_860SPS,
        GAIN_ONE,
        false,  // Changed to false - continuous mode only supports one channel per ADC
        50,
        375, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> oxtank_2{
        "oxtank_2",
        i2c4,
        ADCAddress::GND,
        ADCMode::SingleEnded_1,
        RATE_ADS1115_860SPS,
        GAIN_ONE,
        false,  // Changed to false - continuous mode only supports one channel per ADC
        50,
        375, //Todo
    };

    // i2c3 transducers - copv readings (ADC @ GND address)
    MovingMedianADC<Adafruit_ADS1115> copv_1{
        "copv_1",
        i2c3,
        ADCAddress::GND,
        ADCMode::SingleEnded_0,
        RATE_ADS1115_860SPS,
        GAIN_ONE,
        false,  // Changed to false - continuous mode only supports one channel per ADC
        50,
        1250, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> copv_2{
        "copv_2",
        i2c3,
        ADCAddress::GND,
        ADCMode::SingleEnded_1,
        RATE_ADS1115_860SPS,
        GAIN_ONE,
        false,  // Changed to false - continuous mode only supports one channel per ADC
        50,
        1250, //Todo
    };

    void Setup() override {
        Serial.println("    DevFsLoxGn2Transducers::Setup() - Starting");

        // for serial forwarding
        Serial.println("    Initializing Serial1 for forwarding");
        Serial1.begin(kForwardSerialBaud, SERIAL_8N1, kForwardSerialRxPin,
                      kForwardSerialTxPin);

        // for raspberry pi
        Serial.println("    Initializing Serial2 for Raspberry Pi");
        Serial2.begin(kPiSerialBaud, SERIAL_8N1, kPiSerialRxPin,
                      kPiSerialTxPin);

        Serial.println("    Calibrating transducers (this may take a moment)...");
        Recalibrate();
        Serial.println("    Transducers calibrated");

        // Initialize EREG state to CLOSED (safe default)
        ereg_state_.ereg_closed = true;
        ereg_state_.ereg_stage_1 = false;
        ereg_state_.ereg_stage_2 = false;

        Serial.println("    DevFsLoxGn2Transducers::Setup() - Complete");
    }

    void Loop() override {
        oxtank_1.Tick();
        oxtank_2.Tick();
        copv_1.Tick();
        copv_2.Tick();
        pilot_pres.Tick();
        qd_pres.Tick();

        // raw values (not medians)
        FsLoxGn2TransducersPacket fs_transducers_packet{
            .ts = micros(),
            .oxtank_1 = oxtank_1.GetLatestPsi(),
            .oxtank_2 = oxtank_2.GetLatestPsi(),
            .copv_1 = copv_1.GetLatestPsi(),
            .copv_2 = copv_2.GetLatestPsi(),
            .pilot_pres = pilot_pres.GetLatestPsi(),
            .qd_pres = qd_pres.GetLatestPsi(),
            .ereg_closed = ereg_state_.ereg_closed,
            .ereg_stage_1 = ereg_state_.ereg_stage_1,
            .ereg_stage_2 = ereg_state_.ereg_stage_2,
        };

        SendToPi(fs_transducers_packet);

        transducers_freq_logger.Tick();

        serial_forwarder.Tick();

        // oxtank_1.PrintLatestPsi();
        // oxtank_2.PrintLatestPsi();
        // copv_1.PrintLatestPsi();
        // copv_2.PrintLatestPsi();
        // pilot_pres.PrintLatestPsi();
        // qd_pres.PrintLatestPsi();

        // delay(500);

        FsCommandPacket command_packet;
        FsStatePacket state_packet;
        EregStateData ereg_state_data;
        RelayCurrentMonitorPacket relay_imon_packet;
        FsThermocouplesPacket thermo_packet;

        switch (Receive(&command_packet, &state_packet, &ereg_state_data, &relay_imon_packet, &thermo_packet)) {
            case 0:
                Serial.print("[GN2 TRANSDUCERS] Received FsCommandPacket, command: ");
                Serial.println(static_cast<int>(command_packet.command));
                if (command_packet.command == FsCommand::RESTART) {
                    Die("Restarting by command");
                }
                if (command_packet.command ==
                    FsCommand::RECALIBRATE_TRANSDUCERS) {
                    Recalibrate();
                }
                break;
            case 1:
                // Serial.print("[GN2 TRANSDUCERS] Received FsStatePacket from FsRelays, state: ");
                // Serial.print(static_cast<int>(state_packet.state));
                // Serial.print(", ms_since_boot: ");
                // Serial.println(state_packet.ms_since_boot);
                SendToPi(state_packet);
                break;
            case 2:
                // Received EREG state from DevEregControl
                // Serial.print("[GN2 TRANSDUCERS] Received EregStateData: closed=");
                // Serial.print(ereg_state_data.ereg_closed);
                // Serial.print(", stage1=");
                // Serial.print(ereg_state_data.ereg_stage_1);
                // Serial.print(", stage2=");
                // Serial.println(ereg_state_data.ereg_stage_2);
                ereg_state_ = ereg_state_data;
                break;
            case 3:
                // Received relay current monitor data from DevRelayImon
                // Serial.println("[GN2 TRANSDUCERS] Received RelayCurrentMonitorPacket from FsRelays");
                SendToPi(relay_imon_packet);
                break;
            case 4:
                // Received thermocouple data from DevFsThermocouples
                // Serial.println("[GN2 TRANSDUCERS] Received FsThermocouplesPacket");
                SendToPi(thermo_packet);
                break;
        }
    }

    void Recalibrate() {
        oxtank_1.Recalibrate(kCalibrateSamples);
        oxtank_2.Recalibrate(kCalibrateSamples);
        copv_1.Recalibrate(kCalibrateSamples);
        copv_2.Recalibrate(kCalibrateSamples);
        pilot_pres.Recalibrate(kCalibrateSamples);
        qd_pres.Recalibrate(kCalibrateSamples);
    }

    template <typename T>
    void SendToPi(const T& data) {
        size_t packet_size = sizeof(data);
        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&data);

        // Serial.print("[PI TX] Sending packet, size: ");
        // Serial.print(packet_size);

        // // Show first few bytes of actual data to confirm it's not all zeros
        // Serial.print(" bytes, data preview: 0x");
        // for (size_t i = 0; i < min(packet_size, (size_t)4); i++) {
        //     if (data_ptr[i] < 0x10) Serial.print("0");
        //     Serial.print(data_ptr[i], HEX);
        //     Serial.print(" ");
        // }

        // Write packet data
        size_t bytes_written = Serial2.write(data_ptr, packet_size);

        // Write delimiters
        size_t delim1_written = Serial2.write(kPacketDelimeter1);
        size_t delim2_written = Serial2.write(kPacketDelimeter2);

        // Verify all bytes were written
        if (bytes_written != packet_size) {
            Serial.print("[PI TX ERROR] Only wrote ");
            Serial.print(bytes_written);
            Serial.print("/");
            Serial.print(packet_size);
            Serial.println(" bytes!");
        } else if (delim1_written != 1 || delim2_written != 1) {
            Serial.println("[PI TX ERROR] Failed to write delimiters!");
        }
        // else {
        //     Serial.println(" [OK]");
        // }
    }

   private:
    // ===== misc =====

    // just VS code intellisense being dumb; Serial2 is accessible globally
    HardwareSerial Serial2{2};
    utils::FrequencyLogger transducers_freq_logger{"Transducers"};

    // ===== for EREG state =====

    EregStateData ereg_state_;

    // ===== for serial forwarding =====

    static const int kForwardSerialRxPin = 37;
    static const int kForwardSerialTxPin = 36;

    static const unsigned long kForwardSerialBaud = 230400;

    utils::SerialForwarder serial_forwarder{"Serial Forwarder", Serial1,
                                            Serial2};

    // ===== for raspberry pi =====

    static const int kPiSerialRxPin = 18;  // ESP32 RX <- Pi TX
    static const int kPiSerialTxPin = 8;   // ESP32 TX -> Pi RX

    static const unsigned long kPiSerialBaud = 115200;

    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    // ===== for transducers =====

    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;
    const int kCalibrateSamples = 100;  // Reduced from 500 for faster calibration

    // i2c3 transducers - pilot and qd pressure readings (ADC @ VIN address)
    MovingMedianADC<Adafruit_ADS1115> pilot_pres{
        "pilot_pres",
        i2c3,
        ADCAddress::VIN,
        ADCMode::SingleEnded_0,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        375, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> qd_pres{
        "qd_pres",
        i2c3,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_ONE,
        kContinuous,
        kWindowSize,
        100, //Todo
    };
};

REGISTER_AVIONICS_DEVICE(DevFsLoxGn2Transducers);
