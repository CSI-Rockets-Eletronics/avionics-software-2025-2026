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
    I2CWire i2c3{3, 47, 21, 400000};  // Changed from bus 0 to bus 3, 400kHz
    I2CWire i2c4{4, 14, 13, 400000};  // Changed from bus 1 to bus 4, 400kHz

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
        i2c3,
        ADCAddress::VIN,
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

    // i2c3 transducers - pilot and qd pressure readings (ADC @ VIN address)
    MovingMedianADC<Adafruit_ADS1115> pilot_pres{
        "pilot_pres",
        i2c3,
        ADCAddress::VIN,
        ADCMode::SingleEnded_0,
        RATE_ADS1115_860SPS,
        GAIN_ONE,
        false,  // Changed to false - continuous mode only supports one channel per ADC
        50,
        375, //Todo
    };

    MovingMedianADC<Adafruit_ADS1115> qd_pres{
        "qd_pres",
        i2c4,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        RATE_ADS1115_860SPS,
        GAIN_ONE,
        false,  // Changed to false - continuous mode only supports one channel per ADC
        50,
        375, //Todo
        true   // debug_skip_init - TEMPORARILY skipping hardware init to avoid boot loop
    };

    void Setup() override {
        Serial.println("    DevFsLoxGn2Transducers::Setup() - Starting");

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

        // Send to fs_scientific1 PacketForwarder via ESP-NOW
        Send(DeviceType::DevFsPacketForwarder, fs_transducers_packet);

        transducers_freq_logger.Tick();

        // oxtank_1.PrintLatestPsi();
        // oxtank_2.PrintLatestPsi();
        // copv_1.PrintLatestPsi();
        // copv_2.PrintLatestPsi();
        // pilot_pres.PrintLatestPsi();
        // qd_pres.PrintLatestPsi();

        // delay(500);

        FsCommandPacket command_packet;
        EregStateData ereg_state_data;

        switch (Receive(&command_packet, &ereg_state_data)) {
            case 0:
                Serial.print("[GN2 TRANSDUCERS] Received FsCommandPacket, command: ");
                Serial.println(static_cast<int>(command_packet.command));

                // Forward EREG commands to DevEregControl (local device on same node)
                if (command_packet.command == FsCommand::EREG_CLOSED ||
                    command_packet.command == FsCommand::EREG_STAGE_1 ||
                    command_packet.command == FsCommand::EREG_STAGE_2) {
                    Serial.println("[GN2 TRANSDUCERS] Forwarding EREG command to DevEregControl");
                    Send(DeviceType::DevEregControl, command_packet);
                }

                if (command_packet.command == FsCommand::RESTART) {
                    Die("Restarting by command");
                }
                if (command_packet.command ==
                    FsCommand::RECALIBRATE_TRANSDUCERS) {
                    Recalibrate();
                }
                break;
            case 1:
                // Received EREG state from DevEregControl (local device on same node)
                ereg_state_ = ereg_state_data;
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

   private:
    // ===== misc =====

    utils::FrequencyLogger transducers_freq_logger{"Transducers"};

    // ===== for EREG state =====

    EregStateData ereg_state_;

    // ===== for transducers =====

    const uint16_t kRate = RATE_ADS1115_860SPS;
    const bool kContinuous = true;
    const int kWindowSize = 50;
    const int kCalibrateSamples = 100;  // Reduced from 500 for faster calibration
};


REGISTER_AVIONICS_DEVICE(DevFsLoxGn2Transducers);
