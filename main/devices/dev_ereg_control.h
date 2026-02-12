// EREG (Electronic Regulator) Control Device
// PID-based pressure regulation system with servo control

#include <Adafruit_ADS1X15.h>
#include <moving_median_adc.h>
#include <Arduino.h>
#include <ESP32Servo.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;
using namespace moving_median_adc;

// State machine for EREG operation
enum EregState
{
    EREG_CLOSED,   // Servo at 0 degrees (initial/safe state)
    EREG_STAGE_1,  // Servo at 5 degrees, vent until 450 PSI
    EREG_STAGE_2   // Active PID control
};

class DevEregControl : public Device {
   public:
    void Setup() override {
        Serial.setTimeout(10);
        // Serial is started in main.cpp; we only attach and center.
        g_servo_.attach(kServoPin, kPulseMinUs, kPulseMaxUs);
        // g_servo_.writeMicroseconds(kCenterUs);

        // Initialize I2C buses AFTER Arduino core is initialized
        InitializeI2C();

        Recalibrate();

        // Print CSV header
        Serial.println("time,CurrentAngle,UpperPSI,LowerPSI,dAngle,CycleTimes,setpoint,kp,ki,kd");
    }

    void Loop() override {
        // Handle incoming FsCommandPacket
        ParseCommand();

        // Handle incoming serial commands
        HandleSerialCommands();

        // Update sensor readings
        ereg_upper_.Tick();
        ereg_lower_.Tick();
        ereg_upper_psi_ = ereg_upper_.GetLatestPsi();
        ereg_lower_psi_ = ereg_lower_.GetLatestPsi();

        // Execute state machine logic
        unsigned long now = millis();

        // State machine logic
        if (current_state_ == EREG_CLOSED)
        {
            // Hold servo at center position (closed/safe state)
            g_servo_.writeMicroseconds(kCenterUs);
        }
        else if (current_state_ == EREG_STAGE_1)
        {
            // Open servo to 5 degrees and monitor pressure
            current_angle_ = 5.0f;
            int pw = map(5, -90, 90, kPulseMinUs, kPulseMaxUs);
            g_servo_.writeMicroseconds(pw);

            // Check pressure threshold every loop iteration (safety critical)
            if (ereg_lower_psi_ >= 450.0f)
            {
                current_state_ = EREG_CLOSED;
                Serial.println("Stage 1 complete: Pressure threshold reached, closing valve");
            }
        }
        else if (current_state_ == EREG_STAGE_2)
        {
            // Run PID control (only execute at PID_PERIOD_MS intervals)
            if (now - last_pid_ms_ >= kPidPeriodMs)
            {
                last_pid_ms_ = now;

                double measurement = static_cast<double>(ereg_lower_psi_);

                double dAngle = PidControl(setpoint_, measurement);

                current_angle_ += static_cast<float>(dAngle);

                if (current_angle_ > 70.0f)
                    current_angle_ = 70.0f;
                if (current_angle_ < 0.0f)
                    current_angle_ = 0.0f;

                current_angle_ = ApplyBacklashComp(current_angle_);

                int angle_int = static_cast<int>(current_angle_);
                int pw = map(angle_int, -90, 90, kPulseMinUs, kPulseMaxUs);
                g_servo_.writeMicroseconds(pw);

                last_print_time_ = now;
                Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                              now / 1000.0f,
                              current_angle_,
                              ereg_upper_psi_,
                              ereg_lower_psi_,
                              dAngle,
                              static_cast<double>(kPidPeriodMs),
                              setpoint_,
                              kp_,
                              ki_,
                              kd_);
            }
        }
    }

   private:
    void ParseCommand() {
        FsCommandPacket command_packet;

        if (Receive(&command_packet) != 0) {
            return;
        }

        switch (command_packet.command) {
            case FsCommand::EREG_CLOSED:
                current_state_ = EREG_CLOSED;
                Serial.println("Command received: EREG_CLOSED");
                break;
            case FsCommand::EREG_STAGE_1:
                current_state_ = EREG_STAGE_1;
                Serial.println("Command received: EREG_STAGE_1");
                break;
            case FsCommand::EREG_STAGE_2:
                current_state_ = EREG_STAGE_2;
                ResetPID();
                Serial.println("Command received: EREG_STAGE_2");
                break;
            case FsCommand::RESTART:
                Die("Restarting by command");
                break;
            default:
                // ignore commands we don't handle
                break;
        }

        // Send current state to DevFsLoxGn2Transducers
        SendStateToTransducers();
    }

    void SendStateToTransducers() {
        // Send state to DevFsLoxGn2Transducers so it can be included in their packet
        struct EregStateData {
            bool ereg_closed;
            bool ereg_stage_1;
            bool ereg_stage_2;
        } ereg_state_data{
            .ereg_closed = (current_state_ == EREG_CLOSED),
            .ereg_stage_1 = (current_state_ == EREG_STAGE_1),
            .ereg_stage_2 = (current_state_ == EREG_STAGE_2),
        };

        Send(DeviceType::DevFsLoxGn2Transducers, ereg_state_data);
    }

    // ===== Configuration Constants =====
    static constexpr int kServoPin = 12;
    static constexpr int kPulseMinUs = 500;  // pulse at -90°
    static constexpr int kPulseMaxUs = 2500; // pulse at +90°
    static constexpr int kCenterUs = (kPulseMinUs + kPulseMaxUs) / 2;

    // Transducer Configuration
    static constexpr uint16_t kRate = RATE_ADS1115_860SPS;
    static constexpr bool kContinuous = true;
    static constexpr int kWindowSize = 50;
    static constexpr int kCalibrateSamples = 500;

    // PID Configuration
    static constexpr double kPidPeriodMs = 6;
    static constexpr double kDt = 0.006;

    // ===== Member Variables =====

    // Servo control
    Servo g_servo_;

    // I2C buses
    I2CWire i2c2_{0, 5, 6};
    I2CWire i2c3_{1, 7, 15};

    // Pressure transducers
    // https://www.dataq.com/resources/pdfs/datasheets/WNK81MA.pdf
    MovingMedianADC<Adafruit_ADS1115> ereg_upper_{
        "ereg_upper",
        i2c2_,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_TWOTHIRDS, // 10k PSI = 4.5V; we read up to 5k PSI
        kContinuous,
        kWindowSize,
        75,
    };

    MovingMedianADC<Adafruit_ADS1115> ereg_lower_{
        "ereg_lower",
        i2c3_,
        ADCAddress::VIN,
        ADCMode::SingleEnded_1,
        kRate,
        GAIN_TWOTHIRDS, // 10k PSI = 4.5V; we read up to 5k PSI
        kContinuous,
        kWindowSize,
        75,
    };

    // State variables
    EregState current_state_ = EREG_CLOSED;
    float current_angle_ = 0.0f;
    float target_angle_ = 0.0f;
    float ereg_upper_psi_ = 0.0f;
    float ereg_lower_psi_ = 0.0f;

    // PID state
    double integral_ = 0.0;   // kept (unused by incremental form) to avoid unrelated edits
    double prev_error_ = 0.0;
    double prev2_error_ = 0.0;

    // PID gains
    double setpoint_ = 70.0f;
    double kp_ = 0.035;
    double ki_ = 0.000;
    double kd_ = 0.002;

    // Timing
    unsigned long last_pid_ms_ = 0;
    unsigned long last_print_time_ = 0;

    // Backlash compensation
    float last_cmd_angle_ = 0.0f;
    int last_cmd_dir_ = 0;

    // Control flag
    bool flag_ = false;

    // ===== Helper Methods =====

    void Recalibrate() {
        ereg_upper_.Recalibrate(kCalibrateSamples);
        ereg_lower_.Recalibrate(kCalibrateSamples);
    }

    double PidControl(double setpoint, double measurement) {
        // Incremental / velocity-form PID:
        // Returns Δu (here: Δangle) each cycle, meant to be applied as:
        // current_angle += dAngle;

        const double error = setpoint - measurement;

        // First difference of error
        const double de = error - prev_error_;

        // Second difference of error
        const double d2e = error - 2.0 * prev_error_ + prev2_error_;

        // Δu = Kp*Δe + Ki*e*dt + Kd*(Δ²e/dt)
        const double output = (kp_ * de) + (ki_ * error * kDt) + (kd_ * (d2e / kDt));

        // Shift error history
        prev2_error_ = prev_error_;
        prev_error_  = error;

        return output;
    }

    void InitializeI2C() {
        Serial.println("Initializing I2C buses...");

        // Initialize I2C buses
        i2c2_.begin();
        delay(100);
        i2c3_.begin();
        delay(100);

        Serial.println("Initializing ADCs...");

        // Initialize ADCs
        if (!ereg_upper_.Initialize())
        {
            Serial.println("ERROR: Failed to initialize upper ADC!");
        }

        if (!ereg_lower_.Initialize())
        {
            Serial.println("ERROR: Failed to initialize lower ADC!");
        }

        delay(100);
    }

    void ResetPID() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        prev2_error_ = 0.0;
    }

    void HandleSerialCommands() {
        // Try to read a full line (handles CRLF or LF)
        String line = Serial.readStringUntil('\n');
        if (line.length() == 0)
        {
            // some terminals send only CR; consume it if present
            line = Serial.readStringUntil('\r');
        }

        line.trim();

        // Check for recalibration command
        if (line.equalsIgnoreCase("recal"))
        {
            Serial.println("Recalibrating transducers...");
            Recalibrate();
            Serial.println("Recalibration complete.");
            return;
        }
        else if (line.equalsIgnoreCase("start"))
        {
            flag_ = true;
            Serial.println("PID started.");
            ResetPID();
            return;
        }
        else if (line.equalsIgnoreCase("stop"))
        {
            flag_ = false;
            Serial.println("PID stopped.");
            return;
        }
        else if (line.equalsIgnoreCase("closed"))
        {
            current_state_ = EREG_CLOSED;
            Serial.println("State: EREG_CLOSED");
            return;
        }
        else if (line.equalsIgnoreCase("stage1"))
        {
            current_state_ = EREG_STAGE_1;
            Serial.println("State: EREG_STAGE_1 (vent mode)");
            return;
        }
        else if (line.equalsIgnoreCase("stage2"))
        {
            current_state_ = EREG_STAGE_2;
            ResetPID();
            Serial.println("State: EREG_STAGE_2 (PID control active)");
            return;
        }
        else if (line.startsWith("gains = ["))
        {
            float new_setpoint, new_Kp, new_Ki, new_Kd;
            if (sscanf(line.c_str(), "gains = [%f, %f, %f, %f]", &new_setpoint, &new_Kp, &new_Ki, &new_Kd) == 4 ||
                sscanf(line.c_str(), "gains = [%f,%f,%f,%f]", &new_setpoint, &new_Kp, &new_Ki, &new_Kd) == 4)
            {
                setpoint_ = new_setpoint;
                kp_ = new_Kp;
                ki_ = new_Ki;
                kd_ = new_Kd;
                Serial.printf("Gains updated: setpoint=%.5f, Kp=%.5f, Ki=%.5f, Kd=%.5f\n", setpoint_, kp_, ki_, kd_);
                ResetPID();
            }
            else
            {
                Serial.println("Invalid format. Use: gains = [setpoint, Kp, Ki, Kd]");
            }
            return;
        }
        else if (line.length() != 0)
        {
            int angle = line.toInt();

            // Clamp to your servo range: -90° .. +90°
            angle = constrain(angle, -90, 90);

            // Update your state
            current_angle_ = static_cast<float>(angle);
            target_angle_ = current_angle_; // optional, if you use target_angle

            current_angle_ = ApplyBacklashComp(current_angle_);

            // Map angle to pulse and command the servo immediately
            int angle_int = static_cast<int>(current_angle_);
            int pw = map(angle_int, -90, 90, kPulseMinUs, kPulseMaxUs);
            g_servo_.writeMicroseconds(pw);

            // Optionally stop PID so manual position holds
            flag_ = false;

            Serial.printf("Manual angle %d -> %d us\n> ", angle_int, pw);
            return;
        }
    }

    float ApplyBacklashComp(float cmd_angle) {
        int new_dir = (cmd_angle > last_cmd_angle_) ? 1 : (cmd_angle < last_cmd_angle_) ? -1 : 0;

        if (last_cmd_dir_ != 0 && new_dir != 0 && new_dir != last_cmd_dir_)
        {
            cmd_angle += (new_dir > 0) ? 0.0f : -0.0f;
        }

        cmd_angle = constrain(cmd_angle, -90.0f, 90.0f);

        if (new_dir != 0)
            last_cmd_dir_ = new_dir;

        last_cmd_angle_ = cmd_angle;
        return cmd_angle;
    }
};

REGISTER_AVIONICS_DEVICE(DevEregControl);
