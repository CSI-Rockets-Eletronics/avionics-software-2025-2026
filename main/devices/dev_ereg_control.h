// EREG (Electronic Regulator) Control Device
// PID-based pressure regulation system with servo control

#include <Arduino.h>
#include <ESP32Servo.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"
#include "dev_fs_lox_gn2_transducers.h"

using namespace avionics;

// State machine for EREG operation
enum EregState
{
    EREG_CLOSED,   // Servo at closed position (initial/safe state) - always takes precedence
    EREG_STAGE_1,  // PID control, angle clamped 0-13 degrees
    EREG_STAGE_2   // PID control, angle clamped 0-90 degrees
};

class DevEregControl : public Device {
   public:
    void Setup() override {
        g_servo_.attach(kServoPin, kPulseMinUs, kPulseMaxUs);

        // Get reference to transducers device
        auto maybe_device = Node::FindDevice(DeviceType::DevFsLoxGn2Transducers);
        if (!maybe_device) {
            Die("EREG: Cannot find transducers device!");
        }
        transducers_ = static_cast<DevFsLoxGn2Transducers*>(&maybe_device.value().get());
    }

    void Loop() override {
        // EREG_CLOSED takes absolute precedence: always enforce it first
        // before processing any new commands, so a CLOSED command received
        // last loop iteration is never overridden by stale state.
        ParseCommand();

        // Get pressure readings from transducers device
        // ereg_upper uses copv_1, ereg_lower uses oxtank_1
        // The transducers are already being Tick()'d in DevFsLoxGn2Transducers::Loop()
        ereg_upper_psi_ = transducers_->copv_1.GetLatestPsi();
        ereg_lower_psi_ = transducers_->oxtank_1.GetLatestPsi();

        // Safety check: automatically close EREG if lower pressure exceeds safety limit
        if (ereg_lower_psi_ > kMaxSafePressurePsi) {
            SetState(EREG_CLOSED);
        }

        unsigned long now = millis();

        if (current_state_ == EREG_CLOSED)
        {
            // CLOSED state: hold servo at closed position
            // Resets angle so PID starts fresh if a stage is later commanded
            current_angle_ = 0.0f;
            g_servo_.writeMicroseconds(kCenterUs);
        }
        else if (current_state_ == EREG_STAGE_1)
        {
            // STAGE 1: PID control, angle clamped between 0 and 13 degrees
            RunPidLoop(now, kStage1MaxAngle);
        }
        else if (current_state_ == EREG_STAGE_2)
        {
            // STAGE 2: PID control, angle clamped between 0 and 90 degrees
            RunPidLoop(now, kStage2MaxAngle);
        }
    }

   private:
    // ===== State Transition =====

    // All state changes go through here to enforce CLOSED precedence.
    // EREG_CLOSED can always be entered from any state.
    // EREG_STAGE_1 / EREG_STAGE_2 can only be entered if not currently CLOSED
    // by an active hold -- but per requirements, any explicit command is honored;
    // CLOSED simply overrides everything when commanded.
    void SetState(EregState new_state) {
        if (new_state == EREG_CLOSED) {
            // CLOSED takes absolute precedence -- unconditionally enter it
            current_state_ = EREG_CLOSED;
            Serial.println("EREG: CLOSED (overrides all other states)");
        } else if (current_state_ == EREG_CLOSED && new_state != EREG_CLOSED) {
            // Leaving CLOSED: only allowed when an explicit stage command arrives
            current_state_ = new_state;
            Serial.println(new_state == EREG_STAGE_1
                ? "EREG: STAGE_1 (PID active, 0-13 deg)"
                : "EREG: STAGE_2 (PID active, 0-90 deg)");
        } else {
            // Transitioning between STAGE_1 and STAGE_2: allowed freely
            current_state_ = new_state;
            Serial.println(new_state == EREG_STAGE_1
                ? "EREG: STAGE_1 (PID active, 0-13 deg)"
                : "EREG: STAGE_2 (PID active, 0-90 deg)");
        }
    }

    void ParseCommand() {
        FsCommandPacket command_packet;

        if (Receive(&command_packet) != 0) {
            return;
        }

        switch (command_packet.command) {
            case FsCommand::EREG_CLOSED:
                SetState(EREG_CLOSED);
                break;
            case FsCommand::EREG_STAGE_1:
                SetState(EREG_STAGE_1);
                // Bumpless transfer: seed error history to current error
                // so P and D terms don't spike on entry
                BumplessResetPID();
                break;
            case FsCommand::EREG_STAGE_2:
                SetState(EREG_STAGE_2);
                // Bumpless transfer: seed error history to current error
                BumplessResetPID();
                break;
            case FsCommand::RESTART:
                Die("Restarting by command");
                break;
            default:
                // ignore commands we don't handle
                break;
        }

        // Always broadcast current state after any command
        SendStateToTransducers();
    }

    void SendStateToTransducers() {
        struct EregStateData {
            bool ereg_closed;
            bool ereg_stage_1;
            bool ereg_stage_2;
        } ereg_state_data{
            .ereg_closed  = (current_state_ == EREG_CLOSED),
            .ereg_stage_1 = (current_state_ == EREG_STAGE_1),
            .ereg_stage_2 = (current_state_ == EREG_STAGE_2),
        };

        Send(DeviceType::DevFsLoxGn2Transducers, ereg_state_data);
    }

    // ===== PID Execution =====

    // Shared PID loop called by both STAGE_1 and STAGE_2.
    // max_angle_deg is the only difference between the two stages.
    void RunPidLoop(unsigned long now, float max_angle_deg) {
        if (now - last_pid_ms_ < kPidPeriodMs) {
            return;
        }
        last_pid_ms_ = now;

        // Update dynamic gains based on current upper transducer pressure
        UpdateDynamicGains(ereg_upper_psi_);

        double measurement = static_cast<double>(ereg_lower_psi_);
        double dAngle = PidControl(setpoint_, measurement);

        current_angle_ += static_cast<float>(dAngle);

        // Clamp to stage-specific range
        if (current_angle_ > max_angle_deg)
            current_angle_ = max_angle_deg;
        if (current_angle_ < 0.0f)
            current_angle_ = 0.0f;

        current_angle_ = ApplyBacklashComp(current_angle_);

        // Float-precision PWM mapping (avoids integer rounding loss)
        float pw_f = (float)kPulseMinUs +
                     ((current_angle_ + 90.0f) * (float)(kPulseMaxUs - kPulseMinUs) / 180.0f);
        int pw = (int)(pw_f + 0.5f);
        g_servo_.writeMicroseconds(pw);
    }

    // ===== PID Algorithm =====

    double PidControl(double setpoint, double measurement) {
        // Incremental / velocity-form PID.
        // Returns delta-angle each cycle; caller does: current_angle_ += dAngle.

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

    // Hard reset -- zeros all error history
    void ResetPID() {
        integral_    = 0.0;
        prev_error_  = 0.0;
        prev2_error_ = 0.0;
    }

    // Bumpless transfer -- seeds error history to current error so
    // P and D terms start at zero instead of spiking on state entry
    void BumplessResetPID() {
        integral_ = 0.0;
        double e0    = setpoint_ - static_cast<double>(ereg_lower_psi_);
        prev_error_  = e0;
        prev2_error_ = e0;
    }

    // ===== Dynamic Gain Scaling =====

    // Placeholder scale function: returns 1.0 (no scaling) until
    // calibrated during testing. Replace body with empirical algorithm.
    double GainScaleFromUpperPsi(float upper_psi) {
        (void)upper_psi;
        return 1.0;
    }

    // Called once per PID cycle. Scales active gains from base gains.
    void UpdateDynamicGains(float upper_psi) {
        const double scale = GainScaleFromUpperPsi(upper_psi);
        kp_ = kp_base_ * scale;
        ki_ = ki_base_ * scale;
        kd_ = kd_base_ * scale;
    }

    // ===== Hardware Helpers =====

    float ApplyBacklashComp(float cmd_angle) {
        int new_dir = (cmd_angle > last_cmd_angle_) ? 1
                    : (cmd_angle < last_cmd_angle_) ? -1
                    : 0;

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

    // ===== Configuration Constants =====

    static constexpr int kServoPin    = 12;
    static constexpr int kPulseMinUs  = 500;   // pulse at -90°
    static constexpr int kPulseMaxUs  = 2500;  // pulse at +90°
    static constexpr int kCenterUs    = (kPulseMinUs + kPulseMaxUs) / 2;

    // Stage angle limits
    static constexpr float kStage1MaxAngle = 13.0f;  // degrees
    static constexpr float kStage2MaxAngle = 90.0f;  // degrees

    // Safety limits
    static constexpr float kMaxSafePressurePsi = 550.0f;  // Auto-close if ereg_lower exceeds this

    // PID timing
    static constexpr double kPidPeriodMs = 6.0;
    static constexpr double kDt          = 0.006;

    // ===== Member Variables =====

    // Servo
    Servo g_servo_;

    // Pointer to transducers device for reading pressure data
    DevFsLoxGn2Transducers* transducers_ = nullptr;

    // State
    EregState current_state_ = EREG_CLOSED;
    float current_angle_     = 0.0f;
    float ereg_upper_psi_    = 0.0f;
    float ereg_lower_psi_    = 0.0f;

    // PID error history
    double integral_    = 0.0;  // unused by velocity-form but retained for symmetry
    double prev_error_  = 0.0;
    double prev2_error_ = 0.0;

    // PID gains -- base values define the unscaled setpoint
    // Active gains (kp_, ki_, kd_) are updated each cycle by UpdateDynamicGains()
    double setpoint_ = 70.0;

    double kp_base_ = 0.035;
    double ki_base_ = 0.000;
    double kd_base_ = 0.002;

    double kp_ = kp_base_;
    double ki_ = ki_base_;
    double kd_ = kd_base_;

    // Timing
    unsigned long last_pid_ms_ = 0;

    // Backlash compensation
    float last_cmd_angle_ = 0.0f;
    int   last_cmd_dir_   = 0;
};

REGISTER_AVIONICS_DEVICE(DevEregControl);
