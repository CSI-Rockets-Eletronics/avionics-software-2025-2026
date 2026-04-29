#include "avionics.h"

using namespace avionics;

// Relay pins (top row, l to r): 6, 17, 8, 48, 39, 41
// Relay pins (bottom row, l to r): 4, 15, 21, 47, 38, 40

enum class RelayPin : int {
    GN2_DRAIN = 39,
    GN2_FILL = 38,
    DEPRESS = 15,
    PRESS_PILOT = 21,
    RUN = 8,
    LOX_FILL = 47,
    LOX_DISCONNECT = 48,
    IGNITER = 40,
    EREG_POWER = 41,
};

struct RelayStates {
    bool gn2_drain = false;
    bool gn2_fill = false;
    bool depress = false;
    bool press_pilot = false;
    bool run = false;
    bool lox_fill = false;
    bool lox_disconnect = false;
    bool igniter = false;
    bool ereg_power = false;
};

using MS = unsigned long;

class DevFsRelays : public Device {
   private:
    const MS kPilotValveOpenDurationMs = 1000;
    const MS kPilotValveClosedDurationMs = 9000;

    const MS kGN2FillOpenDurationMs = 5000;
    const MS kGN2FillClosedDurationMs = 5000;

    const MS kFillAPulseDurationMs = 500;
    const MS kFillBPulseDurationMs = 1000;
    const MS kFillCPulseDurationMs = 5000;

    // ENGINE_PRIME timing
    const MS kEnginePrimePilotOpenDelayMs = 1000;  // Press pilot open for 1s before GN2 fill

    // FIRE timing (starts after ENGINE_PRIME)
    const MS kFireIgniterOnDelayMs = 3000;         // Wait 3s after ereg_Stage2 activates before igniter fires
    const MS kFireIgniterOffDelayMs = 3500;        // 500ms pulse (3s + 500ms)
    const MS kFireRunOpenDelayMs = 10000;          // Wait 7s after igniter on, then open run (3s + 7s = 10s total)
    const MS kFireBackToStandbyDelayMs = 30000;    // 3s + 7s + 20s = 30s total

    // SAFETY: Maximum igniter pulse duration - NEVER exceed this
    const MS kMaxIgniterPulseDurationMs = 500;  // 500ms max

    // safety to make sure we don't hold open solenoids for too long
    // in the CUSTOM state
    const MS kMaxCustomOpenDurationMs = 30000;  // 30s

    FsState cur_state = FsState::STANDBY;

    // time of entering the current state
    MS enter_state_ms = millis();
    // time of entering a state in which we must pulse the pilot valve
    MS enter_depress_pulse_ms = millis();
    // time when igniter was last turned on (for safety timeout)
    MS igniter_on_ms = 0;
    // track if igniter is currently energized
    bool igniter_is_on = false;

    RelayStates relay_states;

    // Independent controls that persist across state changes
    bool manual_gn2_fill = false;
    bool manual_gn2_drain = false;
    bool manual_lox_fill = false;
    bool manual_lox_disconnect = false;

   public:
    void Setup() override {
        SetPinToOutput(RelayPin::GN2_DRAIN);
        SetPinToOutput(RelayPin::GN2_FILL);
        SetPinToOutput(RelayPin::DEPRESS);
        SetPinToOutput(RelayPin::PRESS_PILOT);
        SetPinToOutput(RelayPin::RUN);
        SetPinToOutput(RelayPin::LOX_FILL);
        SetPinToOutput(RelayPin::LOX_DISCONNECT);
        SetPinToOutput(RelayPin::IGNITER);
        SetPinToOutput(RelayPin::EREG_POWER);
    }

    void Loop() override {
        ParseCommand();
        TransitionStates();

        // CUSTOM state -> relays were set in ParseCommand()
        if (cur_state != FsState::CUSTOM) {
            UpdateRelayStates();
        }

        // SAFETY: Enforce maximum igniter pulse duration
        EnforceIgniterSafety();

        FlushRelays();
        SendState();
    }

    bool ShouldPulsePilotVent(FsState state) {
        return state == FsState::GN2_STANDBY || state == FsState::GN2_FILL ||
               state == FsState::GN2_PULSE_FILL_A ||
               state == FsState::GN2_PULSE_FILL_B ||
               state == FsState::GN2_PULSE_FILL_C;
    }

    void ParseCommand() {
        FsCommandPacket command_packet;

        if (Receive(&command_packet) != 0) {
            return;
        }

        Serial.print("[FS RELAYS] Received FsCommandPacket, command: ");
        Serial.println(static_cast<int>(command_packet.command));

        if (command_packet.command == FsCommand::RESTART) {
            Serial.println("[FS RELAYS] RESTART command received - rebooting!");
            Die("Restarting by command");
            return;
        }

        FsState prev_state = cur_state;

        switch (command_packet.command) {
            case FsCommand::STATE_CUSTOM:
                cur_state = FsState::CUSTOM;
                UpdateCustomRelayStates(command_packet);
                break;
            case FsCommand::STATE_ABORT:
                cur_state = FsState::ABORT;
                break;
            case FsCommand::STATE_STANDBY:
                cur_state = FsState::STANDBY;
                break;
            case FsCommand::STATE_GN2_STANDBY:
                cur_state = FsState::GN2_STANDBY;
                break;
            case FsCommand::STATE_GN2_FILL:
                cur_state = FsState::GN2_FILL;
                break;
            case FsCommand::STATE_GN2_PULSE_FILL_A:
                cur_state = FsState::GN2_PULSE_FILL_A;
                break;
            case FsCommand::STATE_GN2_PULSE_FILL_B:
                cur_state = FsState::GN2_PULSE_FILL_B;
                break;
            case FsCommand::STATE_GN2_PULSE_FILL_C:
                cur_state = FsState::GN2_PULSE_FILL_C;
                break;
            case FsCommand::STATE_ENGINE_PRIME:
                cur_state = FsState::ENGINE_PRIME;
                break;
            case FsCommand::STATE_FIRE:
                cur_state = FsState::FIRE;
                break;
            case FsCommand::STATE_FIRE_MANUAL_PRESS_PILOT:
                cur_state = FsState::FIRE_MANUAL_PRESS_PILOT;
                break;
            case FsCommand::STATE_FIRE_MANUAL_DOME_PILOT_CLOSE:
                cur_state = FsState::FIRE_MANUAL_DOME_PILOT_CLOSE;
                break;
            case FsCommand::STATE_FIRE_MANUAL_IGNITER:
                cur_state = FsState::FIRE_MANUAL_IGNITER;
                break;
            case FsCommand::STATE_FIRE_MANUAL_RUN:
                cur_state = FsState::FIRE_MANUAL_RUN;
                break;
            default:
                // ignore commands we don't want to handle
                break;
        }

        // Only reset the state timer if the state actually changed
        if (prev_state != cur_state) {
            enter_state_ms = millis();
        }

        if (ShouldPulsePilotVent(cur_state) &&
            !ShouldPulsePilotVent(prev_state)) {
            enter_depress_pulse_ms = millis();
        }

        Serial.print("[FS RELAYS] Entered state: ");
        Serial.print(static_cast<int>(cur_state));
        if (prev_state != cur_state) {
            Serial.print(" (changed from ");
            Serial.print(static_cast<int>(prev_state));
            Serial.println(")");
        } else {
            Serial.println(" (no change)");
        }
    }

    void TransitionStates() {
        MS time_in_state = millis() - enter_state_ms;

        if (cur_state == FsState::GN2_PULSE_FILL_A &&
            time_in_state >= kFillAPulseDurationMs) {
            cur_state = FsState::GN2_STANDBY;
        }
        if (cur_state == FsState::GN2_PULSE_FILL_B &&
            time_in_state >= kFillBPulseDurationMs) {
            cur_state = FsState::GN2_STANDBY;
        }
        if (cur_state == FsState::GN2_PULSE_FILL_C &&
            time_in_state >= kFillCPulseDurationMs) {
            cur_state = FsState::GN2_STANDBY;
        }

        if (cur_state == FsState::FIRE &&
            time_in_state >= kFireBackToStandbyDelayMs) {
            cur_state = FsState::STANDBY;
        }

        // SAFETY: Auto-transition FIRE_MANUAL_IGNITER after 500ms
        if (cur_state == FsState::FIRE_MANUAL_IGNITER &&
            time_in_state >= kMaxIgniterPulseDurationMs) {
            cur_state = FsState::STANDBY;
            Serial.println("[FS RELAYS SAFETY] FIRE_MANUAL_IGNITER auto-transitioned to STANDBY after 500ms");
        }

        if (cur_state == FsState::CUSTOM &&
            time_in_state >= kMaxCustomOpenDurationMs) {
            cur_state = FsState::STANDBY;
        }
    }

    void UpdateRelayStates() {
        // reset all relay states except ereg_power (persists beyond CUSTOM timeout)
        bool preserve_ereg_power = relay_states.ereg_power;
        relay_states = RelayStates();
        relay_states.ereg_power = preserve_ereg_power;

        // set all relays except for the pilot vent

        MS time_in_state = millis() - enter_state_ms;

        MS gn2_fill_period = kGN2FillOpenDurationMs + kGN2FillClosedDurationMs;
        MS time_in_gn2_fill_period = time_in_state % gn2_fill_period;

        switch (cur_state) {
            case FsState::CUSTOM:
                // this function won't be called in the CUSTOM state
                break;
            case FsState::ABORT:
                // ABORT: Open depress solenoid, close all others, keep ereg power on
                // All relay_states default to false (closed) except depress and ereg_power
                relay_states.depress = true;
                // Explicitly ensure all other solenoids are closed
                relay_states.gn2_drain = false;
                relay_states.gn2_fill = false;
                relay_states.press_pilot = false;
                relay_states.run = false;
                relay_states.lox_fill = false;
                relay_states.lox_disconnect = false;
                relay_states.igniter = false;
                // Keep ereg_power preserved (set above from previous state)
                break;
            case FsState::STANDBY:
            case FsState::GN2_STANDBY:
                // no relays to set
                break;
            case FsState::GN2_FILL:
                relay_states.gn2_fill =
                    time_in_gn2_fill_period < kGN2FillOpenDurationMs;
                break;
            case FsState::GN2_PULSE_FILL_A:
            case FsState::GN2_PULSE_FILL_B:
            case FsState::GN2_PULSE_FILL_C:
                relay_states.gn2_fill = true;
                break;
            case FsState::ENGINE_PRIME:
                // ENGINE_PRIME: press pilot open, wait 1s, then gn2_fill opens
                relay_states.press_pilot = true;
                if (time_in_state >= kEnginePrimePilotOpenDelayMs) {
                    relay_states.gn2_fill = true;
                }
                break;
            case FsState::FIRE:
                // FIRE: continue holding press pilot + gn2 fill from ENGINE_PRIME
                // Wait 10s for ereg_Stage2 to activate, then fire igniter for 500ms
                // Wait 7s after igniter on, then open run
                // Everything stays open for 20s after run opens (37s total)
                relay_states.press_pilot = true;
                relay_states.gn2_fill = true;

                if (time_in_state >= kFireIgniterOnDelayMs &&
                    time_in_state < kFireIgniterOffDelayMs) {
                    relay_states.igniter = true;
                }

                if (time_in_state >= kFireRunOpenDelayMs) {
                    relay_states.run = true;
                }
                break;
            case FsState::FIRE_MANUAL_PRESS_PILOT:
                relay_states.press_pilot = true;
                break;
            case FsState::FIRE_MANUAL_DOME_PILOT_CLOSE:
                relay_states.press_pilot = false;
                break;
            case FsState::FIRE_MANUAL_IGNITER:
                relay_states.igniter = true;
                break;
            case FsState::FIRE_MANUAL_RUN:
                relay_states.run = true;
                relay_states.lox_fill = true;
                break;
        }

        // set the pilot vent

        if (ShouldPulsePilotVent(cur_state)) {
            MS pulse_pilot_period =
                kPilotValveOpenDurationMs + kPilotValveClosedDurationMs;
            MS time_in_pulse_pilot_period =
                (millis() - enter_depress_pulse_ms) % pulse_pilot_period;

            relay_states.depress =
                time_in_pulse_pilot_period < kPilotValveOpenDurationMs;
        }

        // Apply manual overrides (OR operation allows manual control without exiting state)
        relay_states.gn2_drain = relay_states.gn2_drain || manual_gn2_drain;
        relay_states.gn2_fill = relay_states.gn2_fill || manual_gn2_fill;
        relay_states.lox_fill = relay_states.lox_fill || manual_lox_fill;
        relay_states.lox_disconnect = relay_states.lox_disconnect || manual_lox_disconnect;
    }

    void UpdateCustomRelayStates(FsCommandPacket command_packet) {
        relay_states.gn2_drain = command_packet.gn2_drain;
        relay_states.gn2_fill = command_packet.gn2_fill;
        relay_states.depress = command_packet.depress;
        relay_states.press_pilot = command_packet.press_pilot;
        relay_states.run = command_packet.run;
        relay_states.lox_fill = command_packet.lox_fill;
        relay_states.lox_disconnect = command_packet.lox_disconnect;
        relay_states.igniter = command_packet.igniter;
        relay_states.ereg_power = command_packet.ereg_power;

        // Update manual overrides (persist when leaving CUSTOM state)
        manual_gn2_drain = command_packet.gn2_drain;
        manual_gn2_fill = command_packet.gn2_fill;
        manual_lox_fill = command_packet.lox_fill;
        manual_lox_disconnect = command_packet.lox_disconnect;
    }

    // SAFETY: Enforce maximum igniter pulse duration
    // This runs every loop to ensure igniter never exceeds kMaxIgniterPulseDurationMs
    void EnforceIgniterSafety() {
        // Detect rising edge (igniter turned on)
        if (relay_states.igniter && !igniter_is_on) {
            igniter_on_ms = millis();
            igniter_is_on = true;
            Serial.println("[FS RELAYS SAFETY] Igniter turned ON");
        }

        // Detect falling edge (igniter turned off)
        if (!relay_states.igniter && igniter_is_on) {
            MS duration = millis() - igniter_on_ms;
            igniter_is_on = false;
            Serial.print("[FS RELAYS SAFETY] Igniter turned OFF after ");
            Serial.print(duration);
            Serial.println(" ms");
        }

        // SAFETY ENFORCEMENT: Force igniter off if exceeds max duration
        if (igniter_is_on) {
            MS time_on = millis() - igniter_on_ms;
            if (time_on >= kMaxIgniterPulseDurationMs) {
                relay_states.igniter = false;
                igniter_is_on = false;
                Serial.print("[FS RELAYS SAFETY] IGNITER SAFETY CUTOFF at ");
                Serial.print(time_on);
                Serial.println(" ms - FORCED OFF!");
            }
        }
    }

    void FlushRelays() {
        FlushRelay(RelayPin::GN2_DRAIN, relay_states.gn2_drain);
        FlushRelay(RelayPin::GN2_FILL, relay_states.gn2_fill);
        FlushRelay(RelayPin::DEPRESS, relay_states.depress);
        FlushRelay(RelayPin::PRESS_PILOT, relay_states.press_pilot);
        FlushRelay(RelayPin::RUN, relay_states.run);
        FlushRelay(RelayPin::LOX_FILL, relay_states.lox_fill);
        FlushRelay(RelayPin::LOX_DISCONNECT, relay_states.lox_disconnect);
        FlushRelay(RelayPin::IGNITER, relay_states.igniter);
        FlushRelay(RelayPin::EREG_POWER, relay_states.ereg_power);
    }

    void SetPinToOutput(RelayPin pin) {
        pinMode(static_cast<int>(pin), OUTPUT);
    }

    void FlushRelay(RelayPin pin, bool state) {
        digitalWrite(static_cast<int>(pin), state ? HIGH : LOW);
    }

    void SendState() {
        FsStatePacket state_packet{
            .ms_since_boot = millis(),
            .state = cur_state,
            .gn2_drain = relay_states.gn2_drain,
            .gn2_fill = relay_states.gn2_fill,
            .depress = relay_states.depress,
            .press_pilot = relay_states.press_pilot,
            .run = relay_states.run,
            .lox_fill = relay_states.lox_fill,
            .lox_disconnect = relay_states.lox_disconnect,
            .igniter = relay_states.igniter,
            .ereg_power = relay_states.ereg_power,
        };

        Send(DeviceType::DevFsLoxGn2Transducers, state_packet);
    }
};

REGISTER_AVIONICS_DEVICE(DevFsRelays);
