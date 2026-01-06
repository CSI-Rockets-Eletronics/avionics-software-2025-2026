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
};

using MS = unsigned long;

class DevFsRelays : public Device {
   private:
    const MS kPilotValveOpenDurationMs = 5000;
    const MS kPilotValveClosedDurationMs = 5000;

    const MS kGN2FillOpenDurationMs = 5000;
    const MS kGN2FillClosedDurationMs = 5000;

    const MS kFillAPulseDurationMs = 1000;
    const MS kFillBPulseDurationMs = 5000;
    const MS kFillCPulseDurationMs = 10000;

    // dome pilot opens at T-15s
    const MS kFireDomePilotCloseDelayMs = 5000;  // T-10s
    const MS kFireIgniterOnDelayMs = 10000;      // T-5s
    const MS kFireIgniterOffDelayMs = 13000;     // T-2s
    const MS kFireRunOpenDelayMs = 15000;        // T-0s
    const MS kFireBackToStandbyDelayMs = 25000;  // T+10s

    // safety to make sure we don't hold open solenoids for too long
    // in the CUSTOM state
    const MS kMaxCustomOpenDurationMs = 45000;  // 45s

    FsState cur_state = FsState::STANDBY;

    // time of entering the current state
    MS enter_state_ms = millis();
    // time of entering a state in which we must pulse the pilot valve
    MS enter_depress_pulse_ms = millis();

    RelayStates relay_states;

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
    }

    void Loop() override {
        ParseCommand();
        TransitionStates();

        // CUSTOM state -> relays were set in ParseCommand()
        if (cur_state != FsState::CUSTOM) {
            UpdateRelayStates();
        }

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

        if (command_packet.command == FsCommand::RESTART) {
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

        enter_state_ms = millis();

        if (ShouldPulsePilotVent(cur_state) &&
            !ShouldPulsePilotVent(prev_state)) {
            enter_depress_pulse_ms = millis();
        }

        Serial.print("Entered state: ");
        Serial.println(static_cast<int>(cur_state));
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

        if (cur_state == FsState::CUSTOM &&
            time_in_state >= kMaxCustomOpenDurationMs) {
            cur_state = FsState::STANDBY;
        }
    }

    void UpdateRelayStates() {
        // reset all relay states
        relay_states = RelayStates();

        // set all relays except for the pilot vent

        MS time_in_state = millis() - enter_state_ms;

        MS gn2_fill_period = kGN2FillOpenDurationMs + kGN2FillClosedDurationMs;
        MS time_in_gn2_fill_period = time_in_state % gn2_fill_period;

        switch (cur_state) {
            case FsState::CUSTOM:
                // this function won't be called in the CUSTOM state
                break;
            case FsState::ABORT:
                relay_states.gn2_drain = true;
                relay_states.depress = true;
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
            case FsState::FIRE:
                if (time_in_state < kFireDomePilotCloseDelayMs) {
                    relay_states.press_pilot = true;
                } else if (time_in_state < kFireIgniterOnDelayMs) {
                    // do nothing; dome pilot is closed
                } else if (time_in_state < kFireIgniterOffDelayMs) {
                    relay_states.igniter = true;
                } else if (time_in_state < kFireRunOpenDelayMs) {
                    // do nothing; igniter is off
                } else {
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
        };

        Send(DeviceType::DevFsLoxGn2Transducers, state_packet);
    }
};

REGISTER_AVIONICS_DEVICE(DevFsRelays);
