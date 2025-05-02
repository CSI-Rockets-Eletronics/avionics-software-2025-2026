#include "avionics.h"

using namespace avionics;

// Relay pins (left to right): 5, 1, 6, 7, 15, 16, 17, 18, 8, 2, 12, 9

enum class RelayPin : int {
    GN2_ABORT = 5,
    GN2_FILL = 1,
    PILOT_VENT = 6,
    DOME_PILOT_OPEN = 8,
    RUN = 18,
    FIVE_TWO = 2,
    WATER_SUPPRESSION = 16,
    IGNITER = 17,
};

struct RelayStates {
    bool gn2_abort = false;
    bool gn2_fill = false;
    bool pilot_vent = false;
    bool dome_pilot_open = false;
    bool run = false;
    bool five_two = false;
    bool water_suppression = false;
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

    // igniter turns on at t-7s
    const MS kFireIgniterOffDelayMs = 3500;      // T-3.5s
    const MS kFireDomePilotOpenDelayMs = 4000;   // T-3s
    const MS kFireRunOpenDelayMs = 7000;         // T-0s
    const MS kFireBackToStandbyDelayMs = 17000;  // T+10s

    // safety to make sure we don't hold open solenoids for too long
    // in the CUSTOM state
    const MS kMaxCustomOpenDurationMs = 45000;  // 45s

    FsState cur_state = FsState::STANDBY;

    // time of entering the current state
    MS enter_state_ms = millis();
    // time of entering a state in which we must pulse the pilot valve
    MS enter_pilot_vent_pulse_ms = millis();

    RelayStates relay_states;

   public:
    void Setup() override {
        SetPinToOutput(RelayPin::GN2_ABORT);
        SetPinToOutput(RelayPin::GN2_FILL);
        SetPinToOutput(RelayPin::PILOT_VENT);
        SetPinToOutput(RelayPin::DOME_PILOT_OPEN);
        SetPinToOutput(RelayPin::RUN);
        SetPinToOutput(RelayPin::FIVE_TWO);
        SetPinToOutput(RelayPin::WATER_SUPPRESSION);
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
            case FsCommand::STATE_FIRE_MANUAL_DOME_PILOT_OPEN:
                cur_state = FsState::FIRE_MANUAL_DOME_PILOT_OPEN;
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
            enter_pilot_vent_pulse_ms = millis();
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
                relay_states.gn2_abort = true;
                relay_states.pilot_vent = true;
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
                if (time_in_state < kFireIgniterOffDelayMs) {
                    relay_states.igniter = true;
                } else if (time_in_state < kFireDomePilotOpenDelayMs) {
                    // do nothing; igniter is off
                } else if (time_in_state < kFireRunOpenDelayMs) {
                    relay_states.dome_pilot_open = true;
                } else if (time_in_state < kFireBackToStandbyDelayMs) {
                    relay_states.dome_pilot_open = true;
                    relay_states.run = true;
                    relay_states.five_two = true;
                }
                break;
            case FsState::FIRE_MANUAL_DOME_PILOT_OPEN:
                relay_states.dome_pilot_open = true;
                break;
            case FsState::FIRE_MANUAL_DOME_PILOT_CLOSE:
                relay_states.dome_pilot_open = false;
                break;
            case FsState::FIRE_MANUAL_IGNITER:
                relay_states.igniter = true;
                break;
            case FsState::FIRE_MANUAL_RUN:
                relay_states.run = true;
                relay_states.five_two = true;
                break;
        }

        // set the pilot vent

        if (ShouldPulsePilotVent(cur_state)) {
            MS pulse_pilot_period =
                kPilotValveOpenDurationMs + kPilotValveClosedDurationMs;
            MS time_in_pulse_pilot_period =
                (millis() - enter_pilot_vent_pulse_ms) % pulse_pilot_period;

            relay_states.pilot_vent =
                time_in_pulse_pilot_period < kPilotValveOpenDurationMs;
        }
    }

    void UpdateCustomRelayStates(FsCommandPacket command_packet) {
        relay_states.gn2_abort = command_packet.gn2_abort;
        relay_states.gn2_fill = command_packet.gn2_fill;
        relay_states.pilot_vent = command_packet.pilot_vent;
        relay_states.dome_pilot_open = command_packet.dome_pilot_open;
        relay_states.run = command_packet.run;
        relay_states.five_two = command_packet.five_two;
        relay_states.water_suppression = command_packet.water_suppression;
        relay_states.igniter = command_packet.igniter;
    }

    void FlushRelays() {
        FlushRelay(RelayPin::GN2_ABORT, relay_states.gn2_abort);
        FlushRelay(RelayPin::GN2_FILL, relay_states.gn2_fill);
        FlushRelay(RelayPin::PILOT_VENT, relay_states.pilot_vent);
        FlushRelay(RelayPin::DOME_PILOT_OPEN, relay_states.dome_pilot_open);
        FlushRelay(RelayPin::RUN, relay_states.run);
        FlushRelay(RelayPin::FIVE_TWO, relay_states.five_two);
        FlushRelay(RelayPin::WATER_SUPPRESSION, relay_states.water_suppression);
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
            .gn2_abort = relay_states.gn2_abort,
            .gn2_fill = relay_states.gn2_fill,
            .pilot_vent = relay_states.pilot_vent,
            .dome_pilot_open = relay_states.dome_pilot_open,
            .run = relay_states.run,
            .five_two = relay_states.five_two,
            .water_suppression = relay_states.water_suppression,
            .igniter = relay_states.igniter,
        };

        Send(DeviceType::DevFsLoxGn2Transducers, state_packet);
    }
};

REGISTER_AVIONICS_DEVICE(DevFsRelays);
