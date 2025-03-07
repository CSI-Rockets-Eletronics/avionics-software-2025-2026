#include "avionics.h"

using namespace avionics;

enum Solenoids {
    GN2_ABORT, 
    GN2_FILL,
    PILOT_VENT,
    DOME_PILOT_OPEN,
    RUN,
    WATER_SUPPRESSION,
    IGNITER
};

class DevFsRelays : public Device {
   private:
    const int kRelayPins[12] = {5, 1, 6, 7, 15, 16, 17, 18, 8, 2, 12, 9};
    
    const int kPilotValveOpenDurationMs = 5000;  
    const int kPilotValveClosedDurationMs = 5000;

    const int kGN2ValveOpenDurationMs = 5000; 
    const int kGN2ValveClosedDurationMs = 5000;

    const int kDelayMs = 1000;

    const unsigned long kFillAPulseDurationMs = 1000;
    const unsigned long kFillBPulseDurationMs = 2000;
    const unsigned long kFillCPulseDurationMs = 3000;

    FsState fsCurrentState = FsState::STANDBY;

    unsigned long lastGN2UpdateTime = 0;
    unsigned long lastPilotUpdateTime = 0; 
    bool gn2PulseState = false;
    bool pilotPulseState = false; 

    unsigned long currentTime = 0;

   public:
    void Setup() override {
        for (int i = 0; i < 12; i++) {
            pinMode(kRelayPins[i], OUTPUT);
        }
    }

    void Loop() override {
        // Implement stuff here 👀

        FsCommandPacket command_packet;
        if (Receive(&command_packet) == 0) {
            switch (command_packet.command) {
                case FsCommand::RESTART:
                    Die("Restarting by command");
                    break;
                case FsCommand::STATE_CUSTOM:
                    fsCurrentState = FsState::CUSTOM;
                    currentTime = millis();
                    break;
                case FsCommand::STATE_ABORT:
                    fsCurrentState = FsState::ABORT;
                    break;
                case FsCommand::STATE_STANDBY:
                    fsCurrentState = FsState::STANDBY;
                    break;
                case FsCommand::STATE_GN2_STANDBY:
                    fsCurrentState = FsState::GN2_STANDBY;
                    currentTime = millis();
                    break;
                case FsCommand::STATE_GN2_FILL:
                    fsCurrentState = FsState::GN2_FILL;
                    currentTime = millis();
                    break;
                case FsCommand::STATE_GN2_PULSE_FILL_A:
                    fsCurrentState = FsState::GN2_PULSE_FILL_A;
                    currentTime = millis();
                    break;
                case FsCommand::STATE_GN2_PULSE_FILL_B:
                    fsCurrentState = FsState::GN2_PULSE_FILL_B;
                    currentTime = millis();
                    break;
                case FsCommand::STATE_GN2_PULSE_FILL_C:
                    fsCurrentState = FsState::GN2_PULSE_FILL_C;
                    currentTime = millis();
                    break;
                case FsCommand::STATE_FIRE:
                    fsCurrentState = FsState::FIRE;
                    currentTime = millis();
                    break;
                case FsCommand::STATE_FIRE_MANUAL_DOME_PILOT_OPEN:
                    fsCurrentState = FsState::FIRE_MANUAL_DOME_PILOT_OPEN;
                    break;
                case FsCommand::STATE_FIRE_MANUAL_DOME_PILOT_CLOSE:
                    fsCurrentState = FsState::FIRE_MANUAL_DOME_PILOT_CLOSE;
                    break;
                case FsCommand::STATE_FIRE_MANUAL_IGNITER:
                    fsCurrentState = FsState::FIRE_MANUAL_IGNITER;
                    break;
                case FsCommand::STATE_FIRE_MANUAL_RUN:
                    fsCurrentState = FsState::FIRE_MANUAL_RUN;
                    break;
                default:
                    break;
            }
        }

        if (fsCurrentState == FsState::ABORT) {
            for (int i = 0; i < 12; i++) {
                if (i == PILOT_VENT || i == GN2_ABORT) {
                    digitalWrite(kRelayPins[i], HIGH);
                } else {
                    digitalWrite(kRelayPins[i], LOW);
                }
            }
            Serial.println("State: ABORT");
        } else if (fsCurrentState == FsState::STANDBY) {
            for (int i = 0; i < 12; i++) {
                digitalWrite(kRelayPins[i], LOW);
            }
            Serial.println("State: STANDBY");
        } else if (fsCurrentState == FsState::GN2_STANDBY ||
                   fsCurrentState == FsState::GN2_FILL ||
                   fsCurrentState == FsState::GN2_PULSE_FILL_A ||
                   fsCurrentState == FsState::GN2_PULSE_FILL_B ||
                   fsCurrentState == FsState::GN2_PULSE_FILL_C) {
            
            //Pulse pilot valve constantly 
            unsigned long pilotInterval = pilotPulseState ? kPilotValveOpenDurationMs : kPilotValveClosedDurationMs;
            if (currentTime - lastPilotUpdateTime >= pilotInterval) {
                pilotPulseState = !pilotPulseState;
                digitalWrite(kRelayPins[PILOT_VENT], pilotPulseState ? HIGH : LOW);
                lastPilotUpdateTime = currentTime;
            }
            for (int i = 0; i < 12; i++) {
                if (i != PILOT_VENT && i != GN2_FILL) {
                    digitalWrite(kRelayPins[i], LOW);
                }
            }
            
            if (fsCurrentState == FsState::GN2_STANDBY) {
                digitalWrite(kRelayPins[GN2_FILL], LOW);
                Serial.println("State: GN2_STANDBY");
            } else if (fsCurrentState == FsState::GN2_FILL) {
                unsigned long GN2interval = gn2PulseState ? kGN2ValveOpenDurationMs : kGN2ValveClosedDurationMs;
                if (currentTime - lastGN2UpdateTime >= GN2interval) {
                    gn2PulseState = !gn2PulseState;
                    digitalWrite(kRelayPins[GN2_FILL], gn2PulseState ? HIGH : LOW);
                    lastGN2UpdateTime = currentTime;
                }
                Serial.println("State: GN2_FILL");
            } else if (fsCurrentState == FsState::GN2_PULSE_FILL_A) {
                static unsigned long pulseStartTimeA = 0;
                if (pulseStartTimeA == 0) {
                    pulseStartTimeA = currentTime;
                    digitalWrite(kRelayPins[GN2_FILL], HIGH);
                    Serial.println("State: GN2_PULSE_FILL_A - Open");
                }
                if (currentTime - pulseStartTimeA >= kFillAPulseDurationMs) {
                    digitalWrite(kRelayPins[GN2_FILL], LOW);
                    pulseStartTimeA = 0;
                    fsCurrentState = FsState::GN2_STANDBY;
                    Serial.println("State: GN2_PULSE_FILL_A complete, switching to GN2_STANDBY");
                }
            } else if (fsCurrentState == FsState::GN2_PULSE_FILL_B) {
                static unsigned long pulseStartTimeB = 0;
                if (pulseStartTimeB == 0) {
                    pulseStartTimeB = currentTime;
                    digitalWrite(kRelayPins[GN2_FILL], HIGH);
                    Serial.println("State: GN2_PULSE_FILL_B - Open");
                }
                if (currentTime - pulseStartTimeB >= kFillBPulseDurationMs) {
                    digitalWrite(kRelayPins[GN2_FILL], LOW);
                    pulseStartTimeB = 0;
                    fsCurrentState = FsState::GN2_STANDBY;
                    Serial.println("State: GN2_PULSE_FILL_B complete, switching to GN2_STANDBY");
                }
            } else if (fsCurrentState == FsState::GN2_PULSE_FILL_C) {
                static unsigned long pulseStartTimeC = 0;
                if (pulseStartTimeC == 0) {
                    pulseStartTimeC = currentTime;
                    digitalWrite(kRelayPins[GN2_FILL], HIGH);
                    Serial.println("State: GN2_PULSE_FILL_C - Open");
                }
                if (currentTime - pulseStartTimeC >= kFillCPulseDurationMs) {
                    digitalWrite(kRelayPins[GN2_FILL], LOW);
                    pulseStartTimeC = 0;
                    fsCurrentState = FsState::GN2_STANDBY;
                    Serial.println("State: GN2_PULSE_FILL_C complete, switching to GN2_STANDBY");
                }
            }
        } else if (fsCurrentState == FsState::FIRE) {
            static unsigned long stateStartTime = millis();
            unsigned long elapsedTime = millis() - stateStartTime;
            if (elapsedTime < 5000) {
                digitalWrite(kRelayPins[DOME_PILOT_OPEN], HIGH);
                Serial.println("State: FIRE - Domepilot valve open");
            } else if (elapsedTime < 23000) {
                digitalWrite(kRelayPins[DOME_PILOT_OPEN], LOW);
                Serial.println("State: FIRE - Waiting");
            } else if (elapsedTime < 29000) {
                digitalWrite(kRelayPins[IGNITER], HIGH);
                Serial.println("State: FIRE - Igniter on");
            } else if (elapsedTime < 30000) {
                digitalWrite(kRelayPins[IGNITER], LOW);
                Serial.println("State: FIRE - Waiting");
            } else if (elapsedTime < 40000) {
                digitalWrite(kRelayPins[RUN], HIGH);
                Serial.println("State: FIRE - Run valve open");
            } else {
                digitalWrite(kRelayPins[RUN], LOW);
                Serial.println("State: FIRE - Run valve closed");
                // Reset stateStartTime
                stateStartTime = millis();
            }
        } else if (fsCurrentState == FsState::FIRE_MANUAL_DOME_PILOT_OPEN) {
            digitalWrite(kRelayPins[DOME_PILOT_OPEN], HIGH);
            Serial.println("State: FIRE_MANUAL_DOME_PILOT_OPEN");
        } else if (fsCurrentState == FsState::FIRE_MANUAL_DOME_PILOT_CLOSE) {
            digitalWrite(kRelayPins[DOME_PILOT_OPEN], LOW);
            Serial.println("State: FIRE_MANUAL_DOME_PILOT_CLOSE");
        } else if (fsCurrentState == FsState::FIRE_MANUAL_IGNITER) {
            digitalWrite(kRelayPins[IGNITER], HIGH);
            Serial.println("State: FIRE_MANUAL_IGNITER");
        } else if (fsCurrentState == FsState::FIRE_MANUAL_RUN) {
            digitalWrite(kRelayPins[RUN], HIGH);
            Serial.println("State: FIRE_MANUAL_RUN");
        } else if (fsCurrentState == FsState::CUSTOM) {
            digitalWrite(kRelayPins[GN2_ABORT], command_packet.gn2_abort ? HIGH : LOW);
            digitalWrite(kRelayPins[GN2_FILL], command_packet.gn2_fill ? HIGH : LOW);
            digitalWrite(kRelayPins[PILOT_VENT], command_packet.pilot_vent ? HIGH : LOW);
            digitalWrite(kRelayPins[DOME_PILOT_OPEN], command_packet.dome_pilot_open ? HIGH : LOW);
            digitalWrite(kRelayPins[RUN], command_packet.run ? HIGH : LOW);
            digitalWrite(kRelayPins[WATER_SUPPRESSION], command_packet.water_suppression ? HIGH : LOW);
            digitalWrite(kRelayPins[IGNITER], command_packet.igniter ? HIGH : LOW);
            Serial.println("State: CUSTOM");
        }
    }
};

REGISTER_AVIONICS_DEVICE(DevFsRelays);
