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
}

class DevFsRelays : public Device {
   private:
    const int kRelayPins[12] = {5, 1, 6, 7, 15, 16, 17, 18, 8, 2, 12, 9};
    
    const int kPilotValveOpenDurationMs = 5000;  
    const int kPilotValveClosedDurationMs = 5000;

    const int kGN2ValveOpenDurationMs = 5000; 
    const int kGN2ValveClosedDurationMs = 5000;

    const int kDelayMs = 1000;

    FsCommand currentCommand = FsCommand::STATE_STANDBY;

    unsigned long lastUpdateTime = 0;
    bool gn2PulseState = false;

   public:
    void Setup() override {
        for (int i = 0; i < 12; i++) {
            pinMode(kRelayPins[i], OUTPUT);
        }
    }

    void Loop() override {
        for (int i = 0; i < 12; i++) {
            digitalWrite(kRelayPins[i], HIGH);
        }
        delay(kDelayMs);
        Serial.println("High");

        for (int i = 0; i < 12; i++) {
            digitalWrite(kRelayPins[i], LOW);
        }
        delay(kDelayMs);
        Serial.println("Low");

        // Implement stuff here 👀

        FsCommandPacket command_packet;
        if (Receive(&command_packet) == 0) {
            if (command_packet.command == FsCommand::RESTART) {
                Die("Restarting by command");
            }
        }

        if (currentCommand == FsCommand::STATE_ABORT) {
            for (int i = 0; i < 12; i++) {
                if (i == PILOT_VENT || i == GN2_ABORT) {
                    digitalWrite(kRelayPins[i], HIGH);
                } else {
                    digitalWrite(kRelayPins[i], LOW);
                }
            }
            Serial.println("State: ABORT");
        } else if (currentCommand == FsCommand::STATE_STANDBY) {
            digitalWrite(kRelayPins[PILOT_VENT], HIGH);
            for (int i = 0; i < 12; i++) {
                if (i != PILOT_VENT && i != GN2_FILL) {
                    digitalWrite(kRelayPins[i], LOW);
                }
            }
            Serial.println("State: STANDBY");
        } else if (currentCommand == FsCommand::STATE_GN2_STANDBY ||
                   currentCommand == FsCommand::STATE_GN2_FILL ||
                   currentCommand == FsCommand::STATE_GN2_PULSE_FILL_A ||
                   currentCommand == FsCommand::STATE_GN2_PULSE_FILL_B ||
                   currentCommand == FsCommand::STATE_GN2_PULSE_FILL_C) {
            
            unsigned long currentTime = millis();

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
            
            if (currentCommand == FsCommand::STATE_GN2_STANDBY) {
                digitalWrite(kRelayPins[GN2_FILL], LOW);
                Serial.println("State: GN2_STANDBY");
            } else if (currentCommand == FsCommand::STATE_GN2_FILL) {
                unsigned long GN2interval = gn2PulseState ? kGN2ValveOpenDurationMs : kGN2ValveClosedDurationMs;
                if (currentTime - lastUpdateTime >= GN2interval) {
                    gn2PulseState = !gn2PulseState;
                    digitalWrite(kRelayPins[GN2_FILL], gn2PulseState ? HIGH : LOW);
                    lastUpdateTime = currentTime;
                }
                Serial.println("State: GN2_FILL");
            } else if (currentCommand == FsCommand::STATE_GN2_PULSE_FILL_A) {
                const unsigned long fillAPulseOn = 1000;
                const unsigned long fillAPulseOff = 2000;
                static unsigned long lastFillAPulseTime = 0;
                static bool fillAPulseState = false;
                unsigned long intervalA = fillAPulseState ? fillAPulseOn : fillAPulseOff;
                if (currentTime - lastFillAPulseTime >= intervalA) {
                    fillAPulseState = !fillAPulseState;
                    digitalWrite(kRelayPins[GN2_FILL], fillAPulseState ? HIGH : LOW);
                    lastFillAPulseTime = currentTime;
                }
                Serial.println("State: GN2_PULSE_FILL_A");
            } else if (currentCommand == FsCommand::STATE_GN2_PULSE_FILL_B) {
                const unsigned long fillBPulseOn = 2000;
                const unsigned long fillBPulseOff = 2000;
                static unsigned long lastFillBPulseTime = 0;
                static bool fillBPulseState = false;
                unsigned long intervalB = fillBPulseState ? fillBPulseOn : fillBPulseOff;
                if (currentTime - lastFillBPulseTime >= intervalB) {
                    fillBPulseState = !fillBPulseState;
                    digitalWrite(kRelayPins[GN2_FILL], fillBPulseState ? HIGH : LOW);
                    lastFillBPulseTime = currentTime;
                }
                Serial.println("State: GN2_PULSE_FILL_B");
            } else if (currentCommand == FsCommand::STATE_GN2_PULSE_FILL_C) {
                const unsigned long fillCPulseOn = 3000;
                const unsigned long fillCPulseOff = 3000;
                static unsigned long lastFillCPulseTime = 0;
                static bool fillCPulseState = false;
                unsigned long intervalC = fillCPulseState ? fillCPulseOn : fillCPulseOff;
                if (currentTime - lastFillCPulseTime >= intervalC) {
                    fillCPulseState = !fillCPulseState;
                    digitalWrite(kRelayPins[GN2_FILL], fillCPulseState ? HIGH : LOW);
                    lastFillCPulseTime = currentTime;
                }
                Serial.println("State: GN2_PULSE_FILL_C");
            }
        } else if (currentCommand == FsCommand::STATE_FIRE) {
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
        } else if (currentCommand == FsCommand::STATE_FIRE_MANUAL_DOME_PILOT_OPEN) {
            digitalWrite(kRelayPins[DOME_PILOT_OPEN], HIGH);
            Serial.println("State: FIRE_MANUAL_DOME_PILOT_OPEN");
        } else if (currentCommand == FsCommand::STATE_FIRE_MANUAL_DOME_PILOT_CLOSE) {
            digitalWrite(kRelayPins[DOME_PILOT_OPEN], LOW);
            Serial.println("State: FIRE_MANUAL_DOME_PILOT_CLOSE");
        } else if (currentCommand == FsCommand::STATE_FIRE_MANUAL_IGNITER) {
            digitalWrite(kRelayPins[IGNITER], HIGH);
            Serial.println("State: FIRE_MANUAL_IGNITER");
        } else if (currentCommand == FsCommand::STATE_FIRE_MANUAL_RUN) {
            digitalWrite(kRelayPins[RUN], HIGH);
            Serial.println("State: FIRE_MANUAL_RUN");
        } else if (currentCommand == FsCommand::STATE_CUSTOM) {
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
