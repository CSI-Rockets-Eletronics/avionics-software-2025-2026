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
            currentCommand = command_packet.command;
        }

        if (currentCommand == FsCommand::STATE_STANDBY) {
            for (int i = 0; i < 12; i++) {
                if (i == PILOT_VENT) {
                    digitalWrite(kRelayPins[i], HIGH);
                } else {
                    digitalWrite(kRelayPins[i], LOW);
                }
            }
            Serial.println("State: STANDBY");
        } else if (currentCommand == FsCommand::STATE_GN2_STANDBY) {
            digitalWrite(kRelayPins[PILOT_VENT], HIGH);
            for (int i = 0; i < 12; i++) {
                if (i != PILOT_VENT && i != GN2_FILL) {
                    digitalWrite(kRelayPins[i], LOW);
                }
            }
            digitalWrite(kRelayPins[GN2_FILL], HIGH);
            delay(kGN2ValveOpenDurationMs);
            digitalWrite(kRelayPins[GN2_FILL], LOW);
            delay(kGN2ValveClosedDurationMs);
        } 
    }
};

REGISTER_AVIONICS_DEVICE(DevFsRelays);
