#include "avionics.h"
#include "packets.h"

class DevPiSerial : public avionics::Device {
   public:
    void Setup() override {
        // TODO
    }

    void Loop() override {
        // TODO

        avionics::PiSerialPacket packet;
        if (!Receive(&packet)) {
            delay(10);
            return;
        }

        // Serial.print("Received message: ");
        // Serial.println(packet.msg);

        recv_count++;
        if (recv_count == profile_interval) {
            long now = millis();
            long elapsed = now - last_profile_time;
            last_profile_time = now;
            recv_count = 0;

            float hz = 1000.0 / elapsed * profile_interval;
            Serial.print("PiSerial hz: ");
            Serial.println(hz);
        }
    }

   private:
    // TODO
    long profile_interval = 100;
    long last_profile_time = 0;
    long recv_count = 0;
};

REGISTER_AVIONICS_DEVICE(DevPiSerial);
