#include "avionics.h"
#include "packets.h"

class DevPiSerial : public avionics::Device {
   public:
    void Setup() override {
        // TODO
    }

    void Loop() override {
        // TODO
    }

    void OnReceive(uint8_t* bytes, size_t len) override {
        avionics::PiSerialPacket packet;
        if (!Parse(bytes, len, &packet)) return;

        Serial.print("Received message: ");
        Serial.println(packet.msg);
    }

   private:
    // TODO
};

REGISTER_AVIONICS_DEVICE(DevPiSerial);
