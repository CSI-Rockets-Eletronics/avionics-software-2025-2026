#include <MPU9255.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;

class DevFsPiPacketBroadcaster : public Device {
   public:
    void Setup() override {
        Serial2.begin(kPiSerialBaud, SERIAL_8N1, kPiSerialRxPin,
                      kPiSerialTxPin);
    }

    void Loop() override { pi_packet_reader.Tick(); }

    void OnPiPacket(const uint8_t* buffer, size_t size) {
        if (size == sizeof(FsCommandPacket)) {
            FsCommandPacket* command_packet = (FsCommandPacket*)buffer;

            Send(DeviceType::DevFsRelays, *command_packet);
            Send(DeviceType::DevFsInjectorTransducers, *command_packet);
            Send(DeviceType::DevFsLoxGn2Transducers, *command_packet);
        } else {
            Serial.println("Unknown packet from Raspberry Pi");
        }
    }

   private:
    static const int kPiSerialRxPin = 40;
    static const int kPiSerialTxPin = 39;

    static const unsigned long kPiSerialBaud = 230400;

    // just VS code intellisense being dumb; Serial2 is accessible globally
    HardwareSerial Serial2{2};

    // assumes Serial2 is initialized elsewhere to communicate with the
    // Raspberry Pi
    utils::SerialPacketReader pi_packet_reader{
        Serial2, [this](const uint8_t* buffer, size_t size) {
            OnPiPacket(buffer, size);
        }};
};

REGISTER_AVIONICS_DEVICE(DevFsPiPacketBroadcaster);
