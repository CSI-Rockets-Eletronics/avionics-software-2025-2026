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

            // relays has the highest priority
            Send(DeviceType::DevFsRelays, *command_packet);
            delay(kSendWaitMs);
            // TODO bug: not sending to FsScientific2
            Send(DeviceType::DevFsInjectorTransducers, *command_packet);
            delay(kSendWaitMs);
            Send(DeviceType::DevFsLoxGn2Transducers, *command_packet);
        } else {
            Serial.println("Unknown packet from Raspberry Pi");
        }
    }

   private:
    // time to allow Send() to complete
    static const int kSendWaitMs = 500;

    static const int kPiSerialRxPin = 18;
    static const int kPiSerialTxPin = 8;

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
