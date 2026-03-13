#include <MPU9255.h>

#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;

class DevFsPiPacketBroadcaster : public Device {
   public:
    void Setup() override {
        Serial.println("    DevFsPiPacketBroadcaster::Setup() - Starting");
        Serial2.begin(kPiSerialBaud, SERIAL_8N1, kPiSerialRxPin,
                      kPiSerialTxPin);
        Serial.println("    DevFsPiPacketBroadcaster::Setup() - Complete");
    }

    void Loop() override { pi_packet_reader.Tick(); }

    void OnPiPacket(const uint8_t* buffer, size_t size) {
        Serial.print("[PI RX] Received packet from Raspberry Pi, size: ");
        Serial.print(size);
        Serial.print(" bytes, data: 0x");
        for (size_t i = 0; i < min(size, (size_t)8); i++) {
            if (buffer[i] < 0x10) Serial.print("0");
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        if (size == sizeof(FsCommandPacket)) {
            FsCommandPacket* command_packet = (FsCommandPacket*)buffer;
            Serial.print("[PI RX] Broadcasting FsCommandPacket to other nodes, command: ");
            Serial.println(static_cast<int>(command_packet->command));

            // Send to DevFsLoxGn2Transducers (local)
            // This device will forward EREG commands to DevEregControl
            Serial.println("[PI RX] Sending to DevFsLoxGn2Transducers (local)");
            Send(DeviceType::DevFsLoxGn2Transducers, *command_packet);

            // Send to DevFsRelays (ESP-NOW)
            Serial.println("[PI RX] Sending to DevFsRelays (ESP-NOW)");
            Send(DeviceType::DevFsRelays, *command_packet);
            delay(kSendWaitMs);  // wait for ESP-NOW to complete

            // Send to DevFsInjectorTransducers (ESP-NOW)
            Serial.println("[PI RX] Sending to DevFsInjectorTransducers (ESP-NOW)");
            Send(DeviceType::DevFsInjectorTransducers, *command_packet);
            
            Serial.println("[PI RX] Command broadcast complete");
        } else if (size == sizeof(FsEregGainsPacket)) {
            FsEregGainsPacket* gains_packet = (FsEregGainsPacket*)buffer;
            Serial.print("[PI RX] Received FsEregGainsPacket, command: ");
            Serial.println(static_cast<int>(gains_packet->command));
            
            // Forward to DevFsLoxGn2Transducers (local)
            // This device will forward to DevEregControl
            Serial.println("[PI RX] Sending FsEregGainsPacket to DevFsLoxGn2Transducers (local)");
            Send(DeviceType::DevFsLoxGn2Transducers, *gains_packet);
            
            Serial.println("[PI RX] Gains packet forwarded");
        } else {
            Serial.print("[PI RX] Unknown packet from Raspberry Pi (expected ");
            Serial.print(sizeof(FsCommandPacket));
            Serial.print(" bytes for FsCommandPacket or ");
            Serial.print(sizeof(FsEregGainsPacket));
            Serial.println(" bytes for FsEregGainsPacket)");
        }
    }

   private:
    // time to allow Send() to complete
    static const int kSendWaitMs = 500;

    static const int kPiSerialRxPin = 18;  // ESP32 RX <- Pi TX
    static const int kPiSerialTxPin = 8;   // ESP32 TX -> Pi RX

    static const unsigned long kPiSerialBaud = 115200;

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
