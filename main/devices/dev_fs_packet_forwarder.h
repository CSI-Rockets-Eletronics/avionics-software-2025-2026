#include "avionics.h"
#include "packets.h"
#include "utils.h"

using namespace avionics;

// This device runs on fs_scientific1 node and receives telemetry packets
// from AV bay nodes (AVFluids, AVRelays) and ground nodes (FsRelays, FsScientific2)
// via ESP-NOW, then forwards them to the Raspberry Pi via Serial2

class DevFsPacketForwarder : public Device {
   public:
    void Setup() override {
        Serial.println("    DevFsPacketForwarder::Setup() - Starting");
        Serial2.begin(kPiSerialBaud, SERIAL_8N1, kPiSerialRxPin,
                      kPiSerialTxPin);
        Serial.println("    DevFsPacketForwarder::Setup() - Complete");
    }

    void Loop() override {
        // Receive packets from other nodes via ESP-NOW and forward to Pi
        FsLoxGn2TransducersPacket lox_gn2_packet;
        FsInjectorTransducersPacket injector_packet;
        FsStatePacket fs_state_packet;
        AvRelayStatePacket av_relay_packet;
        RelayCurrentMonitorPacket relay_imon_packet;
        FsThermocouplesPacket thermo_packet;

        int received_type = Receive(
            &lox_gn2_packet,
            &injector_packet,
            &fs_state_packet,
            &av_relay_packet,
            &relay_imon_packet,
            &thermo_packet
        );

        switch (received_type) {
            case 0:
                // Received FsLoxGn2TransducersPacket from AVFluids node
                Serial.println("[FORWARDER] Received FsLoxGn2TransducersPacket, forwarding to Pi");
                SendToPi(lox_gn2_packet);
                break;
            case 1:
                // Received FsInjectorTransducersPacket from FsScientific2 node
                Serial.println("[FORWARDER] Received FsInjectorTransducersPacket, forwarding to Pi");
                SendToPi(injector_packet);
                break;
            case 2:
                // Received FsStatePacket from FsRelays node
                Serial.println("[FORWARDER] Received FsStatePacket, forwarding to Pi");
                SendToPi(fs_state_packet);
                break;
            case 3:
                // Received AvRelayStatePacket from AVRelays node
                Serial.println("[FORWARDER] Received AvRelayStatePacket, forwarding to Pi");
                SendToPi(av_relay_packet);
                break;
            case 4:
                // Received RelayCurrentMonitorPacket from relay current monitor
                Serial.println("[FORWARDER] Received RelayCurrentMonitorPacket, forwarding to Pi");
                SendToPi(relay_imon_packet);
                break;
            case 5:
                // Received FsThermocouplesPacket from thermocouples
                Serial.println("[FORWARDER] Received FsThermocouplesPacket, forwarding to Pi");
                SendToPi(thermo_packet);
                break;
            default:
                // No packet received or unrecognized packet
                break;
        }

        freq_logger.Tick();
    }

    template <typename T>
    void SendToPi(const T& data) {
        size_t packet_size = sizeof(data);
        const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&data);

        // Write packet data
        size_t bytes_written = Serial2.write(data_ptr, packet_size);

        // Write delimiters
        size_t delim1_written = Serial2.write(kPacketDelimeter1);
        size_t delim2_written = Serial2.write(kPacketDelimeter2);

        // Verify all bytes were written
        if (bytes_written != packet_size) {
            Serial.print("[FORWARDER PI TX ERROR] Only wrote ");
            Serial.print(bytes_written);
            Serial.print("/");
            Serial.print(packet_size);
            Serial.println(" bytes!");
        } else if (delim1_written != 1 || delim2_written != 1) {
            Serial.println("[FORWARDER PI TX ERROR] Failed to write delimiters!");
        }
    }

   private:
    static const int kPiSerialRxPin = 18;  // ESP32 RX <- Pi TX
    static const int kPiSerialTxPin = 8;   // ESP32 TX -> Pi RX

    static const unsigned long kPiSerialBaud = 115200;

    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    // just VS code intellisense being dumb; Serial2 is accessible globally
    HardwareSerial Serial2{2};

    utils::FrequencyLogger freq_logger{"PacketForwarder"};
};

REGISTER_AVIONICS_DEVICE(DevFsPacketForwarder);
