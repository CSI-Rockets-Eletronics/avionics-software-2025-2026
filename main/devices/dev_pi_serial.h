#include "avionics.h"
#include "packets.h"

using namespace avionics;

const int kPiSerialRxPin = 47;
const int kPiSerialTxPin = 48;

const unsigned long kPiSerialBaud = 230400;
const uint8_t kPiPacketDelimeter[] = {0b10101010, 0b01010101};

class DevPiSerial : public Device {
   public:
    void Setup() override {
        // for raspberry pi
        Serial2.begin(kPiSerialBaud, SERIAL_8N1, kPiSerialRxPin,
                      kPiSerialTxPin);
    }

    void Loop() override {
        GpsPacket gps_packet;
        ImuPacket imu_packet;
        DhtPacket dht_packet;

        switch (Receive(&gps_packet, &imu_packet, &dht_packet)) {
            case 0:
                Serial.println("Received GPS packet");
                SendToPi(gps_packet);
                break;
            case 1:
                Serial.println("Received IMU packet");
                Serial.print("ax: ");
                Serial.print(imu_packet.ax);
                Serial.print("\tay: ");
                Serial.print(imu_packet.ay);
                Serial.print("\taz: ");
                Serial.print(imu_packet.az);
                Serial.print("\tgx: ");
                Serial.print(imu_packet.gx);
                Serial.print("\tgy: ");
                Serial.print(imu_packet.gy);
                Serial.print("\tgz: ");
                Serial.println(imu_packet.gz);

                SendToPi(imu_packet);
                break;
            case 2:
                Serial.println("Received DHT packet");
                SendToPi(dht_packet);
                break;
        }

        delay(10);
    }

    template <typename T>
    void SendToPi(const T& data) {
        Serial2.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
        Serial2.write(kPiPacketDelimeter, sizeof(kPiPacketDelimeter));
    }
};

REGISTER_AVIONICS_DEVICE(DevPiSerial);
