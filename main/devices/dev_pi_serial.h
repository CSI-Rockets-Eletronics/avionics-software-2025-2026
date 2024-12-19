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
                Serial2.write((uint8_t*)&gps_packet, sizeof(gps_packet));
                break;
            case 1:
                Serial.println("Received IMU packet");
                Serial2.write((uint8_t*)&imu_packet, sizeof(imu_packet));
                break;
            case 2:
                Serial.println("Received DHT packet");
                Serial2.write((uint8_t*)&dht_packet, sizeof(dht_packet));
                break;
        }

        delay(10);
    }
};

REGISTER_AVIONICS_DEVICE(DevPiSerial);
