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
                got_gps = true;

                radio_packet.gps_ts_tail = gps_packet.ts & 0xFF;
                radio_packet.gps_fix = gps_packet.fix;
                radio_packet.gps_fixquality = gps_packet.fixquality;
                radio_packet.gps_satellites = gps_packet.satellites;
                radio_packet.gps_latitude_fixed = gps_packet.latitude_fixed;
                radio_packet.gps_longitude_fixed = gps_packet.longitude_fixed;
                radio_packet.gps_altitude = gps_packet.altitude;

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
                got_imu = true;

                radio_packet.imu_gz = imu_packet.gz;

                break;
            case 2:
                Serial.println("Received DHT packet");
                SendToPi(dht_packet);
                break;

            default:
                // yield and wait for more data
                delay(10);
                return;
        }

        if (got_gps && got_imu) {
            Send(DeviceType::DevRadio, radio_packet);
        }
    }

    template <typename T>
    void SendToPi(const T& data) {
        Serial2.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
        Serial2.write(kPiPacketDelimeter, sizeof(kPiPacketDelimeter));
    }

   private:
    RadioPacket radio_packet;
    // don't send first packet until we have both
    bool got_gps = false;
    bool got_imu = false;
};

REGISTER_AVIONICS_DEVICE(DevPiSerial);
