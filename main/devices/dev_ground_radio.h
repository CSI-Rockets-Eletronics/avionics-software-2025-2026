#include <RH_RF95.h>

#include "avionics.h"
#include "packets.h"
#include "rockets_client.h"

using namespace avionics;

class DevGroundRadio : public Device {
   private:
    static const int kInterruptPin = 2;

    static const int kFrequency = 433;
    static const int kTxPower = 20;  // max power

    const rockets_client::WifiConfig kWifiConfig{
        .ssid = "Columbia University",
        .password = "",
    };
    const rockets_client::ServerConfig kServerConfig{
        .host = "csiwiki.me.columbia.edu",
        .port = 3001,
        .pathPrefix = "/rocketsdata2",
    };

    const char* kEnvironmentKey = "0";
    const char* kDeviceName = "RadioGround";

    RH_RF95 rf95{SS, kInterruptPin};  // uses default SPI pins

   public:
    void Setup() override {
        if (!rf95.init()) {
            return Die("RF95 init failed");
        }
        if (!rf95.setFrequency(kFrequency)) {
            return Die("RF95 setFrequency failed");
        }
        rf95.setTxPower(kTxPower, false);

        RocketsClientInit();
    }

    void Loop() override {
        rf95.waitAvailable(10);

        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (!rf95.recv(buf, &len)) {
            Serial.println("RF95 recv failed");
            return;
        }

        if (len != sizeof(RadioPacket)) {
            Serial.println("RF95 recv len mismatch");
            return;
        }

        RadioPacket* radio_packet = (RadioPacket*)buf;

        Serial.print("gps_ts_tail: ");
        Serial.print(radio_packet->gps_ts_tail);
        Serial.print("\tfix: ");
        Serial.print(radio_packet->gps_fix);
        Serial.print("\tlat: ");
        Serial.print(radio_packet->gps_latitude_fixed);
        Serial.print("\tlon: ");
        Serial.print(radio_packet->gps_longitude_fixed);
        Serial.print("\taz: ");
        Serial.println(radio_packet->imu_az);

        RocketsClientQueueRecord(radio_packet);
    }

   private:
    void RocketsClientInit() {
        rockets_client::init(kWifiConfig, kServerConfig, kEnvironmentKey,
                             kDeviceName);
    }

    void RocketsClientQueueRecord(RadioPacket* rp) {
        rockets_client::StaticJsonDoc recordData;

        recordData["gps_ts_tail"] = rp->gps_ts_tail;
        recordData["gps_fix"] = rp->gps_fix != 0;  // convert to bool

        if (rp->gps_fix) {
            recordData["gps_fixquality"] = rp->gps_fixquality;
            recordData["gps_satellites"] = rp->gps_satellites;
            recordData["gps_latitude_fixed"] = rp->gps_latitude_fixed;
            recordData["gps_longitude_fixed"] = rp->gps_longitude_fixed;
            recordData["gps_altitude"] = rp->gps_altitude;
        }

        recordData["imu_az"] = rp->imu_az;

        rockets_client::queueRecord(recordData);
    }
};

REGISTER_AVIONICS_DEVICE(DevGroundRadio);
