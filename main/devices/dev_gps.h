#include <Adafruit_GPS.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

static const int kRxPin = 99;  // TODO
static const int kTxPin = 99;  // TODO

class DevGps : public Device {
   public:
    void Setup() override {
        // gpsSerial.setPins(kRxPin, kTxPin);
        // gps.begin(9600);
        // // include recommended minimum (RMC) and fix (GGA)
        // gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        // gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
        // // request uupdates on antenna status
        // gps.sendCommand(PGCMD_ANTENNA);

        // delay(1000);
    }

    void Loop() override {
        GpsPacket gps_packet{
            .ts = micros(),
            .fix = 1,
            .fixquality = 1,
            .satellites = 1,
            .latitude_fixed = 0,
            .longitude_fixed = 0,
            .altitude = 0.0,
        };
        Send(DeviceType::DevPiSerial, gps_packet);

        delay(100);
    }

   private:
    HardwareSerial& gpsSerial = Serial1;
    Adafruit_GPS gps{&gpsSerial};
};

REGISTER_AVIONICS_DEVICE(DevGps);
