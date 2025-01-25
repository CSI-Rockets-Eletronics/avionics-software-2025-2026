#include <Adafruit_GPS.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

class DevGps : public Device {
   private:
    // note: the txPin arg for setPins() is actually the RX pin, and vice versa
    static const int kRxPin = 12;
    static const int kTxPin = 13;

    HardwareSerial& gpsSerial = Serial1;
    Adafruit_GPS gps{&gpsSerial};

   public:
    void Setup() override {
        gpsSerial.setPins(kRxPin, kTxPin);
        gps.begin(9600);
        // include recommended minimum (RMC) and fix (GGA)
        gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        gps.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
        // request uupdates on antenna status
        gps.sendCommand(PGCMD_ANTENNA);

        delay(1000);
    }

    void Loop() override {
        gps.read();

        if (gps.newNMEAreceived()) {
            if (!gps.parse(gps.lastNMEA())) {
                Serial.println("Failed to parse GPS data");
                return;
            }

            GpsPacket gps_packet{
                .ts = micros(),
                .fix = gps.fix,
                .fixquality = gps.fixquality,
                .satellites = gps.satellites,
                .latitude_fixed = gps.latitude_fixed,
                .longitude_fixed = gps.longitude_fixed,
                .altitude = gps.altitude,
            };
            Send(DeviceType::DevPiSerial, gps_packet);
        }
    }
};

REGISTER_AVIONICS_DEVICE(DevGps);
