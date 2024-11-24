#include <Adafruit_GPS.h>
#include <Arduino.h>

#include "device.h"

const int kRxPin = 99;  // TODO
const int kTxPin = 99;  // TODO

class DevGps : public device::Device {
   public:
    void Init() override {
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
        // TODO
    }

   private:
    HardwareSerial& gpsSerial = Serial1;
    Adafruit_GPS gps{&gpsSerial};
};

DevGps devGps;
