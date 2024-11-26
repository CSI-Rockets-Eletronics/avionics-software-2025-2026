#include <Adafruit_GPS.h>

#include "avionics.h"
#include "packets.h"

static const int kRxPin = 99;  // TODO
static const int kTxPin = 99;  // TODO

class DevGps : public avionics::Device {
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
        // TODO
        avionics::PiSerialPacket packet{.msg = "Ping from GPS!"};
        Send(avionics::DeviceType::DevPiSerial, packet);
    }

   private:
    HardwareSerial& gpsSerial = Serial1;
    Adafruit_GPS gps{&gpsSerial};
};

REGISTER_AVIONICS_DEVICE(DevGps);
