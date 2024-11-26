#include <DHT.h>
#include <MPU9255.h>

#include "avionics.h"
#include "packets.h"

static const int kDhtPin = 99;  // TODO

static const bandwidth kAccBandwidth = acc_460Hz;
static const bandwidth kGyroBandwidth = gyro_250Hz;

static const scales kAccScale = scale_16g;
static const scales kGyroScale = scale_2000dps;

class DevDhtImu : public avionics::Device {
   public:
    void Setup() override {
        // dht.begin();

        // if (mpu.init()) {
        //     return Die("MPU9255 init failed");
        // }

        // mpu.set_acc_bandwidth(kAccBandwidth);
        // mpu.set_gyro_bandwidth(kGyroBandwidth);

        // mpu.set_acc_scale(kAccScale);
        // mpu.set_gyro_scale(kGyroScale);
    }

    void Loop() override {
        // TODO
        avionics::PiSerialPacket packet{.msg = "Ping from IMU!"};
        Send(avionics::DeviceType::DevPiSerial, packet);
        Serial.println("Sent message");
    }

   private:
    DHT dht{kDhtPin, DHT11};
    MPU9255 mpu;  // uses default I2C pins
};

REGISTER_AVIONICS_DEVICE(DevDhtImu);
