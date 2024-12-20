#include <MPU9255.h>

#include "avionics.h"
#include "packets.h"

using namespace avionics;

static const bandwidth kAccBandwidth = acc_460Hz;
static const bandwidth kGyroBandwidth = gyro_250Hz;

static const scales kAccScale = scale_16g;
static const scales kGyroScale = scale_2000dps;

class DevImu : public Device {
   public:
    void Setup() override {
        if (mpu.init()) {
            Die("MPU9255 init failed");
        }

        // save power
        mpu.disable(magnetometer);
        mpu.disable(thermometer);

        mpu.set_acc_bandwidth(kAccBandwidth);
        mpu.set_gyro_bandwidth(kGyroBandwidth);

        mpu.set_acc_scale(kAccScale);
        mpu.set_gyro_scale(kGyroScale);
    }

    void Loop() override {
        mpu.read_acc();
        mpu.read_gyro();

        ImuPacket imu_packet{
            .ts = micros(),
            .ax = mpu.ax,
            .ay = mpu.ay,
            .az = mpu.az,
            .gx = mpu.gx,
            .gy = mpu.gy,
            .gz = mpu.gz,
        };
        Send(DeviceType::DevPiSerial, imu_packet);
    }

   private:
    MPU9255 mpu;  // uses default I2C pins
};

REGISTER_AVIONICS_DEVICE(DevImu);
