#ifndef AV_DEVICE_H_
#define AV_DEVICE_H_

#include <memory>

namespace device {

class Device {
   public:
    Device();
    ~Device();
    virtual void init() = 0;
    virtual void loop() = 0;
};

void InitAll();
void LoopAll();

}  // namespace device

#endif  // AV_DEVICE_H_