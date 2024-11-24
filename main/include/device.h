#ifndef AV_DEVICE_H_
#define AV_DEVICE_H_

#include <memory>

namespace device {

class Device {
   public:
    Device();
    ~Device();
    virtual void Setup() = 0;
    void LoopIfSetupSucceeded();

   protected:
    virtual void Loop() = 0;
    void SetupDie(std::string msg);

   private:
    bool setupFailed = false;
};

void SetupAll();
void LoopAll();

}  // namespace device

#endif  // AV_DEVICE_H_
