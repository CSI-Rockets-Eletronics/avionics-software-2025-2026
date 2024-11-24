#ifndef AV_DEVICE_H_
#define AV_DEVICE_H_

#include <memory>

namespace device {

class Device {
   public:
    Device();
    ~Device();
    virtual void Init() = 0;
    void LoopIfInit();

   protected:
    virtual void Loop() = 0;
    void InitDie(std::string msg);

   private:
    bool init_failed_ = false;
};

void InitAll();
void LoopAll();

}  // namespace device

#endif  // AV_DEVICE_H_