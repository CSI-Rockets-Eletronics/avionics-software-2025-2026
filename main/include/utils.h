#ifndef UTILS_H_
#define UTILS_H_

#include <Arduino.h>

#include <string>

namespace utils {

class FrequencyLogger {
   public:
    FrequencyLogger(const std::string label,
                    unsigned long printIntervalMs = 1000)
        : label(label),
          printIntervalMs(printIntervalMs),
          lastPrintTime(millis()),
          tickCount(0) {}

    void tick() {
        tickCount++;

        if (millis() - lastPrintTime > printIntervalMs) {
            Serial.print("[");
            Serial.print(label.c_str());
            Serial.print("] Frequency: ");
            Serial.print((unsigned long)(tickCount * 1000.0 / printIntervalMs));
            Serial.println(" Hz");
            lastPrintTime = millis();
            tickCount = 0;
        }
    }

   private:
    const std::string label;
    const unsigned long printIntervalMs;

    unsigned long lastPrintTime;
    unsigned long tickCount;
};

}  // namespace utils

#endif  // UTILS_H_
