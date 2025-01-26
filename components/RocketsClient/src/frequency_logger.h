#ifndef FREQUENCY_LOGGER_H_
#define FREQUENCY_LOGGER_H_

#include <Arduino.h>

class FrequencyLogger {
   public:
    FrequencyLogger(String label, unsigned long printIntervalMs)
        : label(label),
          printIntervalMs(printIntervalMs),
          lastPrintTime(millis()),
          tickCount(0) {}

    void tick() {
        tickCount++;

        if (millis() - lastPrintTime > printIntervalMs) {
            Serial.print("[");
            Serial.print(label);
            Serial.print("] Frequency: ");
            Serial.print((unsigned long)(tickCount * 1000.0 / printIntervalMs));
            Serial.println(" Hz");
            lastPrintTime = millis();
            tickCount = 0;
        }
    }

   private:
    String label;
    unsigned long printIntervalMs;
    unsigned long lastPrintTime;
    unsigned long tickCount;
};

#endif  // FREQUENCY_LOGGER_H_
