#ifndef UTILS_H_
#define UTILS_H_

#include <Arduino.h>

#include <string>

namespace utils {

class FrequencyLogger {
   public:
    FrequencyLogger(const std::string label,
                    unsigned long print_interval_ms = 1000)
        : label(label),
          print_interval_ms(print_interval_ms),
          last_print_time(millis()),
          tick_count(0) {}

    void Tick() {
        tick_count++;

        if (millis() - last_print_time > print_interval_ms) {
            Serial.print("[");
            Serial.print(label.c_str());
            Serial.print("] Frequency: ");
            Serial.print(
                (unsigned long)(tick_count * 1000.0 / print_interval_ms));
            Serial.println(" Hz");
            last_print_time = millis();
            tick_count = 0;
        }
    }

   private:
    const std::string label;
    const unsigned long print_interval_ms;

    unsigned long last_print_time;
    unsigned long tick_count;
};

}  // namespace utils

#endif  // UTILS_H_
