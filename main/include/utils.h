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

class SerialPacketReader {
   public:
    SerialPacketReader(HardwareSerial& serial,
                       std::function<void(uint8_t*, size_t)> packet_handler)
        : serial(serial), packet_handler(packet_handler) {}

    void Tick() {
        while (serial.available()) {
            // if write would overflow, clear the buffer
            if (bufferIndex >= kMaxPacketSize) {
                bufferIndex = 0;
            }

            buffer[bufferIndex] = serial.read();
            bufferIndex++;
            // here, bufferIndex is at most kMaxPacketSize
        }

        if (bufferIndex >= 2 && buffer[bufferIndex - 2] == kPacketDelimeter1 &&
            buffer[bufferIndex - 1] == kPacketDelimeter2) {
            packet_handler(buffer, bufferIndex - 2);  // excludes the delimeters
            bufferIndex = 0;
        }
    }

   private:
    static const size_t kMaxPacketSize = 256;

    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    HardwareSerial& serial;
    std::function<void(uint8_t*, size_t)> packet_handler;

    uint8_t buffer[kMaxPacketSize];
    size_t bufferIndex = 0;  // also equals the number of bytes in the buffer
};

class SerialForwarder {
   public:
    SerialForwarder(std::string label, HardwareSerial& from, HardwareSerial& to)
        : from(from), to(to), freq_logger(label) {}

    void Tick() {
        while (from.available()) {
            // if write would overflow, clear the buffer
            if (bufferIndex >= kMaxPacketSize) {
                bufferIndex = 0;
            }

            buffer[bufferIndex] = from.read();
            bufferIndex++;
            // here, bufferIndex is at most kMaxPacketSize
        }

        if (bufferIndex >= 2 && buffer[bufferIndex - 2] == kPacketDelimeter1 &&
            buffer[bufferIndex - 1] == kPacketDelimeter2) {
            // Debug: print packet details
            Serial.print("[SERIAL FWD] Forwarding packet, size: ");
            Serial.print(bufferIndex - 2);  // exclude delimiters
            Serial.print(" bytes, data: 0x");
            for (size_t i = 0; i < min((size_t)8, bufferIndex - 2); i++) {
                if (buffer[i] < 0x10) Serial.print("0");
                Serial.print(buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            to.write(buffer, bufferIndex);  // includes the delimeters
            bufferIndex = 0;

            freq_logger.Tick();
        }
    }

   private:
    static const size_t kMaxPacketSize = 256;

    static const uint8_t kPacketDelimeter1 = 0b10101010;
    static const uint8_t kPacketDelimeter2 = 0b01010101;

    HardwareSerial& from;
    HardwareSerial& to;

    FrequencyLogger freq_logger;

    uint8_t buffer[kMaxPacketSize];
    size_t bufferIndex = 0;  // also equals the number of bytes in the buffer
};

}  // namespace utils

#endif  // UTILS_H_
