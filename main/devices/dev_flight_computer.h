#pragma once

#include "avionics.h"
#include "packets.h"
#include <SPI.h>
#include <Wire.h>

// Sensor libraries
#include <Adafruit_BMP3XX.h>      // Barometer (I2C or SPI)
#include <Adafruit_MPU6050.h>     // IMU (I2C) - or use MPU9250
#include <Adafruit_GPS.h>         // GPS
#include <SPIMemory.h>            // Flash memory chip

using namespace avionics;

class DevFlightComputer : public Device {
private:
    // ==================== HARDWARE CONFIGURATION ====================
    
    // SPI Chip Select Pins
    static const int kBmpCsPin = 5;      // Barometer CS
    static const int kImuCsPin = 15;     // IMU CS (if using SPI version)
    static const int kFlashCsPin = 4;    // Flash memory CS
    
    // GPS Serial
    static const int kGpsRxPin = 16;
    static const int kGpsTxPin = 17;
    
    // Pyro Channels
    static const int kDroguePyroPin = 25;
    static const int kMainPyroPin = 26;
    static const int kPyroContinuityDroguePin = 32;  // Check if pyro connected
    static const int kPyroContinuityMainPin = 33;
    
    // Buzzer for audio feedback
    static const int kBuzzerPin = 27;
    
    // ==================== FLIGHT STATE MACHINE ====================
    
    enum FlightState {
        PAD_IDLE,           // On pad, waiting for launch
        POWERED_ASCENT,     // Motor burning
        COASTING,           // Motor burned out, going up
        APOGEE_DETECTED,    // Peak altitude reached
        DROGUE_DEPLOYED,    // Drogue chute out
        MAIN_DEPLOYED,      // Main chute out
        LANDED              // On ground
    };
    
    FlightState current_state_ = PAD_IDLE;
    FlightState last_state_ = PAD_IDLE;
    
    // ==================== SENSOR OBJECTS ====================
    
    Adafruit_BMP3XX bmp_;           // Barometer (SPI)
    Adafruit_MPU6050 mpu_;          // IMU (I2C)
    HardwareSerial gpsSerial_;      // GPS UART
    Adafruit_GPS gps_;              // GPS parser
    SPIFlash flash_;                // Flash memory
    
    // ==================== SENSOR DATA ====================
    
    struct SensorData {
        // Barometer
        float pressure_pa;
        float altitude_m;
        float temperature_c;
        
        // IMU
        float accel_x, accel_y, accel_z;  // m/s^2
        float gyro_x, gyro_y, gyro_z;     // rad/s
        float temp_imu;
        
        // Derived
        float vertical_accel_g;  // In G's, along rocket axis
        float vertical_velocity_ms;  // m/s
        
        // GPS
        bool gps_fix;
        uint8_t gps_satellites;
        float gps_latitude;
        float gps_longitude;
        float gps_altitude_m;
        float gps_speed_mps;
        
        // Orientation (calculated from IMU)
        float pitch_deg;
        float roll_deg;
        float yaw_deg;
        
        // Timing
        unsigned long timestamp_ms;
    };
    
    SensorData current_data_;
    
    // ==================== FLIGHT PARAMETERS ====================
    
    // Launch detection
    static constexpr float kLaunchAccelThresholdG = 3.0;  // 3G acceleration
    static constexpr int kLaunchConfirmCount = 5;  // Consecutive readings
    int launch_confirm_counter_ = 0;
    
    // Apogee detection
    static constexpr float kApogeeDescentThresholdM = 3.0;  // 3m descent
    static constexpr int kApogeeConfirmCount = 5;
    int apogee_confirm_counter_ = 0;
    
    // Main deployment
    static constexpr float kMainDeployAltitudeM = 152.4;  // 500 feet = 152.4m
    
    // Landing detection
    static constexpr float kLandingVelocityThreshold = 2.0;  // m/s
    static constexpr int kLandingConfirmCount = 10;
    int landing_confirm_counter_ = 0;
    
    // Pyro firing
    static constexpr int kPyroDurationMs = 1000;  // 1 second
    
    // ==================== FLIGHT METRICS ====================
    
    unsigned long launch_time_ms_ = 0;
    unsigned long apogee_time_ms_ = 0;
    unsigned long landing_time_ms_ = 0;
    
    float max_altitude_m_ = 0;
    float max_velocity_ms_ = 0;
    float max_acceleration_g_ = 0;
    
    float ground_altitude_m_ = 0;  // Calibrated ground level
    float launch_altitude_m_ = 0;
    
    // Altitude filtering for apogee detection
    static const int kAltitudeBufferSize = 20;
    float altitude_buffer_[kAltitudeBufferSize] = {0};
    int altitude_buffer_idx_ = 0;
    
    // Velocity estimation (simple differentiation)
    float last_altitude_m_ = 0;
    unsigned long last_altitude_time_ms_ = 0;
    
    // ==================== DATA LOGGING ====================
    
    static const int kFlashPageSize = 256;
    static const int kFlashSectorSize = 4096;
    uint32_t current_flash_address_ = 0;
    uint32_t log_entry_count_ = 0;
    
    struct FlightLogEntry {
        unsigned long timestamp_ms;
        float altitude_m;
        float velocity_ms;
        float accel_g;
        float pressure_pa;
        float gps_lat;
        float gps_lon;
        uint8_t state;
        uint8_t gps_sats;
    } __attribute__((packed));
    
    // ==================== TELEMETRY ====================
    
    unsigned long last_telemetry_ms_ = 0;
    static constexpr int kTelemetryIntervalMs = 100;  // 10Hz to ground
    
    unsigned long last_log_ms_ = 0;
    static constexpr int kLogIntervalMs = 50;  // 20Hz logging
    
public:
    void Setup() override {
        Serial.println("=== Flight Computer Initializing ===");
        
        // Initialize SPI
        SPI.begin();
        
        // Initialize I2C
        Wire.begin();
        
        // Setup GPIO
        pinMode(kDroguePyroPin, OUTPUT);
        pinMode(kMainPyroPin, OUTPUT);
        pinMode(kPyroContinuityDroguePin, INPUT_PULLUP);
        pinMode(kPyroContinuityMainPin, INPUT_PULLUP);
        pinMode(kBuzzerPin, OUTPUT);
        
        digitalWrite(kDroguePyroPin, LOW);
        digitalWrite(kMainPyroPin, LOW);
        digitalWrite(kBuzzerPin, LOW);
        
        // Initialize Barometer (SPI)
        if (!InitBarometer()) {
            Die("Barometer init failed!");
        }
        
        // Initialize IMU (I2C)
        if (!InitIMU()) {
            Die("IMU init failed!");
        }
        
        // Initialize GPS
        if (!InitGPS()) {
            Die("GPS init failed!");
        }
        
        // Initialize Flash Memory
        if (!InitFlash()) {
            Die("Flash memory init failed!");
        }
        
        // Calibrate ground level
        CalibrateGroundLevel();
        
        // Check pyro continuity
        CheckPyroContinuity();
        
        // Startup beep sequence
        StartupBeeps();
        
        Serial.println("=== Flight Computer Ready ===");
    }
    
    void Loop() override {
        // Read all sensors
        ReadBarometer();
        ReadIMU();
        ReadGPS();
        
        // Update derived values
        UpdateDerivedValues();
        
        // Run flight state machine
        UpdateFlightState();
        
        // Log data to flash
        if (millis() - last_log_ms_ >= kLogIntervalMs) {
            LogDataToFlash();
            last_log_ms_ = millis();
        }
        
        // Send telemetry to ground station
        if (millis() - last_telemetry_ms_ >= kTelemetryIntervalMs) {
            SendTelemetry();
            last_telemetry_ms_ = millis();
        }
        
        // Audio feedback based on state
        UpdateBuzzer();
        
        delay(10);  // ~100Hz main loop
    }
    
private:
    // ==================== SENSOR INITIALIZATION ====================
    
    bool InitBarometer() {
        Serial.print("Initializing BMP388 barometer (SPI)... ");
        
        if (!bmp_.begin_SPI(kBmpCsPin)) {
            Serial.println("FAILED");
            return false;
        }
        
        // Configure oversampling and filtering
        bmp_.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp_.setPressureOversampling(BMP3_OVERSAMPLING_32X);
        bmp_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp_.setOutputDataRate(BMP3_ODR_100_HZ);
        
        Serial.println("OK");
        return true;
    }
    
    bool InitIMU() {
        Serial.print("Initializing MPU6050 IMU (I2C)... ");
        
        if (!mpu_.begin()) {
            Serial.println("FAILED");
            return false;
        }
        
        // Configure IMU
        mpu_.setAccelerometerRange(MPU6050_RANGE_16_G);
        mpu_.setGyroRange(MPU6050_RANGE_2000_DEG);
        mpu_.setFilterBandwidth(MPU6050_BAND_94_HZ);
        
        Serial.println("OK");
        return true;
    }
    
    bool InitGPS() {
        Serial.print("Initializing GPS... ");
        
        gpsSerial_.begin(9600, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
        gps_.begin(&gpsSerial_);
        
        // Request RMC and GGA sentences
        gps_.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        gps_.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);  // 5Hz update
        
        Serial.println("OK");
        return true;
    }
    
    bool InitFlash() {
        Serial.print("Initializing Flash memory... ");
        
        if (!flash_.begin(kFlashCsPin)) {
            Serial.println("FAILED");
            return false;
        }
        
        uint32_t flash_id = flash_.getJEDECID();
        Serial.print("OK (ID: 0x");
        Serial.print(flash_id, HEX);
        Serial.println(")");
        
        // Find next available address (simple approach)
        current_flash_address_ = 0;
        
        return true;
    }
    
    // ==================== SENSOR READING ====================
    
    void ReadBarometer() {
        if (!bmp_.performReading()) {
            Serial.println("BMP388 read failed");
            return;
        }
        
        current_data_.pressure_pa = bmp_.pressure;
        current_data_.altitude_m = bmp_.readAltitude(1013.25) - ground_altitude_m_;
        current_data_.temperature_c = bmp_.temperature;
        
        // Update altitude buffer for apogee detection
        altitude_buffer_[altitude_buffer_idx_] = current_data_.altitude_m;
        altitude_buffer_idx_ = (altitude_buffer_idx_ + 1) % kAltitudeBufferSize;
    }
    
    void ReadIMU() {
        sensors_event_t accel, gyro, temp;
        mpu_.getEvent(&accel, &gyro, &temp);
        
        current_data_.accel_x = accel.acceleration.x;
        current_data_.accel_y = accel.acceleration.y;
        current_data_.accel_z = accel.acceleration.z;
        
        current_data_.gyro_x = gyro.gyro.x;
        current_data_.gyro_y = gyro.gyro.y;
        current_data_.gyro_z = gyro.gyro.z;
        
        current_data_.temp_imu = temp.temperature;
        
        // Calculate vertical acceleration (assuming Z-axis is up)
        // This is simplified - you'd want proper orientation tracking
        current_data_.vertical_accel_g = current_data_.accel_z / 9.81;
    }
    
    void ReadGPS() {
        // Read GPS data
        while (gpsSerial_.available()) {
            gps_.read();
        }
        
        if (gps_.newNMEAreceived()) {
            gps_.parse(gps_.lastNMEA());
            
            current_data_.gps_fix = gps_.fix;
            current_data_.gps_satellites = gps_.satellites;
            
            if (gps_.fix) {
                current_data_.gps_latitude = gps_.latitudeDegrees;
                current_data_.gps_longitude = gps_.longitudeDegrees;
                current_data_.gps_altitude_m = gps_.altitude;
                current_data_.gps_speed_mps = gps_.speed * 0.51444;  // knots to m/s
            }
        }
    }
    
    void UpdateDerivedValues() {
        current_data_.timestamp_ms = millis();
        
        // Estimate vertical velocity (simple differentiation)
        if (last_altitude_time_ms_ > 0) {
            float dt = (current_data_.timestamp_ms - last_altitude_time_ms_) / 1000.0;
            if (dt > 0) {
                current_data_.vertical_velocity_ms = 
                    (current_data_.altitude_m - last_altitude_m_) / dt;
            }
        }
        
        last_altitude_m_ = current_data_.altitude_m;
        last_altitude_time_ms_ = current_data_.timestamp_ms;
        
        // Update max values
        if (current_data_.altitude_m > max_altitude_m_) {
            max_altitude_m_ = current_data_.altitude_m;
        }
        if (abs(current_data_.vertical_velocity_ms) > max_velocity_ms_) {
            max_velocity_ms_ = abs(current_data_.vertical_velocity_ms);
        }
        if (abs(current_data_.vertical_accel_g) > max_acceleration_g_) {
            max_acceleration_g_ = abs(current_data_.vertical_accel_g);
        }
    }
    
    // ==================== FLIGHT STATE MACHINE ====================
    
    void UpdateFlightState() {
        last_state_ = current_state_;
        
        switch (current_state_) {
            case PAD_IDLE:
                CheckForLaunch();
                break;
                
            case POWERED_ASCENT:
                CheckForCoasting();
                break;
                
            case COASTING:
                CheckForApogee();
                break;
                
            case APOGEE_DETECTED:
                if (millis() - apogee_time_ms_ > kPyroDurationMs + 500) {
                    current_state_ = DROGUE_DEPLOYED;
                }
                break;
                
            case DROGUE_DEPLOYED:
                CheckForMainDeploy();
                break;
                
            case MAIN_DEPLOYED:
                CheckForLanding();
                break;
                
            case LANDED:
                // Post-flight state
                break;
        }
        
        // State transition actions
        if (current_state_ != last_state_) {
            OnStateChange();
        }
    }
    
    void CheckForLaunch() {
        // Detect launch by sustained high acceleration
        if (current_data_.vertical_accel_g > kLaunchAccelThresholdG) {
            launch_confirm_counter_++;
            
            if (launch_confirm_counter_ >= kLaunchConfirmCount) {
                current_state_ = POWERED_ASCENT;
                launch_time_ms_ = millis();
                launch_altitude_m_ = current_data_.altitude_m;
                Serial.println("*** LAUNCH DETECTED ***");
            }
        } else {
            launch_confirm_counter_ = 0;
        }
    }
    
    void CheckForCoasting() {
        // Transition to coasting when acceleration drops below threshold
        // Typically happens when motor burns out
        if (current_data_.vertical_accel_g < 1.5) {  // Below 1.5G
            current_state_ = COASTING;
            Serial.println("*** COASTING ***");
        }
    }
    
    void CheckForApogee() {
        // Detect apogee when altitude starts decreasing
        if (current_data_.altitude_m < max_altitude_m_ - kApogeeDescentThresholdM) {
            apogee_confirm_counter_++;
            
            if (apogee_confirm_counter_ >= kApogeeConfirmCount) {
                current_state_ = APOGEE_DETECTED;
                apogee_time_ms_ = millis();
                Serial.print("*** APOGEE DETECTED at ");
                Serial.print(max_altitude_m_);
                Serial.println("m ***");
                
                DeployDrogue();
            }
        } else {
            apogee_confirm_counter_ = 0;
        }
    }
    
    void CheckForMainDeploy() {
        // Deploy main chute at preset altitude
        if (current_data_.altitude_m < kMainDeployAltitudeM && 
            current_data_.altitude_m > 10.0) {  // Safety check
            current_state_ = MAIN_DEPLOYED;
            Serial.print("*** MAIN DEPLOY at ");
            Serial.print(current_data_.altitude_m);
            Serial.println("m ***");
            
            DeployMain();
        }
    }
    
    void CheckForLanding() {
        // Detect landing by low velocity and low altitude
        if (abs(current_data_.vertical_velocity_ms) < kLandingVelocityThreshold &&
            current_data_.altitude_m < 5.0) {
            landing_confirm_counter_++;
            
            if (landing_confirm_counter_ >= kLandingConfirmCount) {
                current_state_ = LANDED;
                landing_time_ms_ = millis();
                Serial.println("*** LANDED ***");
                PrintFlightSummary();
            }
        } else {
            landing_confirm_counter_ = 0;
        }
    }
    
    void OnStateChange() {
        // Log state changes
        Serial.print("State: ");
        Serial.print(StateToString(last_state_));
        Serial.print(" -> ");
        Serial.println(StateToString(current_state_));
    }
    
    // ==================== PYRO CONTROL ====================
    
    void DeployDrogue() {
        Serial.println("FIRING DROGUE PYRO!");
        digitalWrite(kDroguePyroPin, HIGH);
        delay(kPyroDurationMs);
        digitalWrite(kDroguePyroPin, LOW);
    }
    
    void DeployMain() {
        Serial.println("FIRING MAIN PYRO!");
        digitalWrite(kMainPyroPin, HIGH);
        delay(kPyroDurationMs);
        digitalWrite(kMainPyroPin, LOW);
    }
    
    void CheckPyroContinuity() {
        bool drogue_cont = digitalRead(kPyroContinuityDroguePin) == LOW;
        bool main_cont = digitalRead(kPyroContinuityMainPin) == LOW;
        
        Serial.print("Pyro Continuity - Drogue: ");
        Serial.print(drogue_cont ? "OK" : "FAIL");
        Serial.print(", Main: ");
        Serial.println(main_cont ? "OK" : "FAIL");
    }
    
    // ==================== DATA LOGGING ====================
    
    void LogDataToFlash() {
        FlightLogEntry entry;
        entry.timestamp_ms = current_data_.timestamp_ms;
        entry.altitude_m = current_data_.altitude_m;
        entry.velocity_ms = current_data_.vertical_velocity_ms;
        entry.accel_g = current_data_.vertical_accel_g;
        entry.pressure_pa = current_data_.pressure_pa;
        entry.gps_lat = current_data_.gps_latitude;
        entry.gps_lon = current_data_.gps_longitude;
        entry.state = (uint8_t)current_state_;
        entry.gps_sats = current_data_.gps_satellites;
        
        // Write to flash
        flash_.writeByteArray(current_flash_address_, (uint8_t*)&entry, sizeof(entry));
        current_flash_address_ += sizeof(entry);
        log_entry_count_++;
        
        // Wrap around if we hit end of flash (simple approach)
        if (current_flash_address_ > 16 * 1024 * 1024) {  // 16MB
            current_flash_address_ = 0;
        }
    }
    
    // ==================== TELEMETRY ====================
    
    void SendTelemetry() {
        // Create telemetry packet
        FlightTelemetryPacket packet;
        packet.timestamp_ms = current_data_.timestamp_ms;
        packet.state = (uint8_t)current_state_;
        packet.altitude_m = current_data_.altitude_m;
        packet.velocity_ms = current_data_.vertical_velocity_ms;
        packet.accel_g = current_data_.vertical_accel_g;
        packet.pressure_pa = current_data_.pressure_pa;
        packet.gps_fix = current_data_.gps_fix;
        packet.gps_lat = current_data_.gps_latitude;
        packet.gps_lon = current_data_.gps_longitude;
        packet.gps_sats = current_data_.gps_satellites;
        packet.max_altitude_m = max_altitude_m_;
        
        // Send to RocketRadio device via ESP-NOW
        Send(DeviceType::DevRocketRadio, (uint8_t*)&packet, sizeof(packet));
    }
    
    // ==================== CALIBRATION & UTILITIES ====================
    
    void CalibrateGroundLevel() {
        Serial.print("Calibrating ground level... ");
        
        const int samples = 50;
        float sum = 0;
        
        for (int i = 0; i < samples; i++) {
            if (!bmp_.performReading()) {
                Serial.println("FAILED");
                return;
            }
            sum += bmp_.readAltitude(1013.25);
            delay(20);
        }
        
        ground_altitude_m_ = sum / samples;
        Serial.print("OK (");
        Serial.print(ground_altitude_m_);
        Serial.println("m)");
    }
    
    void StartupBeeps() {
        for (int i = 0; i < 3; i++) {
            digitalWrite(kBuzzerPin, HIGH);
            delay(100);
            digitalWrite(kBuzzerPin, LOW);
            delay(100);
        }
    }
    
    void UpdateBuzzer() {
        // Different beep patterns for different states
        switch (current_state_) {
            case PAD_IDLE:
                // Slow beep
                if (millis() % 2000 < 100) {
                    digitalWrite(kBuzzerPin, HIGH);
                } else {
                    digitalWrite(kBuzzerPin, LOW);
                }
                break;
                
            case LANDED:
                // Fast beep
                if (millis() % 500 < 100) {
                    digitalWrite(kBuzzerPin, HIGH);
                } else {
                    digitalWrite(kBuzzerPin, LOW);
                }
                break;
                
            default:
                digitalWrite(kBuzzerPin, LOW);
                break;
        }
    }
    
    const char* StateToString(FlightState state) {
        switch (state) {
            case PAD_IDLE: return "PAD_IDLE";
            case POWERED_ASCENT: return "POWERED_ASCENT";
            case COASTING: return "COASTING";
            case APOGEE_DETECTED: return "APOGEE_DETECTED";
            case DROGUE_DEPLOYED: return "DROGUE_DEPLOYED";
            case MAIN_DEPLOYED: return "MAIN_DEPLOYED";
            case LANDED: return "LANDED";
            default: return "UNKNOWN";
        }
    }
    
    void PrintFlightSummary() {
        Serial.println("\n=== FLIGHT SUMMARY ===");
        Serial.print("Flight time: ");
        Serial.print((landing_time_ms_ - launch_time_ms_) / 1000.0);
        Serial.println("s");
        
        Serial.print("Max altitude: ");
        Serial.print(max_altitude_m_);
        Serial.println("m");
        
        Serial.print("Max velocity: ");
        Serial.print(max_velocity_ms_);
        Serial.println("m/s");
        
        Serial.print("Max acceleration: ");
        Serial.print(max_acceleration_g_);
        Serial.println("G");
        
        Serial.print("Log entries: ");
        Serial.println(log_entry_count_);
        Serial.println("=====================\n");
    }
};

REGISTER_AVIONICS_DEVICE(DevFlightComputer);
