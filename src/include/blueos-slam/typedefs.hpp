#ifndef TYPEDEFS_HPP
#define TYPEDEFS_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <variant>

// Enum class for Mavlink message types
enum class MavlinkMessage {
    ATTITUDE,
    GLOBAL_POSITION_INT,
    SCALED_IMU,
    RAW_IMU,
    SCALED_PRESSURE,
    SERVO_OUTPUT_RAW,
    SYSTEM_TIME
};

// GPSData structure
struct GPSData {
    uint64_t timestamp;  // timestamp in milliseconds
    double altitude;
    double latitude;
    double longitude;
};

// AttitudeData structure
struct AttitudeData {
    uint64_t timestamp;
    double roll;      // Rotation about X (Straight)
    double pitch;     // Rotation about Y (Right)
    double yaw;       // Rotation about Z (Down)
    double pitch_speed;
    double roll_speed;
    double yaw_speed;
};

// IMUData structure
struct IMUData {
    uint64_t timestamp;
    double x_acc;
    double x_gyro;
    double y_acc;
    double y_gyro;
    double z_acc;
    double z_gyro;
};

// PressureData structure
struct PressureData {
    uint64_t timestamp;
    double press_abs;
    double press_diff;
};

// ServoData structure
struct ServoData {
    uint64_t timestamp;
    int servo1;
    int servo2;
    int servo3;
    int servo4;
    int servo5;
    int servo6;
};

// TimeData structure
struct TimeData {
    uint64_t timestamp;
};

// LocalizationData structure that contains multiple data types
struct LocalizationData {
    uint64_t timestamp;
    GPSData gps_data;
    AttitudeData attitude_data;
    IMUData imu_data;
    PressureData pressure_data;
};

// SonarData structure
struct SonarData {
    int angle;
    int transmit_duration;
    int sample_period;
    int number_of_samples;
    std::vector<int> data;  // Replace `list` with `std::vector`
};

#endif // TYPEDEFS_H
