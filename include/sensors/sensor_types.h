#pragma once
#include <cstdint>

/// @brief IMU data structure
struct IMUData {
    float accelX_ms2;       // acceleration in m/s^2
    float accelY_ms2;
    float accelZ_ms2;
    float gyroX_rps;        // angular velocity in rad/s
    float gyroY_rps;
    float gyroZ_rps;
    float temperature_c;    // deg C
    uint64_t timestamp_us;  // sample time, microseconds since boot
};

/// @brief Single-channel accelerometer sample
struct AccelData {
    float accelX_ms2;       // acceleration in m/s^2
    float accelY_ms2;
    float accelZ_ms2;
    uint64_t timestamp_us;  // sample time, microseconds since boot
};

/// @brief Barometer data structure
struct BaroData {
    float pressure_pa;      // Pa
    float altitude_m;       // altitude AGL
    float temperature_c;    // deg C
    uint64_t timestamp_us;  // sample time, microseconds since boot
};
