#include "sensors/lsm6dsox.h"
#include "SPI.h"

LSM6DSOX::LSM6DSOX(int8_t CS_PIN) : cs_pin_(CS_PIN) {}


bool LSM6DSOX::begin() {
    bool ok = (cs_pin_ < 0) ? raw_imu_.begin_I2C() : raw_imu_.begin_SPI(cs_pin_);
    if (!ok) return false;
    // Accel
    raw_imu_.setAccelRange(LSM6DS_ACCEL_RANGE_16_G); 
    raw_imu_.setAccelDataRate(LSM6DS_RATE_208_HZ);

    // Gyro
    raw_imu_.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    raw_imu_.setGyroDataRate(LSM6DS_RATE_208_HZ);
    return true;
}

bool LSM6DSOX::read(IMUData& out) {
    sensors_event_t accel_event;
    sensors_event_t gyro_event;
    sensors_event_t temp_event;

    if (!raw_imu_.getEvent(&accel_event, &gyro_event, &temp_event)) {
        return false;
    }

    // Adafruit_LSM6DS's getEvent() already reports SI units (m/s^2, rad/s),
    // so this is a straight pass-through.
    out.accelX_ms2 = accel_event.acceleration.x;
    out.accelY_ms2 = accel_event.acceleration.y;
    out.accelZ_ms2 = accel_event.acceleration.z;
    out.gyroX_rps = gyro_event.gyro.x - gyro_bias_x_;
    out.gyroY_rps = gyro_event.gyro.y - gyro_bias_y_;
    out.gyroZ_rps = gyro_event.gyro.z - gyro_bias_z_;
    out.temperature_c = temp_event.temperature;
    out.timestamp_us = micros();

    return true;
}


bool LSM6DSOX::calibrate(int samples, int delay_ms) {
    float sx = 0, sy = 0, sz = 0;
    IMUData sample;
    for (int i = 0; i < samples; i++) {
        read(sample);
        sx += sample.gyroX_rps;
        sy += sample.gyroY_rps;
        sz += sample.gyroZ_rps;
        delay(delay_ms);
    }
    gyro_bias_x_ = sx / samples;
    gyro_bias_y_ = sy / samples;
    gyro_bias_z_ = sz / samples;
    return true;
}

bool LSM6DSOX::dataReady() {
    return true;
}