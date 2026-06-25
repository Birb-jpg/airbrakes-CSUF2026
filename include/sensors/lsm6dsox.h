#pragma once
#include "sensors/interfaces/IImu.h"
#include "sensor_types.h"
#include <Adafruit_LSM6DSOX.h>
class LSM6DSOX : public IImu {
    public:
        explicit LSM6DSOX(int8_t CS_PIN = -1);
        bool begin() override;
        bool read(IMUData& out) override;
        bool calibrate(int samples, int delay_ms = 10) override;
        bool dataReady() override;
    private:
        int8_t cs_pin_;
        Adafruit_LSM6DSOX raw_imu_;
        float gyro_bias_x_ = 0.0f;
        float gyro_bias_y_ = 0.0f;
        float gyro_bias_z_ = 0.0f;
};