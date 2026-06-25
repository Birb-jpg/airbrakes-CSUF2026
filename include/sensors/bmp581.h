#pragma once
#include "sensors/interfaces/IBarometer.h"
#include "sensor_types.h"
#include "Adafruit_BMP5xx.h"

class BMP581 : public IBarometer {
    public:
        explicit BMP581(int8_t CS_PIN = -1);
        bool begin() override;
        bool read(BaroData& out) override;
        /// @brief Calibrates the barometer by taking multiple readings and averaging them.
        /// @param samples The number of readings to take.
        /// @param delay_ms The delay between each reading in milliseconds.
        /// @return True if calibration was successful, false otherwise.
        bool calibrate(int samples, int delay_ms = 10) override;
        bool dataReady() override;
    private:
        Adafruit_BMP5xx raw_baro_;
        int8_t cs_pin_;
        float pad_pressure_pa_ = 101325.0f;
};