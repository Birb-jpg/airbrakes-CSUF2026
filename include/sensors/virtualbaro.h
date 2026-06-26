#pragma once
#include "sensors/interfaces/IBarometer.h"
#include "sensor_types.h"

class VirtualBarometer : public IBarometer {
    public:
        explicit VirtualBarometer(int8_t CS_PIN = -1);
        bool begin() override;
        bool read(BaroData& out) override;
        /// @brief Calibrates the barometer by taking multiple readings and averaging them.
        /// @param samples The number of readings to take.
        /// @param delay_ms The delay between each reading in milliseconds.
        /// @return True if calibration was successful, false otherwise.
        bool calibrate(int samples, int delay_ms = 10) override;
        bool dataReady() override;
    private:
        int8_t cs_pin_;
        float pad_pressure_pa_ = 101325.0f;
};