#pragma once
#include "sensors/sensor_types.h"
/// @brief Interface for inertial measurement unit sensors
class IImu {
    public:
        virtual ~IImu() = default;
        virtual bool begin() = 0;
        virtual bool read(IMUData& out) = 0;
        virtual bool calibrate(int samples, int delay_ms) = 0;
        virtual bool dataReady() { return true; }
};