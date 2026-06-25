#pragma once
#include "sensors/sensor_types.h"
/// @brief Interface for barometric altitude sensors
class IBarometer {
    public:
        virtual ~IBarometer() = default;
        virtual bool begin() = 0;
        virtual bool read(BaroData& out) = 0;
        virtual bool calibrate(int samples, int delay_ms) = 0;
        virtual bool dataReady() { return true; }
};