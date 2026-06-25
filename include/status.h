#pragma once
#include <cstdint>

enum class FlightState : uint8_t {
    Init      = 0,  // booting, sensors coming up
    Calibrate = 1,  // on-pad self-cal: gyro bias, baro ground reference
    Idle      = 2,  // armed (RBF pulled), on the rail, waiting for launch
    Boost     = 3,  // motor burning/accelerating, brakes LOCKED
    Coast     = 4,  // burnout -> apogee, airbrake control ACTIVE
    Apogee    = 5,  // apogee detected (transient)
    Descent   = 6,  // post-apogee, falling
    Landed    = 7   // touchdown
};
