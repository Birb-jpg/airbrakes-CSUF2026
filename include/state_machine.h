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

const char* stateToString(FlightState state) {
    switch (state) {
        case FlightState::Init:      return "INIT";
        case FlightState::Calibrate: return "CALIBRATE";
        case FlightState::Idle:      return "IDLE";
        case FlightState::Boost:     return "BOOST";
        case FlightState::Coast:     return "COAST";
        case FlightState::Apogee:    return "APOGEE";
        case FlightState::Descent:   return "DESCENT";
        case FlightState::Landed:    return "LANDED";
        default:                     return "UNKNOWN";
    }
}