#pragma once
#include <cstdint>

enum class FlightState : uint8_t {
    Init      = 0,  // booting, sensors coming up
    Calibrate = 1,  // on-pad self-cal: gyro bias, baro ground reference
    Idle      = 2,  // on the rail, waiting for launch, airbrakes LOCKED
    Armed     = 3,  // armed (Second RBF pulled, or telemtry armed)
    Boost     = 4,  // motor burning/accelerating, brakes LOCKED
    Coast     = 5,  // burnout -> apogee, airbrake control ACTIVE
    Apogee    = 6,  // apogee detected (transient)
    Descent   = 7,  // post-apogee, falling
    Landed    = 8   // touchdown
};

const char* stateToString(FlightState state);

struct FlightData {
    bool  sensorsReady   = false;  // set once Madgwick/Kalman init & sensor self-test pass
    bool  armCommand     = false;  // ground station / switch input, latched externally
 
    float accelMagnitude = 0.0f;   // |a_body|, m/s^2 (for stationary calibration check)
    float accelWorldZ    = 0.0f;   // world-frame vertical accel, m/s^2 (from Madgwick rotation)
    float velocity       = 0.0f;   // Kalman-estimated vertical velocity, m/s (up positive)
    float altitudeRate   = 0.0f;   // baro-derived altitude rate, m/s (up positive)
};


class RangeHoldTimer {
    public:
    // Returns true the tick that the hold duration is satisfied (fires once
    // per successful hold, since reset() is expected to be called on
    // transition, see FlightStateMachine::transition()).
    bool heldInRange(float value, float lo, float hi,
                      uint32_t duration_ms, uint32_t now_ms) {
        const bool inRange = (value >= lo && value <= hi);
 
        if (!inRange) {
            holding_ = false;
            return false;
        }
 
        if (!holding_) {
            holding_   = true;
            startMs_   = now_ms;
            return false;
        }
 
        return (now_ms - startMs_) >= duration_ms;
    }
 
    void reset() { holding_ = false; }
 
    bool isHolding() const { return holding_; }
    uint32_t elapsedMs(uint32_t now_ms) const {
        return holding_ ? (now_ms - startMs_) : 0;
    }
 
private:
    bool     holding_ = false;
    uint32_t startMs_ = 0;
};


class FlightStateMachine {
public:
    void nextState(const FlightData& data, uint32_t now_ms);
 
    FlightState state() const { return state_; }
    uint32_t timeInStateMs(uint32_t now_ms) const { return now_ms - stateEnteredMs_; }
 
private:
    void transition(FlightState next, uint32_t now_ms);
 
    FlightState state_          = FlightState::Init;
    uint32_t    stateEnteredMs_ = 0;
 
    // One hold-timer per transition check. Kept separate (rather than a
    // single shared timer) so a state can arm multiple range checks
    // concurrently in the future without cross-talk.
    RangeHoldTimer calibHold_;
    RangeHoldTimer boostHold_;
    RangeHoldTimer burnoutHold_;
    RangeHoldTimer apogeeHold_;
    RangeHoldTimer descentHold_;
    RangeHoldTimer landedHold_;
        
    static constexpr float G = 9.80665f;
    
    // CALIBRATE -> IDLE: vehicle stationary (|a| ~ 1g) for this long
    static constexpr float    CAL_ACCEL_LO = 0.95f * G;
    static constexpr float    CAL_ACCEL_HI = 1.05f * G;
    static constexpr uint32_t CAL_HOLD_MS  = 2000;
    
    // ARMED -> BOOST: sustained high positive world-frame accel
    static constexpr float    BOOST_ACCEL_LO = 3.0f * G;
    static constexpr float    BOOST_ACCEL_HI = 40.0f * G;
    static constexpr uint32_t BOOST_HOLD_MS  = 50;
    
    // BOOST -> COAST: motor burnout, net accel settles back near -1g
    static constexpr float    BURNOUT_ACCEL_LO = -1.5f * G;
    static constexpr float    BURNOUT_ACCEL_HI = 0.5f * G;
    static constexpr uint32_t BURNOUT_HOLD_MS  = 100;
    
    // COAST -> APOGEE: velocity near zero at the top of the arc
    static constexpr float    APOGEE_VEL_LO  = -1.0f;
    static constexpr float    APOGEE_VEL_HI  = 1.0f;
    static constexpr uint32_t APOGEE_HOLD_MS = 250;
    
    // APOGEE -> DESCENT: confirmed downward velocity
    static constexpr float    DESCENT_VEL_LO  = -1000.0f;
    static constexpr float    DESCENT_VEL_HI  = -3.0f;
    static constexpr uint32_t DESCENT_HOLD_MS = 150;
    
    // DESCENT -> LANDED: altitude rate flat for a long hold
    static constexpr float    LANDED_RATE_LO = -0.5f;
    static constexpr float    LANDED_RATE_HI = 0.5f;
    static constexpr uint32_t LANDED_HOLD_MS = 3000;

};
