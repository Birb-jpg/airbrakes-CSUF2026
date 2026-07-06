#include "state_machine.h"

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


void FlightStateMachine::transition(FlightState next, uint32_t now_ms) {
    state_          = next;
    stateEnteredMs_ = now_ms;
 
    // Reset every hold timer on any transition
    calibHold_.reset();
    boostHold_.reset();
    burnoutHold_.reset();
    apogeeHold_.reset();
    descentHold_.reset();
    landedHold_.reset();
}
 
void FlightStateMachine::nextState(const FlightData& data, uint32_t now_ms) {
    switch (state_) {
 
        case FlightState::Init:
            // Trigger: sensor stack initialized and
            // self-test passed.
            if (data.sensorsReady) {
                transition(FlightState::Calibrate, now_ms);
            }
            break;
 
        case FlightState::Calibrate:
            // Trigger: |a_body| held within [0.95g, 1.05g] for CAL_HOLD_MS
            // confirms the vehicle is stationary long enough for the
            // Madgwick filter / bias estimates to settle.
            if (calibHold_.heldInRange(data.accelMagnitude,CAL_ACCEL_LO, CAL_ACCEL_HI, CAL_HOLD_MS, now_ms)) {
                transition(FlightState::Idle, now_ms);
            }
            break;
 
        case FlightState::Idle:
            // Trigger: external arm command (switch or ground-station
            // uplink), latched FlightData.armCommand.
            if (data.armCommand) {
                transition(FlightState::Armed, now_ms);
            }
            break;
 
        case FlightState::Armed:
            // Trigger: world-frame vertical accel held above ~3g for
            // low-latency detection matters here.
            if (boostHold_.heldInRange(data.accelWorldZ, BOOST_ACCEL_LO, BOOST_ACCEL_HI, BOOST_HOLD_MS, now_ms)) {
                transition(FlightState::Boost, now_ms);
            }
            break;
 
        case FlightState::Boost:
            // Trigger: motor burnout world-frame accel drops back to and
            // holds near [-1.5g, 0.5g] (drag + gravity only, no thrust) for
            // BURNOUT_HOLD_MS.
            if (burnoutHold_.heldInRange(data.accelWorldZ, BURNOUT_ACCEL_LO, BURNOUT_ACCEL_HI, BURNOUT_HOLD_MS, now_ms)) {
                transition(FlightState::Coast, now_ms);
            }
            break;
 
        case FlightState::Coast:
            // Trigger: Kalman-estimated velocity held within [-1, 1] m/s for
            // APOGEE_HOLD_MS vehicle is at (or hovering around) the top
            // of the arc.
            if (apogeeHold_.heldInRange(data.velocity, APOGEE_VEL_LO, APOGEE_VEL_HI, APOGEE_HOLD_MS, now_ms)) {
                transition(FlightState::Apogee, now_ms);
            }
            break;
 
        case FlightState::Apogee:
            // Trigger: velocity held below -3 m/s for DESCENT_HOLD_MS
            // confirms sustained downward motion past the apogee noise
            // band (this is also where you'd fire drogue/main deployment
            // logic upstream of this state machine, if not already fired).
            if (descentHold_.heldInRange(data.velocity, DESCENT_VEL_LO, DESCENT_VEL_HI, DESCENT_HOLD_MS, now_ms)) {
                transition(FlightState::Descent, now_ms);
            }
            break;
 
        case FlightState::Descent:
            // Trigger: baro altitude rate held within [-0.5, 0.5] m/s for a
            // long hold (LANDED_HOLD_MS = 3s) long enough to reject
            // parachute false positives.
            if (landedHold_.heldInRange(data.altitudeRate, LANDED_RATE_LO, LANDED_RATE_HI, LANDED_HOLD_MS, now_ms)) {
                transition(FlightState::Landed, now_ms);
            }
            break;
 
        case FlightState::Landed:
            // Terminal state no automatic transitions out.
        
            break;
    }
}
 
