#include "sensors/rocket_ahrs.h"
#include <Fusion.h> 
// Take note of the doomed ass conversions, m^2/s and rad/s are the SI units im using
// Basically if i put in a unit i expect the same one back
// This fusion lib is just freaky so it uses g/s and deg/s
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (57.295779513f) // 180 divided by PI
#endif
#define MS2_TO_G (0.101971621f) // 1 m/s^2 = 0.101971621 g
static FusionAhrs ahrs;
static FusionBias bias; 
static float vertical_acceleration = 0.0f;


bool setup_fusion() {
    FusionBiasInitialise(&bias);
    FusionAhrsInitialise(&ahrs);
    
    FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.1f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 0.0f,
        .recoveryTriggerPeriod = 5 * 208, // 5 seconds at 208Hz
    };
    
    FusionAhrsSetSettings(&ahrs, &settings);
    return ahrs.startup; // Return true if the AHRS is in startup mode, false otherwise
}

void update_fusion(float ax, float ay, float az, 
                   float gx, float gy, float gz, 
                   float dt) {
    FusionVector gyroscope = {gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG}; // Convert gyroscope readings from radians per second to degrees per second
    FusionVector accelerometer = {ax * MS2_TO_G, ay * MS2_TO_G, az * MS2_TO_G}; // Convert accelerometer readings from m/s^2 to g
    
    gyroscope = FusionBiasUpdate(&bias, gyroscope);
    
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    FusionVector linear_earth = FusionAhrsGetEarthAcceleration(&ahrs);
    vertical_acceleration = linear_earth.axis.z; // Store the vertical acceleration in g
}

float get_true_vertical_acceleration() {
    return vertical_acceleration * 9.80665f; // Convert from g to m/s^2
}