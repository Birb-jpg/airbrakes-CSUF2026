#include "sensors/kalman_filter.h"
#include <cmath> // Required for std::pow

LinearKalmanFilter::LinearKalmanFilter() {
    // Initialize state [position, velocity] to zero
    X.setZero();

    // Initialize State Covariance (Initial uncertainty)
    P.setIdentity();
    P *= 1.0f; 

    // Measurement Matrix: We only directly measure position (via Barometer)
    H << 1.0f, 0.0f;

    // Measurement Noise Covariance: Set based on BMP581 variance
    R << 0.16f; // Variance of baro readings (meters squared)
}

// Stage 1: Predict state forward using AHRS vertical acceleration
void LinearKalmanFilter::predict(float accel_world_ms2, float dt) {
    // Dynamic State Transition Matrix (A)
    Eigen::Matrix2f A;
    A << 1.0f,   dt,
         0.0f, 1.0f;

    // Control Input Matrix (B) - maps acceleration to pos and vel
    Eigen::Vector2f B;
    B << 0.5f * dt * dt,
                dt;

    // Process Noise Covariance (Q) - how much we trust our physics model
    // Crank this up if you expect heavy rocket motor vibrations!
    Eigen::Matrix2f Q;
    float q_accel = 0.5f; // Expected variance of IMU acceleration noise
    Q << 0.25f * std::pow(dt, 4), 0.5f * std::pow(dt, 3),
         0.5f * std::pow(dt, 3),  std::pow(dt, 2);
    Q *= q_accel;

    // Execute LKF Prediction Equations
    X = (A * X) + (B * accel_world_ms2);
    P = (A * P * A.transpose()) + Q;
}

// Stage 2: Correct state using the BMP581 Barometer altitude
void LinearKalmanFilter::update(float baro_altitude_m) {
    // Innovation (Measurement Residual)
    Eigen::Matrix<float, 1, 1> Z;
    Z << baro_altitude_m;
    Eigen::Matrix<float, 1, 1> y = Z - (H * X);

    // Innovation Covariance (S)
    Eigen::Matrix<float, 1, 1> S = (H * P * H.transpose()) + R;

    // Kalman Gain (K)
    Eigen::Vector2f K = P * H.transpose() * S.inverse();

    // Update State Matrix and Covariance
    X = X + (K * y(0,0));
    Eigen::Matrix2f I = Eigen::Matrix2f::Identity();
    P = (I - (K * H)) * P;
}

float LinearKalmanFilter::get_altitude() const { return X(0); }
float LinearKalmanFilter::get_velocity() const { return X(1); }