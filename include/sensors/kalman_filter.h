#pragma once
#ifdef F
#define LKF_SAVED_F F
#undef F
#endif

// 2. Include Eigen safely
#include <Eigen/Dense>

// 3. Restore the Arduino F macro for the rest of your sketch
#ifdef LKF_SAVED_F
#define F(string_literal) LKF_SAVED_F(string_literal)
#undef LKF_SAVED_F
#endif
class LinearKalmanFilter {
public:
    /**
     * @brief Construct a new Linear Kalman Filter object
     * Initializes state vector, state covariance, measurement matrix, and measurement noise.
     */
    LinearKalmanFilter();

    /**
     * @brief Stage 1: Predict state forward using AHRS vertical acceleration
     * * @param accel_world_ms2 The vertical acceleration in the world frame (m/s^2)
     * @param dt Time delta since the last prediction step (seconds)
     */
    void predict(float accel_world_ms2, float dt);

    /**
     * @brief Stage 2: Correct state using the BMP581 Barometer altitude
     * * @param baro_altitude_m The measured altitude from the barometer (meters)
     */
    void update(float baro_altitude_m);

    /**
     * @brief Get the estimated altitude
     * @return float Estimated altitude in meters
     */
    float get_altitude() const;

    /**
     * @brief Get the estimated vertical velocity
     * @return float Estimated velocity in m/s
     */
    float get_velocity() const;
    void reset() {
        X.setZero();
        P.setIdentity();
        P *= 1.0f; // Reset uncertainty
    }

private:
    Eigen::Vector2f X;            // State vector: [position, velocity]
    Eigen::Matrix2f P;            // State Covariance Matrix
    Eigen::Matrix<float, 1, 2> H; // Measurement Matrix
    Eigen::Matrix<float, 1, 1> R; // Measurement Noise Matrix
};