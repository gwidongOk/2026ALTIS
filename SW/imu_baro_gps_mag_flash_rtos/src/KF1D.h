#ifndef KF1D_H
#define KF1D_H

#include <Arduino.h>
#include <ArduinoEigen.h>

/**
 * 1-Axis Vertical Kalman Filter (Ref: 2단부에비오닉스.pdf)
 * States: x = [pos, vel]^T
 * 
 * Predict:
 *   x = A*x + B*a
 *   P = A*P*A' + Q
 * 
 * Update:
 *   K = P*H' * inv(H*P*H' + R)
 *   x = x + K*(z - H*x)
 *   P = (I - K*H)*P
 */
class KF1D {
public:
    KF1D();
    
    // Initialize filter with initial altitude and velocity
    void init(float p0, float v0);
    
    // Prediction step using vertical acceleration [m/s^2] and time step [s]
    void predict(float acc_up, float dt);
    
    // Correction step using altitude observation [m]
    void update(float z_obs);
    
    float getPos() const { return _x(0); }
    float getVel() const { return _x(1); }

private:
    Eigen::Vector2f _x;    // State vector [pos, vel]
    Eigen::Matrix2f _P;    // Error covariance matrix
    
    // Noise parameters from Report Page 7
    const float sigma_a_sq = 6.3e-4f; // Accel variance (m/s^2)^2
    const float sigma_b_sq = 0.089f;  // Baro variance (m^2)
};

#endif
