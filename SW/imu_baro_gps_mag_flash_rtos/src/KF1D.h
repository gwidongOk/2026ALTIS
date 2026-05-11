#ifndef KF1D_H
#define KF1D_H

#include <Arduino.h>

/**
 * 1-Axis (Vertical) Kalman Filter
 * States: [pos, vel, accel_bias]^T
 * Model: 
 *   pos(k+1) = pos(k) + vel(k)*dt + 0.5*(accel - bias)*dt^2
 *   vel(k+1) = vel(k) + (accel - bias)*dt
 *   bias(k+1) = bias(k)
 */
class KF1D {
public:
    KF1D();
    void init(float z0, float v0, float b0);
    void predict(float accel_m_s2, float dt);
    void updateBaro(float z_obs, float var_z);
    
    float getPos() const { return x[0]; }
    float getVel() const { return x[1]; }
    float getBias() const { return x[2]; }

private:
    float x[3];    // State: [z, v, b]
    float P[3][3]; // Covariance
    
    // Process noise (tuning parameters)
    const float Q_accel = 0.05f; 
    const float Q_bias  = 0.001f;
};

#endif
