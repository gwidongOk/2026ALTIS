#ifndef KF1D_H
#define KF1D_H

#include <Arduino.h>
#include <ArduinoEigen.h>

/**
 * 1-Axis Vertical KF with Acceleration-Variance Adaptive Q.
 *
 * States: x = [pos, vel]^T
 *
 * Adaptive Q idea:
 *   Q represents acceleration uncertainty. Instead of a fixed value,
 *   we measure it directly from the last N IMU samples.
 *
 *   - Maintain a rolling window of body-frame vertical specific force.
 *   - Compute its variance every predict() call.
 *   - Use that variance as SIGMA_A_SQ → Q auto-scales.
 *
 *   Normal flight:    low variance  → small Q → baro-dominated (tight)
 *   Parachute deploy: variance spikes instantly (vacc +131 m/s²) →
 *                     large Q → filter opens up and tracks velocity fast
 *   After settling:   variance drops → Q returns to baseline
 *
 *   Reaction speed equals IMU rate (416 Hz), much faster than
 *   innovation-based methods that must wait for the next baro update.
 */
class KF1D {
public:
    KF1D();

    void init(float p0, float v0);
    void predict(float acc_up, float dt);   // acc_up: vertical kinematic accel [m/s²]
    void update(float z_obs);               // z_obs: baro altitude AGL [m]

    float getPos()    const { return _x(0); }
    float getVel()    const { return _x(1); }
    float getSigmaA() const { return _win_var; }  // diagnostic: current σ_a² estimate

private:
    Eigen::Vector2f _x;
    Eigen::Matrix2f _cov;

    // Fixed baro noise (matched to measurement: std=0.298 m)
    const float sigma_b_sq = 0.089f;

    // Baseline accel variance floor (measured at rest: std=0.025 m/s²)
    // Prevents Q from collapsing to 0 in steady state.
    const float sigma_a_base = 6.3e-4f;

    // Rolling window for real-time σ_a² estimation
    static constexpr int WIN = 20;
    float _win[WIN] = {};
    int   _win_idx  = 0;
    bool  _win_full = false;
    float _win_var  = 6.3e-4f;   // current variance estimate

    void pushSample(float acc_up);
    void updateVariance();
};

#endif
