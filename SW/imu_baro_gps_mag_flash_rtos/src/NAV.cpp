#include "NAV.h"
#include <math.h>

NAV::NAV() {
    _nominal.q[0] = 1.0f;
}

void NAV::updateIMU(const Raw_imu &raw) {
    _raw_imu = raw;
    _state_imu.timestamp = raw.timestamp;
    _state_imu.ax = (float)raw.ax * ACCEL_SCALE;
    _state_imu.ay = (float)raw.ay * ACCEL_SCALE;
    _state_imu.az = (float)raw.az * ACCEL_SCALE;
    _state_imu.gx = (float)raw.gx * GYRO_SCALE;
    _state_imu.gy = (float)raw.gy * GYRO_SCALE;
    _state_imu.gz = (float)raw.gz * GYRO_SCALE;

    int64_t now_us = (int64_t)raw.timestamp;
    if (_last_imu_time_us > 0) {
        float dt = (float)(now_us - _last_imu_time_us) * 1e-6f;
        if (dt > 0 && dt < 1.0f) {
            ekfPredict(dt);
        }
    }
    _last_imu_time_us = now_us;
}

void NAV::updatePress(const Raw_press &p) { 
    _press = p; 
    ekfUpdateBaro();
}

void NAV::syncNominal() {
    // 1D KF -> 3D Nominal state mapping
    // We only use the vertical component (p[2], v[2])
    // p[2] = -altitude (NED coordinate system: D is down)
    _nominal.p[2] = -_kf.getPos();
    _nominal.v[2] = -_kf.getVel();
    _nominal.ba[2] = _kf.getBias();
    
    // Other fields stay default or zero
    _nominal.p[0] = 0; _nominal.p[1] = 0;
    _nominal.v[0] = 0; _nominal.v[1] = 0;
    _nominal.ba[0] = 0; _nominal.ba[1] = 0;
}

bool NAV::ekfBegin() {
    // Initial altitude from baro if available
    _kf.init(_press.alt, 0.0f, 0.0f);
    _kf_ready = true;
    syncNominal();
    return true;
}

void NAV::ekfReset() {
    _kf_ready = false;
    _nominal = State_nominal{};
    _nominal.q[0] = 1.0f;
}

void NAV::ekfPredict(float dt) {
    _nominal.timestamp = _state_imu.timestamp;
    if (!_kf_ready) return;
    
    // Vertical acceleration (assuming Z-axis is vertical)
    // Subtract gravity (9.80665) if the sensor measures total acceleration
    // For LSM6DSO32, when upright, AZ is ~1g (9.8m/s^2)
    float acc_vert = _state_imu.az - 9.80665f;
    
    _kf.predict(acc_vert, dt);
    syncNominal();
}

void NAV::ekfUpdateBaro() {
    if (!_kf_ready) return;
    _kf.updateBaro(_press.alt, 0.5f); // var_z = 0.5 (tuning)
    syncNominal();
}
