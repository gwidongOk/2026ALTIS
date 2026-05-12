#include "NAV.h"

using namespace Eigen;

NAV::NAV() {
    ekfReset();
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
            // 1. Integrate Gyro to update Quaternion (Report Page 4-5)
            Vector3f w(_state_imu.gx, _state_imu.gy, _state_imu.gz);
            integrateQuaternion(w, dt);
            
            // 2. Perform KF Prediction
            ekfPredict(dt);
        }
    }
    _last_imu_time_us = now_us;
}

void NAV::integrateQuaternion(const Vector3f& w, float dt) {
    // Quaternion differential: q_dot = 0.5 * q * [0, w]^T (Report Page 5)
    // Using Eigen's AngleAxis for robust 1st order rotation integration
    float w_norm = w.norm();
    if (w_norm > 1e-6f) {
        Quaternionf dq(AngleAxisf(w_norm * dt, w / w_norm));
        _q = (_q * dq).normalized();
    } else {
        // Small angle approximation to avoid division by zero
        Quaternionf dq(1.0f, 0.5f * w.x() * dt, 0.5f * w.y() * dt, 0.5f * w.z() * dt);
        _q = (_q * dq).normalized();
    }
}

void NAV::updatePress(const Raw_press &p) { 
    _press = p; 
    ekfUpdateBaro();
}

void NAV::syncNominal() {
    _nominal.timestamp = _state_imu.timestamp;
    
    // Position/Velocity sync (1D KF -> 3D NED)
    _nominal.p[2] = -_kf.getPos(); // Altitude Up to NED Down
    _nominal.v[2] = -_kf.getVel(); // Velocity Up to NED Down
    
    // Orientation sync
    _nominal.q[0] = _q.w();
    _nominal.q[1] = _q.x();
    _nominal.q[2] = _q.y();
    _nominal.q[3] = _q.z();

    // Reset unused fields
    _nominal.p[0] = 0; _nominal.p[1] = 0;
    _nominal.v[0] = 0; _nominal.v[1] = 0;
    _nominal.ba[0] = 0; _nominal.ba[1] = 0; _nominal.ba[2] = 0;
    _nominal.bg[0] = 0; _nominal.bg[1] = 0; _nominal.bg[2] = 0;
}

bool NAV::ekfBegin() {
    _q.setIdentity();
    _kf.init(_press.alt, 0.0f);
    _kf_ready = true;
    syncNominal();
    return true;
}

void NAV::ekfReset() {
    _kf_ready = false;
    _q.setIdentity();
    _kf.init(0.0f, 0.0f);
    _nominal = State_nominal{};
    _nominal.q[0] = 1.0f;
    _last_imu_time_us = 0;
}

void NAV::ekfPredict(float dt) {
    if (!_kf_ready) return;
    
    // 1. Transform Body Accel to NED Frame (Report Page 5)
    // Sensors measure specific force f_body. Acceleration a_ned = R*f_body + g_ned.
    Vector3f f_body(_state_imu.ax, _state_imu.ay, _state_imu.az);
    
    // R*f_body using Eigen quaternion multiplication
    Vector3f f_ned = _q * f_body;
    
    // Kinematic acceleration in NED Down
    Vector3f a_ned = f_ned + _g_ned;
    
    // 2. Extract Vertical Up Acceleration: a_up = -a_ned.z
    float acc_up = -a_ned.z();
    
    _kf.predict(acc_up, dt);
    syncNominal();
}

void NAV::ekfUpdateBaro() {
    if (!_kf_ready) return;
    _kf.update(_press.alt);
    syncNominal();
}
