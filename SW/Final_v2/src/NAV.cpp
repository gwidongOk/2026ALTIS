#include "NAV.h"

using namespace Eigen;

NAV::NAV() {
    kfReset();
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
    if (_last_imu_time_us > 0 && _kf_ready) {
        float dt = (float)(now_us - _last_imu_time_us) * 1e-6f;
        if (dt > 0 && dt < 1.0f) {
            // 1. Gyro -> quaternion integration
            Vector3f w(_state_imu.gx, _state_imu.gy, _state_imu.gz);
            integrateQuaternion(w, dt);

            // 2. Body specific force -> NED kinematic accel
            Vector3f f_body(_state_imu.ax, _state_imu.ay, _state_imu.az);
            Vector3f a_ned = _q * f_body + _g_ned;

            // 3. D (altitude): KF predict
            float acc_up = -a_ned.z();
            _kf.predict(acc_up, dt);

            // 4. NE: pure dead-reckoning integration (no observation)
            Vector2f a_ne(a_ned.x(), a_ned.y());
            _pos_ne += _vel_ne * dt + 0.5f * a_ne * dt * dt;
            _vel_ne += a_ne * dt;

            syncNominal();
        }
    }
    _last_imu_time_us = now_us;
}

void NAV::integrateQuaternion(const Vector3f& w, float dt) {
    // q_dot = 0.5 * q * [0, w]^T — implemented via AngleAxis for stability
    float w_norm = w.norm();
    if (w_norm > 1e-6f) {
        Quaternionf dq(AngleAxisf(w_norm * dt, w / w_norm));
        _q = (_q * dq).normalized();
    } else {
        Quaternionf dq(1.0f, 0.5f * w.x() * dt, 0.5f * w.y() * dt, 0.5f * w.z() * dt);
        _q = (_q * dq).normalized();
    }
}

void NAV::updatePress(const Raw_press &p) {
    _press = p;
    if (_kf_ready) {
        _kf.update(p.alt);
        syncNominal();
    }
}

void NAV::syncNominal() {
    _nominal.timestamp = _state_imu.timestamp;

    // NED position (NE: integrated, D: KF, Up->Down)
    _nominal.p[0] = _pos_ne.x();
    _nominal.p[1] = _pos_ne.y();
    _nominal.p[2] = -_kf.getPos();

    // NED velocity
    _nominal.v[0] = _vel_ne.x();
    _nominal.v[1] = _vel_ne.y();
    _nominal.v[2] = -_kf.getVel();

    // Quaternion
    _nominal.q[0] = _q.w();
    _nominal.q[1] = _q.x();
    _nominal.q[2] = _q.y();
    _nominal.q[3] = _q.z();
}

bool NAV::kfBegin() {
    // 1. Orientation Alignment
    // Average internal state samples (IMU_Task is already running)
    Vector3f sum_a = Vector3f::Zero();
    int count = 0;
    uint32_t last_t = 0;
    
    // Wait for 50 distinct samples (~0.12s at 400Hz, but loop 500 times max to prevent hang)
    for (int i = 0; i < 500 && count < 50; i++) {
        if (_state_imu.timestamp != last_t) {
            sum_a += Vector3f(_state_imu.ax, _state_imu.ay, _state_imu.az);
            last_t = _state_imu.timestamp;
            count++;
        }
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }

    if (count > 0) {
        Vector3f avg_a = sum_a / (float)count;
        if (avg_a.norm() > 0.1f) {
            avg_a.normalize();
            // Accelerometer measures -g (Upward) in body frame. Match to NED Up [0, 0, -1].
            _q = Quaternionf::FromTwoVectors(avg_a, Vector3f(0.0f, 0.0f, -1.0f));
        } else {
            _q.setIdentity();
        }
    } else {
        _q.setIdentity();
    }

    _pos_ne.setZero();
    _vel_ne.setZero();
    _kf.init(_press.alt, 0.0f);
    _kf_ready = true;
    syncNominal();
    return true;
}

void NAV::kfReset() {
    _kf_ready = false;
    _q.setIdentity();
    _pos_ne.setZero();
    _vel_ne.setZero();
    _kf.init(0.0f, 0.0f);
    _nominal = State_nominal{};
    _nominal.q[0] = 1.0f;
    _last_imu_time_us = 0;
}

void NAV::quatToEuler(float qw, float qx, float qy, float qz,
                      float &roll, float &pitch, float &yaw) {
    // ZYX (yaw -> pitch -> roll) convention
    // roll (X)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (Y) — clamp for gimbal lock
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) pitch = copysignf((float)M_PI * 0.5f, sinp);
    else                     pitch = asinf(sinp);

    // yaw (Z)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = atan2f(siny_cosp, cosy_cosp);
}
