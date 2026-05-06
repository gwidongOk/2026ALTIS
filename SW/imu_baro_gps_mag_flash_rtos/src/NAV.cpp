#include "NAV.h"
#include <math.h>

NAV::NAV() {
    _nominal.q[0] = 1.0f;
    _nominal.q[1] = 0.0f;
    _nominal.q[2] = 0.0f;
    _nominal.q[3] = 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────
// Sensor task callbacks
// ─────────────────────────────────────────────────────────────────────────

void NAV::updateIMU(const Raw_imu &raw) {
    _raw_imu = raw;
    _state_imu.timestamp = raw.timestamp;
    _state_imu.ax = (float)raw.ax * ACCEL_SCALE;
    _state_imu.ay = (float)raw.ay * ACCEL_SCALE;
    _state_imu.az = (float)raw.az * ACCEL_SCALE;
    _state_imu.gx = (float)raw.gx * GYRO_SCALE;
    _state_imu.gy = (float)raw.gy * GYRO_SCALE;
    _state_imu.gz = (float)raw.gz * GYRO_SCALE;

    // Calculate dt and predict
    int64_t now_us = (int64_t)raw.timestamp;
    if (_last_imu_time_us > 0) {
        float dt = (float)(now_us - _last_imu_time_us) * 1e-6f;
        if (dt > 0 && dt < 1.0f) { // Sanity check
            ekfPredict(dt);
        }
    }
    _last_imu_time_us = now_us;
}

void NAV::updatePress(const Raw_press &p) { 
    _press = p; 
    ekfUpdateBaro();
}

void NAV::updateMag(const Raw_mag &m) { 
    _mag = m; 
    ekfUpdateMag();
}

void NAV::updateGps(const Raw_gps &g) {
    _gps = g;
    if (_ekf_ready && _gps.hasPos) {
        _ekf.updateGps(_gps.pn, _gps.pe, _gps.pd,
                       _gps.vn, _gps.ve, _gps.vd,
                       _gps.hAcc, _gps.vAcc);
        syncNominal();
    }
}

// ─────────────────────────────────────────────────────────────────────────
// Internal: sync nominal from filter
// ─────────────────────────────────────────────────────────────────────────

void NAV::syncNominal() {
    const float* p  = _ekf.position();
    const float* v  = _ekf.velocity();
    const float* q  = _ekf.quaternion();
    const float* ba = _ekf.accelBias();
    const float* bg = _ekf.gyroBias();
    _nominal.p[0]  = p[0];   _nominal.p[1]  = p[1];   _nominal.p[2]  = p[2];
    _nominal.v[0]  = v[0];   _nominal.v[1]  = v[1];   _nominal.v[2]  = v[2];
    _nominal.q[0]  = q[0];   _nominal.q[1]  = q[1];
    _nominal.q[2]  = q[2];   _nominal.q[3]  = q[3];
    _nominal.ba[0] = ba[0];  _nominal.ba[1] = ba[1];  _nominal.ba[2] = ba[2];
    _nominal.bg[0] = bg[0];  _nominal.bg[1] = bg[1];  _nominal.bg[2] = bg[2];
}

// ─────────────────────────────────────────────────────────────────────────
// ES-EKF lifecycle
// ─────────────────────────────────────────────────────────────────────────

bool NAV::ekfBegin() {
    const float a[3] = { _state_imu.ax, _state_imu.ay, _state_imu.az };
    const float m[3] = { _mag.mx,        _mag.my,        _mag.mz       };

    const float a_n = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    const float m_n = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (a_n < 1.0f || m_n < 1e-6f) return false;   // bogus IMU/MAG

    float q0[4];
    ESEKF::triad(a, m, q0);

    float p0[3] = {0.0f, 0.0f, 0.0f};
    float v0[3] = {0.0f, 0.0f, 0.0f};
    if (_gps.hasPos) {
        p0[0] = _gps.pn; p0[1] = _gps.pe; p0[2] = _gps.pd;
        v0[0] = _gps.vn; v0[1] = _gps.ve; v0[2] = _gps.vd;
    }

    _ekf.init(p0, v0, q0);
    _ekf_ready = true;
    syncNominal();
    return true;
}

void NAV::ekfBegin(const float p0[3], const float v0[3], const float q0[4]) {
    _ekf.init(p0, v0, q0);
    _ekf_ready = true;
    syncNominal();
}

void NAV::ekfReset() {
    _ekf_ready = false;
    _ekf.reset();
    _nominal = State_nominal{};
    _nominal.q[0] = 1.0f;
}

// ─────────────────────────────────────────────────────────────────────────
// ES-EKF cycle
// ─────────────────────────────────────────────────────────────────────────

void NAV::ekfPredict(float dt) {
    _nominal.timestamp = _state_imu.timestamp;
    if (!_ekf_ready) return;
    const float a[3] = { _state_imu.ax, _state_imu.ay, _state_imu.az };
    const float w[3] = { _state_imu.gx, _state_imu.gy, _state_imu.gz };
    _ekf.predict(a, w, dt);
    syncNominal();
}

void NAV::ekfUpdateBaro() {
    if (!_ekf_ready) return;
    _ekf.updateBaro(_press.alt);
    syncNominal();
}

void NAV::ekfUpdateMag() {
    if (!_ekf_ready) return;
    const float m[3] = { _mag.mx, _mag.my, _mag.mz };
    _ekf.updateMag(m);
    syncNominal();
}

void NAV::ekfUpdateStaticAlignment() {
    if (!_ekf_ready) return;
    
    // 1. Zero Velocity Update (속도는 0이다)
    _ekf.updateZupt();
    
    // 2. Inclinometer Update (가속도계는 중력만 측정한다 -> 수평 및 가속도 바이어스 잡기)
    const float a[3] = { _state_imu.ax, _state_imu.ay, _state_imu.az };
    _ekf.updateAccStatic(a);
    
    // 3. Zero Angular Rate Update (자이로는 0을 측정한다 -> 자이로 바이어스 잡기)
    const float w[3] = { _state_imu.gx, _state_imu.gy, _state_imu.gz };
    _ekf.updateGyroStatic(w);
    
    // 4. Compass Update (지자기를 이용해 방위각 잡기)
    const float m[3] = { _mag.mx, _mag.my, _mag.mz };
    _ekf.updateMag(m);
    
    syncNominal();
}

void NAV::ekfUpdateGps() {
    if (!_ekf_ready || !_gps.hasPos) return;
    _ekf.updateGps(_gps.pn, _gps.pe, _gps.pd,
                   _gps.vn, _gps.ve, _gps.vd,
                   _gps.hAcc, _gps.vAcc);
    syncNominal();
}
