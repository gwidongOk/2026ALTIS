#include "ESEKF.h"

using namespace Eigen;

// static constexpr 멤버 정의 (ODR-use 대응)
constexpr float ESEKF::G_VAL;
constexpr float ESEKF::DT_IMU;
constexpr float ESEKF::VAR_ACC;
constexpr float ESEKF::VAR_GYRO;
constexpr float ESEKF::VAR_BA;
constexpr float ESEKF::VAR_BG;
constexpr float ESEKF::VAR_BARO;
constexpr float ESEKF::VAR_GPS_POS_H;
constexpr float ESEKF::VAR_GPS_POS_V;
constexpr float ESEKF::VAR_GPS_VEL_H;
constexpr float ESEKF::VAR_GPS_VEL_V;
constexpr float ESEKF::VAR_MAG;

// ─────────────────────────────────────────────────────────────────────────
// Lifecycle / Initialization
// ─────────────────────────────────────────────────────────────────────────

ESEKF::ESEKF() {
    reset();
}

void ESEKF::reset() {
    _p.setZero();
    _v.setZero();
    _q.setIdentity();
    _ba.setZero();
    _bg.setZero();
    syncQuaternionRaw();

    // [SensorSpec 통합] 기본 스펙 로드
    _v_acc  = VAR_ACC;
    _v_gyro = VAR_GYRO;
    _v_ba   = VAR_BA;
    _v_bg   = VAR_BG;

    _v_baro = VAR_BARO;
    _v_mag  = VAR_MAG;
    _v_gps_ph = VAR_GPS_POS_H; _v_gps_pv = VAR_GPS_POS_V;
    _v_gps_vh = VAR_GPS_VEL_H; _v_gps_vv = VAR_GPS_VEL_V;

    _g_ned << 0.0f, 0.0f, G_VAL;
    _m_ref_ned << 0.5961f, -0.0838f, 0.7986f; // Korea Default

    _last_accel_mag = 0.0f;

    // 초기 공분산 (P0)
    _covP.setZero();
    _covP.diagonal().segment<3>(0).array()  = 3.0f * 3.0f;     // pos
    _covP.diagonal().segment<3>(3).array()  = 1.0f * 1.0f;     // vel
    _covP.diagonal().segment<3>(6).array()  = 0.087f * 0.087f; // att (5deg)
    _covP.diagonal().segment<3>(9).array()  = 0.5f * 0.5f;     // ba
    _covP.diagonal().segment<3>(12).array() = 0.01f * 0.01f;   // bg
}

void ESEKF::init(const float p0[3], const float v0[3], const float q0[4]) {
    _p = Vector3f(p0[0], p0[1], p0[2]);
    _v = Vector3f(v0[0], v0[1], v0[2]);
    _q = Quaternionf(q0[0], q0[1], q0[2], q0[3]); // [w,x,y,z]
    _q.normalize();
    syncQuaternionRaw();
}

// ─────────────────────────────────────────────────────────────────────────
// Filter Cycle
// ─────────────────────────────────────────────────────────────────────────

void ESEKF::predict(const float a_m[3], const float w_m[3], float dt) {
    Vector3f am(a_m[0], a_m[1], a_m[2]);
    Vector3f wm(w_m[0], w_m[1], w_m[2]);

    Vector3f a_hat = am - _ba;
    Vector3f w_hat = wm - _bg;

    // bias-corrected specific force 크기 저장 (GPS high-G 페널티에서 사용)
    _last_accel_mag = a_hat.norm();

    Matrix3f R_nb = _q.toRotationMatrix();

    // 1. Nominal State Propagation
    Vector3f a_ned = R_nb * a_hat + _g_ned;
    _p += _v * dt + 0.5f * a_ned * dt * dt;
    _v += a_ned * dt;

    Vector3f theta = w_hat * dt;
    float th_n = theta.norm();
    if (th_n > 1e-10f) {
        _q = (_q * Quaternionf(AngleAxisf(th_n, theta/th_n))).normalized();
    } else {
        _q = (_q * Quaternionf(1.0f, theta.x()*0.5f, theta.y()*0.5f, theta.z()*0.5f)).normalized();
    }
    syncQuaternionRaw();

    // 2. Covariance Propagation (Jacobian F)
    // MATLAB 버전과 동일하게 연속시간 시스템 행렬 Fc 정의 후 테일러 급수 3차 전개 적용
    Matrix<float, 15, 15> Fc = Matrix<float, 15, 15>::Zero();
    Fc.block<3,3>(0, 3) = Matrix3f::Identity();
    Fc.block<3,3>(3, 6) = -R_nb * skew(a_hat);
    Fc.block<3,3>(3, 9) = -R_nb;
    Fc.block<3,3>(6, 6) = -skew(w_hat);
    Fc.block<3,3>(6, 12)= -Matrix3f::Identity();

    Matrix<float, 15, 15> Fdt = Fc * dt;
    Matrix<float, 15, 15> Fdt2 = Fdt * Fdt;
    Matrix<float, 15, 15> Fdt3 = Fdt2 * Fdt;
    
    // F = I + Fdt + 1/2*Fdt^2 + 1/6*Fdt^3
    Matrix<float, 15, 15> F = Matrix<float, 15, 15>::Identity() + Fdt + 0.5f * Fdt2 + (1.0f/6.0f) * Fdt3;

    // Discrete Q
    Matrix<float, 15, 15> Q = Matrix<float, 15, 15>::Zero();
    Q.diagonal().segment<3>(3).array()  = _v_acc * dt;
    Q.diagonal().segment<3>(6).array()  = _v_gyro * dt;
    Q.diagonal().segment<3>(9).array()  = _v_ba * dt;
    Q.diagonal().segment<3>(12).array() = _v_bg * dt;

    _covP = F * _covP * F.transpose() + Q;
    _covP = 0.5f * (_covP + _covP.transpose()).eval();
}

void ESEKF::measurementUpdate(const MatrixXf& H, const VectorXf& y, const MatrixXf& R) {
    MatrixXf S = H * _covP * H.transpose() + R;
    MatrixXf K = _covP * H.transpose() * S.inverse();
    VectorXf dx = K * y;

    // Joseph Update
    MatrixXf IKH = Matrix<float, 15, 15>::Identity() - K * H;
    _covP = IKH * _covP * IKH.transpose() + K * R * K.transpose();
    _covP = 0.5f * (_covP + _covP.transpose()).eval();

    // State Injection
    _p  += dx.segment<3>(0);
    _v  += dx.segment<3>(3);
    _ba += dx.segment<3>(9);
    _bg += dx.segment<3>(12);

    Vector3f dth = dx.segment<3>(6);
    float dth_n = dth.norm();
    if (dth_n > 1e-10f) {
        _q = (_q * Quaternionf(AngleAxisf(dth_n, dth/dth_n))).normalized();
    } else {
        _q = (_q * Quaternionf(1.0f, dth.x()*0.5f, dth.y()*0.5f, dth.z()*0.5f)).normalized();
    }
    syncQuaternionRaw();

    // Reset Jacobian
    Matrix<float, 15, 15> G = Matrix<float, 15, 15>::Identity();
    G.block<3,3>(6, 6) = Matrix3f::Identity() - skew(dth * 0.5f);
    _covP = G * _covP * G.transpose();
    _covP = 0.5f * (_covP + _covP.transpose()).eval();
}

// ─────────────────────────────────────────────────────────────────────────
// Sensor Updates
// ─────────────────────────────────────────────────────────────────────────

void ESEKF::updateGps(float pn, float pe, float pd, float vn, float ve, float vd, float hAcc, float vAcc) {
    Matrix<float, 6, 15> H = Matrix<float, 6, 15>::Zero();
    H.block<3,3>(0,0) = Matrix3f::Identity();
    H.block<3,3>(3,3) = Matrix3f::Identity();

    Vector<float, 6> y;
    y << pn-_p.x(), pe-_p.y(), pd-_p.z(), vn-_v.x(), ve-_v.y(), vd-_v.z();

    // 1. 기본 오차 가중치 (사용자 제안 3배)
    float multiplier = 3.0f;

    // 2. 가속도 기반 동적 스케일링 (4g 한계 대응)
    // 내부에 저장된 bias 보정된 specific force 크기 사용
    // 4g (약 39.2m/s^2)를 넘어가면 GPS 신뢰도를 급격히 낮춤
    const float G4_LIMIT = 4.0f * G_VAL;
    float high_g_penalty = 1.0f;
    if (_last_accel_mag > G4_LIMIT) {
        float excess = _last_accel_mag - G4_LIMIT;
        // 가중치를 제곱 비례로 증가 (4g 초과 시 매우 빠르게 무시됨)
        high_g_penalty = 1.0f + (excess * excess * 10.0f);
        if (high_g_penalty > 1000.0f) high_g_penalty = 1000.0f; // 최대 페널티 제한
    }

    float final_multiplier = multiplier * high_g_penalty;
    float var_h = (hAcc * final_multiplier) * (hAcc * final_multiplier);
    float var_v = (vAcc * final_multiplier) * (vAcc * final_multiplier);
    
    float var_vh = var_h * 0.2f;
    float var_vv = var_v * 0.2f;

    Matrix<float, 6, 6> R = Matrix<float, 6, 6>::Zero();
    R.diagonal() << var_h, var_h, var_v, var_vh, var_vh, var_vv;

    measurementUpdate(H, y, R);
}

void ESEKF::updateBaro(float alt) {
    Matrix<float, 1, 15> H = Matrix<float, 1, 15>::Zero();
    H(0, 2) = -1.0f;
    VectorXf y(1); y(0) = alt - (-_p.z());
    MatrixXf R(1,1); R(0,0) = _v_baro;
    measurementUpdate(H, y, R);
}

void ESEKF::updateMag(const float m_body[3]) {
    Vector3f mb(m_body[0], m_body[1], m_body[2]);
    if (mb.norm() < 1e-4f) return;
    mb.normalize();

    Vector3f z_pred = _q.toRotationMatrix().transpose() * _m_ref_ned;
    Vector3f y = mb - z_pred;

    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 6) = skew(z_pred);

    Matrix3f R = Matrix3f::Identity() * _v_mag;
    measurementUpdate(H, y, R);
}

void ESEKF::updateZupt() {
    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 3) = Matrix3f::Identity();
    Vector3f y = -_v;
    Matrix3f R = Matrix3f::Identity() * 1e-4f;
    measurementUpdate(H, y, R);
}

void ESEKF::updateAccStatic(const float a_m[3]) {
    Vector3f am(a_m[0], a_m[1], a_m[2]);
    Vector3f gravity_body = _q.toRotationMatrix().transpose() * (-_g_ned);
    Vector3f y = am - (gravity_body + _ba);

    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 6) = skew(gravity_body);
    H.block<3,3>(0, 9) = Matrix3f::Identity();

    Matrix3f R = Matrix3f::Identity() * (_v_acc / DT_IMU);
    measurementUpdate(H, y, R);
}

void ESEKF::updateGyroStatic(const float w_m[3]) {
    Vector3f wm(w_m[0], w_m[1], w_m[2]);
    Vector3f y = wm - _bg;
    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 12) = Matrix3f::Identity();
    Matrix3f R = Matrix3f::Identity() * (_v_gyro / DT_IMU);
    measurementUpdate(H, y, R);
}

// ─────────────────────────────────────────────────────────────────────────
// Alignment Loop
// ─────────────────────────────────────────────────────────────────────────

bool ESEKF::runZuptAlignment(IMUProvider imu_proc, float dt, float threshold, float max_time, MagProvider mag_proc) {
    float t = 0;
    int steps = round(0.1f / dt);
    while (t < max_time) {
        float am[3], wm[3];
        for (int i=0; i<steps; i++) { imu_proc(am, wm); predict(am, wm, dt); t += dt; }
        
        float P_before = _covP.diagonal().segment<9>(6).sum();
        updateZupt(); updateAccStatic(am); updateGyroStatic(wm);
        if (mag_proc) { float mm[3]; mag_proc(mm); updateMag(mm); }
        
        float P_after = _covP.diagonal().segment<9>(6).sum();
        if (P_before > 0 && fabsf(P_before - P_after) / P_before < threshold) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────
// Math Utilities
// ─────────────────────────────────────────────────────────────────────────

void ESEKF::syncQuaternionRaw() {
    _q_raw[0] = _q.w(); _q_raw[1] = _q.x(); _q_raw[2] = _q.y(); _q_raw[3] = _q.z();
}

Matrix3f ESEKF::skew(const Vector3f& v) {
    Matrix3f S;
    S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return S;
}

void ESEKF::quat2euler(const float q[4], float rpy[3]) {
    Quaternionf eig_q(q[0], q[1], q[2], q[3]);
    auto eul = eig_q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
    rpy[0] = eul[2]; rpy[1] = eul[1]; rpy[2] = eul[0];
}

void ESEKF::triad(const float acc[3], const float mag[3], float q_out[4], float lat, float lon) {
    Vector3f a(acc[0], acc[1], acc[2]); a.normalize();
    Vector3f m(mag[0], mag[1], mag[2]); m.normalize();

    Vector3f r1(0, 0, 1);
    Vector3f r2(0.5961f, -0.0838f, 0.7986f);
    if (!isnan(lat) && !isnan(lon)) {
        float d, i; wmmKorea(lat, lon, d, i);
        float dr = d*0.01745f, ir = i*0.01745f;
        r2 << cosf(ir)*cosf(dr), cosf(ir)*sinf(dr), sinf(ir);
    }

    auto build = [](const Vector3f& v1, const Vector3f& v2) {
        Vector3f u1 = v1;
        Vector3f u2 = u1.cross(v2).normalized();
        Vector3f u3 = u1.cross(u2);
        Matrix3f M; M << u1, u2, u3; return M;
    };

    Matrix3f R_bn = build(Vector3f(0,0,1), r2) * build(-a, m).transpose();
    Quaternionf q(R_bn);
    q_out[0] = q.w(); q_out[1] = q.x(); q_out[2] = q.y(); q_out[3] = q.z();
}

void ESEKF::wmmKorea(float lat, float lon, float &d, float &i) {
    d = -7.9f - 0.35f*(lat-36.0f) + 0.15f*(lon-127.0f);
    i = 51.0f + 1.40f*(lat-36.0f) + 0.05f*(lon-127.0f);
}
