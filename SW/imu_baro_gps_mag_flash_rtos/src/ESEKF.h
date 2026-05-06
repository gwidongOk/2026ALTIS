#ifndef ESEKF_H
#define ESEKF_H

#include <Arduino.h>
#include <ArduinoEigen.h>

// ─────────────────────────────────────────────────────────────────────────
// ESEKF — 통합형(Self-contained) 15-state Error-State EKF
//   - SensorSpec (상수) 및 NavMath (유틸리티) 기능 포함
// ─────────────────────────────────────────────────────────────────────────

class ESEKF {
public:
    static constexpr int N_STATE = 15;

    // ===== [SensorSpec 통합] 하드웨어 상수 및 노이즈 파라미터 ==========
    static constexpr float G_VAL     = 9.80665f;
    static constexpr float DT_IMU    = 1.0f / 416.0f;
    static constexpr float VAR_ACC   = 2.729445e-4f;
    static constexpr float VAR_GYRO  = 8.061781e-7f;
    static constexpr float VAR_BA    = 1.0e-6f;
    static constexpr float VAR_BG    = 1.0e-8f;

    static constexpr float VAR_BARO      = 4.920342e-2f;
    static constexpr float VAR_GPS_POS_H = 0.694f * 0.694f;
    static constexpr float VAR_GPS_POS_V = 1.282927f * 1.282927f;
    static constexpr float VAR_GPS_VEL_H = 0.2684742f * 0.2684742f;
    static constexpr float VAR_GPS_VEL_V = 0.2684742f * 0.2684742f;
    static constexpr float VAR_MAG       = 1.5e-5f;

    // ===== Filter Lifecycle ============================================
    ESEKF();
    void reset();
    void init(const float p0[3], const float v0[3], const float q0[4]);

    // ===== TRIAD Alignment =============================================
    static void triad(const float acc[3], const float mag[3], float q_out[4], 
                      float lat_deg = NAN, float lon_deg = NAN);

    // ===== Filter Cycle ================================================
    void predict(const float a_m[3], const float w_m[3], float dt);
    void updateGps(float pn, float pe, float pd, float vn, float ve, float vd, float hAcc, float vAcc);
    void updateBaro(float altitude_m);
    void updateMag(const float m_body[3]);
    void updateZupt();
    void updateAccStatic(const float a_m[3]);
    void updateGyroStatic(const float w_m[3]);

    // ===== 자동 정지 정렬 (ZUPT Loop) ==================================
    typedef void (*IMUProvider)(float*, float*);
    typedef void (*MagProvider)(float*);
    bool runZuptAlignment(IMUProvider imu_proc, float dt, float threshold = 1e-4f, 
                          float max_time = 30.0f, MagProvider mag_proc = nullptr);

    // ===== Accessors ===================================================
    const float* position()   const { return _p.data(); }
    const float* velocity()   const { return _v.data(); }
    const float* quaternion() const { return _q_raw;    }
    const float* accelBias()  const { return _ba.data(); }
    const float* gyroBias()   const { return _bg.data(); }
    const float* covariance() const { return _covP.data();  }

    // 자세(δθ) + 바이어스(δba, δbg) 9개 대각 성분 합 — ZUPT 수렴 판정용
    float attBiasCovTrace() const { return _covP.diagonal().segment<9>(6).sum(); }

    // ===== [NavMath 통합] 수학 유틸리티 (Static) =======================
    static void quat2euler(const float q[4], float rpy[3]);
    static Eigen::Matrix3f skew(const Eigen::Vector3f& v);

private:
    // Nominal State
    Eigen::Vector3f _p;
    Eigen::Vector3f _v;
    Eigen::Quaternionf _q;
    float _q_raw[4]; // 외부 반환용 [w,x,y,z]
    Eigen::Vector3f _ba;
    Eigen::Vector3f _bg;

    // Error Covariance
    Eigen::Matrix<float, 15, 15> _covP;

    // Runtime Parameters (초기화 시 스펙값 로드, 런타임 튜닝 가능)
    float _v_acc, _v_gyro, _v_ba, _v_bg;
    float _v_gps_ph, _v_gps_pv, _v_gps_vh, _v_gps_vv;
    float _v_baro, _v_mag;

    Eigen::Vector3f _g_ned;
    Eigen::Vector3f _m_ref_ned;

    // 최근 predict의 bias 보정된 specific force 크기 [m/s²] — high-G GPS 페널티용
    float _last_accel_mag;

    // Internal Helpers
    void measurementUpdate(const Eigen::MatrixXf& H, const Eigen::VectorXf& y, const Eigen::MatrixXf& R);
    void syncQuaternionRaw();
    static void wmmKorea(float lat, float lon, float &d, float &i);
};

#endif
