#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>
#include <ArduinoEigen.h>

#include "sensor_data.h"
#include "KF1D.h"

class NAV {
private:
    Raw_imu       _raw_imu   = {};
    State_imu     _state_imu = {};
    Raw_press     _press     = {};
    State_nominal _nominal   = {};

    KF1D _kf;
    bool _kf_ready = false;

    // Attitude (gyro integration)
    Eigen::Quaternionf _q = Eigen::Quaternionf::Identity();

    // NE position/velocity (dead-reckoning by accel integration)
    Eigen::Vector2f _pos_ne = Eigen::Vector2f::Zero();
    Eigen::Vector2f _vel_ne = Eigen::Vector2f::Zero();

    // Gravity in NED Down [m/s^2]
    const Eigen::Vector3f _g_ned = Eigen::Vector3f(0.0f, 0.0f, 9.80665f);

    int64_t _last_imu_time_us = 0;

    // Sensitivity scales
    static constexpr float ACCEL_SCALE = 0.976f * 0.001f * 9.80665f;
    static constexpr float GYRO_SCALE  = 70.0f  * 0.001f * (M_PI / 180.0f);

    void syncNominal();
    void integrateQuaternion(const Eigen::Vector3f& w, float dt);

public:
    NAV();

    // Task callbacks
    void updateIMU(const Raw_imu &raw);
    void updatePress(const Raw_press &p);

    // Lifecycle
    bool kfBegin();
    void kfReset();
    bool isKfReady() const { return _kf_ready; }

    // Accessors
    Raw_imu       getRawImu()   const { return _raw_imu;   }
    State_imu     getStateImu() const { return _state_imu; }
    Raw_press     getPress()    const { return _press;     }
    State_nominal getNominal()  const { return _nominal;   }

    void quatToEuler(float qw, float qx, float qy, float qz,float &roll, float &pitch, float &yaw);
};
#endif
