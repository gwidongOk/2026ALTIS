#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>

#include "sensor_data.h"
#include "KF1D.h"

class NAV {
private:
    Raw_imu       _raw_imu   = {};
    State_imu     _state_imu = {};
    Raw_press     _press     = {};
    State_nominal _nominal   = {};

    KF1D _kf;
    bool  _kf_ready = false;

    // Time tracking for IMU dt
    int64_t _last_imu_time_us = 0;

    // LSM6DSO32 ±32g / ±2000dps sensitivities
    static constexpr float ACCEL_SCALE = 0.976f * 0.001f * 9.80665f;
    static constexpr float GYRO_SCALE  = 70.0f  * 0.001f * (M_PI / 180.0f);

    void syncNominal();

public:
    NAV();

    // Sensor task callbacks
    void updateIMU(const Raw_imu &raw);
    void updatePress(const Raw_press &p);

    // ===== KF lifecycle =====
    bool ekfBegin();
    void ekfReset();
    bool isEkfReady() const { return _kf_ready; }

    // ===== KF cycle =====
    void ekfPredict(float dt);
    void ekfUpdateBaro();

    // Accessors
    Raw_imu       getRawImu()   const { return _raw_imu;   }
    State_imu     getStateImu() const { return _state_imu; }
    Raw_press     getPress()    const { return _press;     }
    State_nominal getNominal()  const { return _nominal;   }
};

#endif

