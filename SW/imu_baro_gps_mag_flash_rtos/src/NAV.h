#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>

#include "sensor_data.h"
#include "ESEKF.h"

class NAV {
private:
    Raw_imu       _raw_imu   = {};
    State_imu     _state_imu = {};
    Raw_press     _press     = {};
    Raw_mag       _mag       = {};
    Raw_gps       _gps       = {};
    State_nominal _nominal   = {};

    ESEKF _ekf;
    bool  _ekf_ready = false;

    // Time tracking for IMU dt
    int64_t _last_imu_time_us = 0;

    // LSM6DSO32 ±32g / ±2000dps sensitivities
    // (LSM6DSO32::readCalibratedIMU returns int16 in SENSOR LSBs after axis remap)
    static constexpr float ACCEL_SCALE = 0.976f * 0.001f * 9.80665f;
    static constexpr float GYRO_SCALE  = 70.0f  * 0.001f * (M_PI / 180.0f);

    void syncNominal();

public:
    NAV();

    // Sensor task callbacks — copy + (for IMU) convert to SI
    void updateIMU(const Raw_imu &raw);
    void updatePress(const Raw_press &p);
    void updateMag(const Raw_mag &m);
    void updateGps(const Raw_gps &g);

    // ===== ES-EKF lifecycle =====
    // Initialize from latest sensor data (TRIAD on current IMU+Mag).
    // Position/velocity initialized from GPS NED if available, else zero.
    // Returns false if accel or mag readings look invalid.
    bool ekfBegin();
    // Initialize with explicit nominal state.
    void ekfBegin(const float p0[3], const float v0[3], const float q0[4]);
    void ekfReset();
    bool isEkfReady() const { return _ekf_ready; }

    // ===== ES-EKF cycle =====
    void ekfPredict(float dt);
    void ekfUpdateBaro();
    void ekfUpdateMag();
    void ekfUpdateGps();

    // Pad-stationary pseudo-measurements (call only when truly at rest).
    // Consolidated: performs ZUPT, AccStatic, GyroStatic, and Mag update.
    void ekfUpdateStaticAlignment();

    // Tuning passthrough
    ESEKF&       ekf()       { return _ekf; }
    const ESEKF& ekf() const { return _ekf; }

    // Accessors
    Raw_imu       getRawImu()   const { return _raw_imu;   }
    State_imu     getStateImu() const { return _state_imu; }
    Raw_press     getPress()    const { return _press;     }
    Raw_mag       getMag()      const { return _mag;       }
    Raw_gps       getGps()      const { return _gps;       }
    State_nominal getNominal()  const { return _nominal;   }
};

#endif
