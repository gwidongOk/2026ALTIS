#include "NAV.h"

NAV::NAV() {
    state_imu   = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    state_press = {0, 0.0f, 0.0f};
}

Raw_imu NAV::axis(Raw_imu data) {
    Raw_imu aligned;
    aligned.timestamp = data.timestamp;
    aligned.ax =  data.ay;
    aligned.ay =  data.ax;
    aligned.az = -data.az;
    aligned.gx =  data.gy;
    aligned.gy =  data.gx;
    aligned.gz = -data.gz;
    return aligned;
}

void NAV::updateIMU(Raw_imu raw) {
    raw_imu = axis(raw);

    // 가속도 캘리브레이션 보정 + 스케일 변환
    state_imu.ax = ((float)raw_imu.ax - c_accel_x) * ACCEL_SCALE;
    state_imu.ay = ((float)raw_imu.ay - c_accel_y) * ACCEL_SCALE;
    state_imu.az = ((float)raw_imu.az - c_accel_z) * ACCEL_SCALE;

    // 자이로 캘리브레이션 보정 + 스케일 변환
    state_imu.gx = ((float)raw_imu.gx - c_gyro_x) * GYRO_SCALE;
    state_imu.gy = ((float)raw_imu.gy - c_gyro_y) * GYRO_SCALE;
    state_imu.gz = ((float)raw_imu.gz - c_gyro_z) * GYRO_SCALE;

    state_imu.timestamp = raw.timestamp;
}

void NAV::updatePress(Raw_press press) {
    state_press.pressure = press.p;
    state_press.altitude = getAltitude(press.p);
    state_press.timestamp = press.timestamp;
}

float NAV::getAltitude(float current_pressure) {
    return 44330.0f * (1.0f - pow(current_pressure / _padpressure, 0.1903f));
}

// =======================================================
// 캘리브레이션
// =======================================================
void NAV::calibrate(float c_gx, float c_gy, float c_gz,
                    float c_ax, float c_ay, float c_az, float c_p) {
    // 축 변환 적용 (axis 매핑: x←y, y←x, z←-z)
    c_accel_x =  c_ay;
    c_accel_y =  c_ax;
    c_accel_z = -c_az;

    c_gyro_x =  c_gy;
    c_gyro_y =  c_gx;
    c_gyro_z = -c_gz;

    _padpressure = c_p;
}

RocketState_imu NAV::getState_imu()    { return state_imu; }
RocketState_PRESS NAV::getState_press() { return state_press; }

Raw_imu NAV::getRaw_imu() { return raw_imu; }
