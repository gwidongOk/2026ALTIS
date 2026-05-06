#include "NAV.h"

NAV::NAV() {
    state_acc = {0, 0.0f, 0.0f, 0.0f};
    state_gyro = {0, 0.0f, 0.0f, 0.0f};
    state_press = {0, 0.0f, 0.0f};
}

Raw_imu NAV::imu_axis(Raw_imu data) {
    Raw_imu aligned;
    // ★ 실제 보드 부착 방향에 맞춰 축을 맵핑하세요.
    aligned.timestamp = data.timestamp;
    aligned.x = data.y;
    aligned.y = data.x;
    aligned.z = -data.z;
    return aligned;
}

void NAV::updateAccel(Raw_imu raw) {
    raw_acc = imu_axis(raw);
    
    // 가속도는 1G 중력이 유지되어야 하므로 영점 보정을 뺍니다!
    state_acc.x = raw_acc.x * ACCEL_SCALE;
    state_acc.y = raw_acc.y * ACCEL_SCALE;
    state_acc.z = raw_acc.z * ACCEL_SCALE;
    state_acc.timestamp = raw.timestamp;
}

void NAV::updateGyro(Raw_imu raw) {
    raw_gyro = imu_axis(raw);
    
    // Raw 데이터에서 캘리브레이션 바이어스 빼기
    float corr_x = (float)raw_gyro.x - gyro_bias_x;
    float corr_y = (float)raw_gyro.y - gyro_bias_y;
    float corr_z = (float)raw_gyro.z - gyro_bias_z;

    state_gyro.x = corr_x * GYRO_SCALE;
    state_gyro.y = corr_y * GYRO_SCALE;
    state_gyro.z = corr_z * GYRO_SCALE;
    state_gyro.timestamp = raw.timestamp;
}

void NAV::updatePress(Raw_press press) {
    state_press.pressure = press.p;
    state_press.altitude = getAltitude(press.p);
    state_press.timestamp = press.timestamp;
}

float NAV::getAltitude(float current_pressure) {
    float absolute_alt = 44330.0f * (1.0f - pow(current_pressure / 101325.0f, 0.1903f));
    return absolute_alt - _padAltitude;
}

// =======================================================
// 초기 캘리브레이션 세팅
// =======================================================
void NAV::calibrateGyro(float sum_x, float sum_y, float sum_z, int samples) {
    Raw_imu sum_aligned = imu_axis({0, (int16_t)sum_x, (int16_t)sum_y, (int16_t)sum_z});
    gyro_bias_x = sum_aligned.x / (float)samples;
    gyro_bias_y = sum_aligned.y / (float)samples;
    gyro_bias_z = sum_aligned.z / (float)samples;
}

void NAV::calibratePress(float sum_p, int samples) {
    float avg_press = sum_p / (float)samples;
    _padAltitude = 44330.0f * (1.0f - pow(avg_press / 101325.0f, 0.1903f));
}

RocketState_imu NAV::getState_acc() { return state_acc; }
RocketState_imu NAV::getState_gyro() { return state_gyro; }
RocketState_PRESS NAV::getState_press() { return state_press; }

Raw_imu NAV::getraw_acc() { return raw_acc; }
Raw_imu NAV::getraw_gyro() { return raw_gyro; }