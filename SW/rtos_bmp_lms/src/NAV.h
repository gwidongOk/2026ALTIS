#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>

struct Raw_imu {
    unsigned long timestamp;
    int16_t x, y, z;
};

struct RocketState_imu {
    unsigned long timestamp;
    float x, y, z; 
};

struct Raw_press {
    unsigned long timestamp;
    float p;
};

struct RocketState_PRESS {
    unsigned long timestamp;
    float pressure; 
    float altitude; 
};

class NAV {
private:
    Raw_imu raw_acc, raw_gyro;
    RocketState_imu state_acc, state_gyro;
    RocketState_PRESS state_press;

    // imu 센서 부착 방향에 맞춘 축 변환
    Raw_imu imu_axis(Raw_imu data);

    const float ACCEL_SCALE = 0.976f * 0.001f * 9.80665f; 
    const float GYRO_SCALE = 70.0f * 0.001f * (M_PI / 180.0f); 

    float _padAltitude = 0.0f; 
    float getAltitude(float current_pressure);

    // ★ 자이로 바이어스는 음수일 수 있으므로 반드시 float 형태를 유지해야 합니다.
    float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f; 

public:
    NAV();
    
    // 센서 Task에서 호출하는 업데이트 함수
    void updateAccel(Raw_imu raw);
    void updateGyro(Raw_imu raw);
    void updatePress(Raw_press press);
    
    // main.cpp의 setup에서 한 번에 캘리브레이션을 처리하는 함수
    void calibrateGyro(float sum_x, float sum_y, float sum_z, int samples);
    void calibratePress(float sum_p, int samples);
    
    // NAV_Task에서 데이터를 꺼내갈 때 사용
    RocketState_imu getState_acc();
    RocketState_imu getState_gyro();
    RocketState_PRESS getState_press();
    Raw_imu getraw_acc();
    Raw_imu getraw_gyro();
};

#endif