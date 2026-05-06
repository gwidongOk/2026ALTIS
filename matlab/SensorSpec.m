classdef SensorSpec
%SENSORSPEC  센서 스펙 (ESEKF, generate_sim_data 공통 참조)
%   값 출처: 각 센서 데이터시트
%   Q/R 튜닝 시 이 값을 기준으로 스케일링

    properties (Constant)
        % 샘플주기
        dt_imu  (1,1) double = 1/416     % [s] IMU (LSM6DSO32, ODR 416Hz)
        dt_gps  (1,1) double = 1/25      % [s] GPS (NEO-M9N, 25Hz)
        dt_baro (1,1) double = 1/50      % [s] 기압 고도계 (BMP388, 50Hz)
        dt_mag  (1,1) double = 1/50      % [s] 지자기 (MMC5983MA, 50Hz)

        % IMU (LSM6DSO32) — 노이즈 cut off: 가속도 41.6 Hz , gyro 135.9 Hz
        %검색 결과 일반적인 필터에서는 cut off를 신경안씀 -> 추후 튜닝함
        
        var_acc   (1,1) double =  2.729445e-4 % (m/s²)²·s  가속도 노이즈 밀도² (PSD)
        var_gyro  (1,1) double =   8.061781e-7  % (rad/s)²·s  자이로 노이즈 밀도² (PSD)
        var_ba    (1,1) double = (0.001)^2              % (m/s²)²·s  가속도 바이어스 랜덤워크 PSD
        var_bg    (1,1) double = (0.0001)^2             % (rad/s)²·s  자이로 바이어스 랜덤워크 PSD

        % GPS (NEO-M9N) — 분산 (σ²)
        var_gps_pos_h (1,1) double = 0.694^2       % [m²] 수평 위치 (CEP 1.5m)
        var_gps_pos_v (1,1) double = 1.282927^2    % [m²] 수직 위치 (수평 × 2)
        var_gps_vel_h (1,1) double = 0.2684742^2   % [(m/s)²] 수평 속도
        var_gps_vel_v (1,1) double = 0.2684742^2   % [(m/s)²] 수직 속도

        % 기압 고도계 (BMP388) — 기압 RMS 2.3 Pa → 고도 노이즈 0.191 m
        var_baro  (1,1) double = 4.920342e-2     % [m]

        % 지자기 (MMC5983MA) — 단위벡터 기준 노이즈
        var_mag     (1,1) double = (1.963/500)^2 %0.02^2  % [-] (0.04µT RMS / 지구자기장 ~50µT)^2 =0.0008²

        % 물리 상수
        g_ned       (3,1) double = [0; 0; 9.81]    % 중력벡터 NED [m/s²]

        % 지자기 기준벡터 NED (한국: 복각 53°, 편각 -8°)
        m_ref_ned   (3,1) double = [0.5961; -0.0838; 0.7986]
    end

end
