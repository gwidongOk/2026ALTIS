%% ========================================================================
%  [로켓 비행 컴퓨터] GPS 센서 실시간 자체 오차 기반 분석 스크립트
%  - 실측 분산 계산 제거, 오직 센서 리포트(hAcc, vAcc, sAcc)에 의존
% ========================================================================
clc; close all;

% === 1. 시간 단위 변환 (ms -> s) ===
time_s = time_ms / 1e3;

% === 2. 튜닝 배수 (Scale Factor) 설정 ===
tune_scale = 3.0; 

% --- 1. 그래프 중앙 정렬을 위한 기준점(평균) 계산 ---
mean_N_p = mean(N_p, 'omitnan'); mean_E_p = mean(E_p, 'omitnan'); mean_D_p = mean(D_p, 'omitnan');
mean_N_s = mean(N_s, 'omitnan'); mean_E_s = mean(E_s, 'omitnan'); mean_D_s = mean(D_s, 'omitnan');

% --- 2. GPS 센서가 실시간으로 출력한 표준편차에 배수(Scale) 적용 ---
scaled_h_std_array = sqrt(var_h) * tune_scale;
scaled_v_std_array = sqrt(var_v) * tune_scale;
scaled_s_std_array = sqrt(var_s);

scaled_h_std_mean = mean(scaled_h_std_array);
scaled_v_std_mean = mean(scaled_v_std_array);
scaled_s_std_mean = mean(scaled_s_std_array);

fprintf("scaled_h_std_mean %e \n",scaled_h_std_mean);
fprintf("scaled_v_std_mean %e \n",scaled_v_std_mean);
fprintf("scaled_s_std_mean %e \n",scaled_s_std_mean);

% [Figure 1] 위치(Position) NED 데이터 vs 실시간 튜닝 바운더리
fig_pos = figure('Name', 'Position Drift vs Scaled Sensor Bounds', 'Position', [100, 100, 1000, 800]);

% N축
subplot(3,1,1);
plot(time_s, N_p - mean_N_p, 'r', 'LineWidth', 2, 'DisplayName', 'N축 실제 변동'); hold on;
plot(time_s, scaled_h_std_array, 'k--', 'LineWidth', 2, 'DisplayName', '+1\sigma (hAcc * Scale)');
plot(time_s, -scaled_h_std_array, 'k--', 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; hold off;
ylabel('North [m]'); title(sprintf('NED Position Drift vs Sensor Estimated Bounds (Scale: %.1f)', tune_scale));
legend('show', 'Location', 'best');

% E축
subplot(3,1,2);
plot(time_s, E_p - mean_E_p, 'g', 'LineWidth', 2, 'DisplayName', 'E축 실제 변동'); hold on;
plot(time_s, scaled_h_std_array, 'k--', 'LineWidth', 2, 'DisplayName', '+1\sigma (hAcc * Scale)');
plot(time_s, -scaled_h_std_array, 'k--', 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; hold off;
ylabel('East [m]');
legend('show', 'Location', 'best');

% D축 (고도)
subplot(3,1,3);
plot(time_s, D_p - mean_D_p, 'b', 'LineWidth', 2, 'DisplayName', 'D축 실제 변동'); hold on;
plot(time_s, scaled_v_std_array, 'k--', 'LineWidth', 2, 'DisplayName', '+1\sigma (vAcc * Scale)');
plot(time_s, -scaled_v_std_array, 'k--', 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; hold off;
xlabel('Time [sec]'); ylabel('Down [m]');
legend('show', 'Location', 'best');

linkaxes(findobj(fig_pos, 'type', 'axes'), 'x');


% [Figure 2] 속도(Velocity) NED 데이터 vs 실시간 튜닝 바운더리
fig_vel = figure('Name', 'Velocity Noise vs Scaled Sensor Bounds', 'Position', [150, 150, 1000, 800]);

% Vn 속도
subplot(3,1,1);
plot(time_s, N_s, 'r', 'LineWidth', 2, 'DisplayName', 'Vn 실제 속도'); hold on;
plot(time_s, scaled_s_std_array, 'k--', 'LineWidth', 2, 'DisplayName', '+1\sigma (sAcc * Scale)');
plot(time_s, -scaled_s_std_array, 'k--', 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; hold off;
ylabel('Vn [m/s]'); title(sprintf('NED Velocity Noise vs Sensor Estimated Bounds (Scale: %.1f)', tune_scale));
legend('show', 'Location', 'best');

% Ve 속도
subplot(3,1,2);
plot(time_s, E_s, 'g', 'LineWidth', 2, 'DisplayName', 'Ve 실제 속도'); hold on;
plot(time_s, scaled_s_std_array, 'k--', 'LineWidth', 2, 'DisplayName', '+1\sigma (sAcc * Scale)');
plot(time_s, -scaled_s_std_array, 'k--', 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; hold off;
ylabel('Ve [m/s]');
legend('show', 'Location', 'best');

% Vd 속도
subplot(3,1,3);
plot(time_s, D_s, 'b', 'LineWidth', 2, 'DisplayName', 'Vd 실제 속도'); hold on;
plot(time_s, scaled_s_std_array, 'k--', 'LineWidth', 2, 'DisplayName', '+1\sigma (sAcc * Scale)');
plot(time_s, -scaled_s_std_array, 'k--', 'LineWidth', 2, 'HandleVisibility', 'off');
grid on; hold off;
xlabel('Time [s]'); ylabel('Vd [m/s]');
legend('show', 'Location', 'best');

linkaxes(findobj(fig_vel, 'type', 'axes'), 'x');


% [Figure 3] 2D 수평 궤적 산점도 (Scatter Plot)
figure('Name', '2D Scatter with Scaled Bound', 'Position', [200, 200, 700, 700]);
scatter(E_p - mean_E_p, N_p - mean_N_p, 15, 'filled', 'MarkerFaceAlpha', 0.6, 'DisplayName', 'Measured Path'); hold on;

% 배수가 적용된 평균 수평 오차 원 그리기
mean_scaled_h_std = mean(scaled_h_std_array, 'omitnan');
theta = linspace(0, 2*pi, 100);
plot(mean_scaled_h_std * sin(theta), mean_scaled_h_std * cos(theta), 'k--', 'LineWidth', 2, 'DisplayName', sprintf('Scaled Bound (%.1f)', tune_scale));

grid on; axis equal; hold off;
xlabel('East Error [m]'); ylabel('North Error [m]'); 
title(sprintf('GPS 2D Horizontal Scatter Plot (Scale: %.1f)', tune_scale));
legend('show', 'Location', 'best');