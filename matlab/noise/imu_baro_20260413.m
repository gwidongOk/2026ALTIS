
% 단위
t_imu_s = t_imu /1e+6;
t_baro_s = t_baro /1e+6;
acc_x_mps2 = rawacc_x * 0.976 * 0.001 * 9.80665;
acc_y_mps2 = rawacc_y * 0.976 * 0.001 * 9.80665;
acc_z_mps2 = rawacc_z * 0.976 * 0.001 * 9.80665;

gyro_x_radps = rawgyro_x * 70 * 0.001 * (pi/180);
gyro_y_radps = rawgyro_y * 70 * 0.001 * (pi/180);
gyro_z_radps = rawgyro_z * 70 * 0.001 * (pi/180);

%저주파 제거
%detrended_alt = detrend(rawalt);
detrended_alt = rawalt;
% 평균
mean_acc_x = mean(acc_x_mps2);
mean_acc_y = mean(acc_y_mps2);
mean_acc_z = mean(acc_z_mps2);

mean_gyro_x = mean(gyro_x_radps);
mean_gyro_y = mean(gyro_y_radps);
mean_gyro_z = mean(gyro_z_radps);

mean_alt = mean(detrended_alt);

% 분산
var_acc_x = var(acc_x_mps2);
var_acc_y = var(acc_y_mps2);
var_acc_z = var(acc_z_mps2);

var_gyro_x = var(gyro_x_radps);
var_gyro_y = var(gyro_y_radps);
var_gyro_z = var(gyro_z_radps);

var_alt = var(detrended_alt);

% 결과 출력
fprintf('평균 가속도 X = %e, Y = %e, Z = %e\n', mean_acc_x, mean_acc_y, mean_acc_z);
fprintf('평균 자이로 X = %e, Y = %e, Z = %e\n', mean_gyro_x, mean_gyro_y, mean_gyro_z);
fprintf('평균 고도 = %e\n', mean_alt);
fprintf('분산 가속도 X = %e, Y = %e, Z = %e\n', var_acc_x, var_acc_y, var_acc_z);
fprintf('분산 자이로 X = %e, Y = %e, Z = %e\n', var_gyro_x, var_gyro_y, var_gyro_z);
fprintf('분산 고도 = %e\n', var_alt);

figure;
plot(t_imu_s, acc_x_mps2-mean_acc_x);
hold on;
plot(t_imu_s, acc_y_mps2-mean_acc_y);
plot(t_imu_s, acc_z_mps2-mean_acc_z);
grid on;
hold off;
legend('acc_x','acc_y','acc_z');
xlabel('time [sec]'); ylabel('Accel [m/s^2]');
title(sprintf('Accel\n Mean: x: %e, y: %e, z: %e \n Var: x: %e, y: %e, z: %e ', ...
    mean_acc_x,mean_acc_y,mean_acc_z,var_acc_x,var_acc_y,var_acc_z));

figure;
plot(t_imu_s, gyro_x_radps-mean_gyro_x);
hold on;
plot(t_imu_s, gyro_y_radps-mean_gyro_y);
plot(t_imu_s, gyro_z_radps-mean_gyro_z);
grid on;
hold off;
title(sprintf('Gyro\n Mean: x: %e, y: %e, z: %e \n Var: x: %e, y: %e, z: %e ', ...
    mean_gyro_x,mean_gyro_y,mean_gyro_z,var_gyro_x,var_gyro_y,var_gyro_z));
legend('gyro_x','gyro_y','gyro_z');
xlabel('time [sec]'); ylabel('gyro [rad/s]');

figure;
plot(t_baro_s, detrended_alt);
grid on;
xlabel('time [sec]'); ylabel('altitude [m]');
title(sprintf('Baro\n Mean: %.3f \n Var: %.3f', ...
    mean_alt,var_alt));