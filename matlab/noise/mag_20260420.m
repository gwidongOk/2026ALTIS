
% 단위
t_ms = t /1e+6;

% 평균
mean_mx = mean(mx);
mean_my = mean(my);
mean_mz = mean(mz);

% 분산
var_mx = var(mx);
var_my = var(my);
var_mz = var(mz);

% 결과 출력
fprintf('평균 지자기 X = %e, Y = %e, Z = %e\n', mean_mx, mean_my, mean_mz);
fprintf('분산 지자기 X = %e, Y = %e, Z = %e\n', var_mx, var_my, var_mz);

figure;
subplot(3,1,1);
plot(t_ms, mx);
xlabel('time [sec]'); ylabel('mag_X [G]');
title(sprintf('mag\n Mean: x: %e, y: %e, z: %e \n Var: x: %e, y: %e, z: %e ', ...
    mean_mx,mean_my,mean_mz,var_mx,var_my,var_mz));
grid on;
subplot(3,1,2);
plot(t_ms, my);
xlabel('time [sec]'); ylabel('mag_Y [G]');
grid on;
subplot(3,1,3);
plot(t_ms, mz);
xlabel('time [sec]'); ylabel('mag_Z [G]');

