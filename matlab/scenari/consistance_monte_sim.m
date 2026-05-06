clear; clc; close all;
addpath('..');

% ═══════════════════════════════════════════════════════════════════════
% 설정
% ═══════════════════════════════════kwnf════════════════════════════════════
CSV_PATH   = 'openrocket.csv';
N_TRIALS   = 200; % 몬테카를로 반복 횟수
BASE_SEED  = 42; % 난수 생성기 시드(seed) 기준값

S  = SensorSpec;
dt = S.dt_imu;          % 416 Hz → dt ≈ 2.4e-3 s

% OpenRocket CSV 파싱
raw = readmatrix(CSV_PATH, 'CommentStyle', '#');
t_or      = raw(:, 1);
acc_or    = raw(:, 8:10);     % [X, Y, Z]_OR body acc (m/s²)
rate_or   = raw(:, 15:17);    % [Roll(Z), Pitch(Y), Yaw(X)]_OR (deg/s)
zenith0   = raw(1, 18);       % deg
azimuth0  = raw(1, 19);       % deg

% MATLAB Body Frame 정렬
acc_mat  = [acc_or(:,3),  acc_or(:,1),  acc_or(:,2)];  % [m/s²]
rate_mat = deg2rad([rate_or(:,1), rate_or(:,3), rate_or(:,2)]);  % [rad/s]

t_end = t_or(end);
t     = (0 : dt : t_end)';
N     = length(t);

acc_u  = interp1(t_or, acc_mat,  t, 'linear', 'extrap');   % [N×3]
rate_u = interp1(t_or, rate_mat, t, 'linear', 'extrap');   % [N×3]

% ═══════════════════════════════════════════════════════════════════════
% 초기 자세 (zenith, azimuth at t=0)
% ═══════════════════════════════════════════════════════════════════════
yaw_init   = deg2rad(azimuth0);
pitch_init = -deg2rad(zenith0);
roll_init  = 0;
q_init = quaternion([yaw_init, pitch_init, roll_init], 'euler', 'ZYX', 'frame');

% ═══════════════════════════════════════════════════════════════════════
% kinematicTrajectory → Ground Truth
% ═══════════════════════════════════════════════════════════════════════
traj = kinematicTrajectory('SampleRate', round(1/dt), ...
        'Position', [0 0 0], 'Velocity', [0 0 0], 'Orientation', q_init);
[pos_ned, orient, vel_ned] = traj(acc_u, rate_u);

p_true = pos_ned';                  % [3×N] NED
v_true = vel_ned';                  % [3×N] NED
q_true = compact(orient)';          % [4×N] [w; x; y; z]

fprintf('=== ES-EKF 일관성 검증 (OpenRocket truth) ===\n');
fprintf('  시뮬 시간 : %.2f s  (N = %d 샘플, dt = %.4e s)\n', t_end, N, dt);
fprintf('  최대고도  : %.1f m\n', -min(-p_true(3,:)));
fprintf('  반복횟수  : %d trial\n\n', N_TRIALS);

% ═══════════════════════════════════════════════════════════════════════
% 측정 인덱스 (GPS / Baro / Mag) 및 참(Truth) 센서 데이터 생성
% ═══════════════════════════════════════════════════════════════════════
gps_idx  = 1 : round(S.dt_gps  / dt) : N;
baro_idx = 1 : round(S.dt_baro / dt) : N;
mag_idx  = 1 : round(S.dt_mag  / dt) : N;
m_ref_unit = S.m_ref_ned / norm(S.m_ref_ned);

% ── 참 센서 바이어스 (flight 상수) ────────────────────────────────────
true_ba = [0.05; -0.02;  0.01  ];      % [m/s²]
true_bg = [0.001; -0.002; 0.003];      % [rad/s]

% ── 동역학적 가속도를 관성센서 비력(Specific Force)으로 변환 ───
g_ned = [0; 0; 9.80665]; % NED 기준 중력
f_body_true = zeros(3, N);
for k = 1:N
    R_bn = NavUtils.quat2dcm(q_true(:, k));
    g_body = R_bn' * g_ned; % NED 중력을 Body 프레임으로 투영
    f_body_true(:, k) = acc_u(k, :)' - g_body; % 비력 = 가속도 - 중력
end
w_body_true = rate_u';       % [3×N]

% ═══════════════════════════════════════════════════════════════════════
% 결과 저장 배열
% ═══════════════════════════════════════════════════════════════════════
err_p   = zeros(N_TRIALS, N);
err_v   = zeros(N_TRIALS, N);
err_ba  = zeros(N_TRIALS, N);
err_bg  = zeros(N_TRIALS, N);
nees_pv = zeros(N_TRIALS, N);

% [추가] Trial 1의 상태 기록 (시각화용)
state_p_sim  = zeros(3, N);
state_v_sim  = zeros(3, N);
state_q_sim  = zeros(4, N);
state_ba_sim = zeros(3, N);
state_bg_sim = zeros(3, N);

% ── 필터의 오차 공분산(Variance) 합을 저장할 배열 ──
cov_p   = zeros(N_TRIALS, N);
cov_v   = zeros(N_TRIALS, N);
cov_ba  = zeros(N_TRIALS, N);
cov_bg  = zeros(N_TRIALS, N);

% ═══════════════════════════════════════════════════════════════════════
% Monte Carlo 루프
% ═══════════════════════════════════════════════════════════════════════
fprintf('시뮬레이션 진행 중...\n');
for trial = 1:N_TRIALS
    % 진행 상황 출력 (10회마다)
    if mod(trial, 10) == 0 || trial == 1
        fprintf('  Trial %d / %d ...\n', trial, N_TRIALS);
    end
    
    rng(BASE_SEED + trial);
    
    std_acc  = sqrt(S.var_acc  / dt); 
    std_gyro = sqrt(S.var_gyro / dt); 
    
    a_meas = f_body_true + true_ba + std_acc  * randn(3, N); 
    w_meas = w_body_true + true_bg + std_gyro * randn(3, N); 
    
    sig_p = sqrt([S.var_gps_pos_h; S.var_gps_pos_h; S.var_gps_pos_v]);
    sig_v = sqrt([S.var_gps_vel_h; S.var_gps_vel_h; S.var_gps_vel_v]);
    z_gps = [p_true(:,gps_idx) + sig_p .* randn(3, length(gps_idx));
             v_true(:,gps_idx) + sig_v .* randn(3, length(gps_idx))];
             
    z_baro = -p_true(3, baro_idx) + sqrt(S.var_baro) * randn(1, length(baro_idx));
    
    z_mag = zeros(3, length(mag_idx));
    for k = 1:length(mag_idx)
        R_nb = NavUtils.quat2dcm(q_true(:, mag_idx(k)));
        z_mag(:,k) = R_nb' * m_ref_unit + sqrt(S.var_mag) * randn(3,1);
    end
    
    % ── 1. TRIAD를 이용한 초기 자세 결정 ──────────────────────────────
    R_bn_true0 = NavUtils.quat2dcm(q_true(:,1));
    f_static   = R_bn_true0' * (-g_ned); % 발사대에서의 참 비력
    m_static   = R_bn_true0' * m_ref_unit; % 발사대에서의 참 지자기
    
    acc_meas_triad = f_static + true_ba + std_acc * randn(3,1);
    mag_meas_triad = m_static + sqrt(S.var_mag) * randn(3,1);
    
    q0 = ESEKF.run_triad(acc_meas_triad, mag_meas_triad);
    
    % ── 2. EKF 객체 생성 및 초기화 ────────────────────────────────────
    p0 = p_true(:,1) + [0.5; 0.5; 1.0] .* randn(3,1);
    v0 = v_true(:,1) + [0.1; 0.1; 0.2] .* randn(3,1);
    ekf = ESEKF(p0, v0, q0);

    % ── 3. ZUPT를 이용한 초기 바이어스 수렴 (자동 정렬) ──────────────────
    % 센서 노이즈가 포함된 정지 상태 데이터를 공급하는 핸들
    get_static_imu = @() deal(f_static + true_ba + std_acc * randn(3,1), ...
                             true_bg + std_gyro * randn(3,1));
    
    % 지자기 센서 데이터 공급 핸들 (발사 전 정지 상태)
    get_static_mag = @() m_static + sqrt(S.var_mag) * randn(3,1);
                             
    % 정렬 수행: ZUPT(속도0) + 지자기(Heading)를 모두 사용하여 X축 바이어스까지 수렴
    [t_align, is_ok] = ekf.run_zupt_alignment(get_static_imu, dt, 1e-4, 30.0, get_static_mag);
    
    if trial == 1
        fprintf('  [Trial 1] 정렬 완료: %.2f초 소요 (수렴: %d)\n', t_align, is_ok);
    end

    gps_ptr = 1;  baro_ptr = 1;
    
    % ── EKF 루프 ─────────────────────────────────────────────────────
    for k = 1:N
        ekf.predict(a_meas(:,k), w_meas(:,k), dt);
        
        if gps_ptr <= length(gps_idx) && k == gps_idx(gps_ptr)
            ekf.update_gps(z_gps(:, gps_ptr));
            gps_ptr = gps_ptr + 1;
        end
        if baro_ptr <= length(baro_idx) && k == baro_idx(baro_ptr)
            ekf.update_baro(z_baro(baro_ptr));
            baro_ptr = baro_ptr + 1;
        end
        
        if trial == 1
            state_p_sim(:,k)  = ekf.nom.p;
            state_v_sim(:,k)  = ekf.nom.v;
            state_q_sim(:,k)  = ekf.nom.q;
            state_ba_sim(:,k) = ekf.nom.b_a;
            state_bg_sim(:,k) = ekf.nom.b_g;
        end
        
        ep  = ekf.nom.p   - p_true(:, k);
        ev  = ekf.nom.v   - v_true(:, k);
        eba = ekf.nom.b_a - true_ba;
        ebg = ekf.nom.b_g - true_bg;
        
        err_p(trial, k)  = norm(ep);
        err_v(trial, k)  = norm(ev);
        err_ba(trial, k) = norm(eba);
        err_bg(trial, k) = norm(ebg);
        
        % ── 필터의 P 행렬에서 3차원 분산의 합(Trace) 추출 ──
        cov_p(trial, k)  = trace(ekf.par.P(1:3, 1:3));
        cov_v(trial, k)  = trace(ekf.par.P(4:6, 4:6));
        cov_ba(trial, k) = trace(ekf.par.P(10:12, 10:12)); 
        cov_bg(trial, k) = trace(ekf.par.P(13:15, 13:15)); 
        
        e_pv  = [ep; ev];
        P_pv  = ekf.par.P(1:6, 1:6);
        nees_pv(trial, k) = e_pv' * (P_pv \ e_pv);
    end
end
fprintf('모든 Trial 완료.\n');

% ═══════════════════════════════════════════════════════════════════════
% 통계 & 필터 3-Sigma 상한선 도출
% ═══════════════════════════════════════════════════════════════════════
mean_p    = mean(err_p,   1);
mean_v    = mean(err_v,   1);
mean_ba   = mean(err_ba,  1);
mean_bg   = mean(err_bg,  1);
mean_nees = mean(nees_pv, 1);

% ── 필터가 추정한 3-Sigma 상한선 계산 ──
bound_p  = 3 * sqrt(mean(cov_p, 1));
bound_v  = 3 * sqrt(mean(cov_v, 1));
bound_ba = 3 * sqrt(mean(cov_ba, 1));
bound_bg = 3 * sqrt(mean(cov_bg, 1));

last_n   = round(5 / dt);
idx_last = (N - last_n + 1) : N;
nees_last = mean(mean_nees(idx_last));

fprintf('\n─── 전체 비행 성능 ───\n');
fprintf('  위치 평균오차: %.3f m\n', mean(mean_p));
fprintf('  속도 평균오차: %.4f m/s\n', mean(mean_v));

fprintf('\n─── 수렴 후 성능 (마지막 5초) ───\n');
fprintf('  위치 평균오차: %.3f m\n', mean(mean_p(idx_last)));
fprintf('  속도 평균오차: %.4f m/s\n', mean(mean_v(idx_last)));
fprintf('  NEES (6DOF)  : %.2f  (이상값 = 6)\n', nees_last);

% ═══════════════════════════════════════════════════════════════════════
% 시각화 1: 일관성 (Errors)
% ═══════════════════════════════════════════════════════════════════════
figure('Name', sprintf('Consistency Sim [N=%d]', N_TRIALS), ...
       'Position', [50, 50, 1200, 780]);

subplot(2,3,1); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_p(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_p, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_p, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma');
legend('Location','northeast'); xlabel('Time [s]'); ylabel('Norm 오차 [m]'); title('위치 오차 (Norm)');

subplot(2,3,2); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_v(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_v, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_v, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma');
legend('Location','northeast'); xlabel('Time [s]'); ylabel('Norm 오차 [m/s]'); title('속도 오차 (Norm)');

subplot(2,3,3); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_ba(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_ba, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_ba, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma');
legend('Location','northeast'); xlabel('Time [s]'); ylabel('Norm 오차 [m/s²]'); title('가속도 바이어스 오차 (Norm)');

subplot(2,3,4); hold on; grid on;
for trial = 1:N_TRIALS
    plot(t, err_bg(trial,:), 'Color', [0.85 0.85 0.85], 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
plot(t, mean_bg, 'k', 'LineWidth', 2.5, 'DisplayName', '앙상블 평균');
plot(t, bound_bg, 'r--', 'LineWidth', 2.0, 'DisplayName', '필터 +3\sigma');
legend('Location','northeast'); xlabel('Time [s]'); ylabel('Norm 오차 [rad/s]'); title('자이로 바이어스 오차 (Norm)');

subplot(2,3,5); hold on; grid on;
plot(t, mean_nees, 'k',   'LineWidth', 2.5, 'DisplayName', 'NEES');
yline(6,  'g--', 'LineWidth', 2.0, 'DisplayName', '이상값 = 6');
yline(3,  'r:',  'LineWidth', 1.5, 'DisplayName', '과신 경계');
yline(12, 'r:',  'LineWidth', 1.5, 'DisplayName', '과소신뢰 경계');
ylim([0, 30]); legend('Location','northeast'); xlabel('Time [s]'); ylabel('NEES'); title('NEES 일관성 (≈6)');

subplot(2,3,6); hold on; grid on;
histogram(err_p(:,end), 15, 'FaceColor', [0.3 0.5 0.8]);
xline(mean(err_p(:,end)),   'r-',  'LineWidth', 2, 'DisplayName', sprintf('평균=%.2f m', mean(err_p(:,end))));
xline(median(err_p(:,end)), 'g--', 'LineWidth', 1.5, 'DisplayName', sprintf('중앙값=%.2f m', median(err_p(:,end))));
legend('Location','northeast'); xlabel('최종 위치 오차 (Norm) [m]'); ylabel('빈도'); title('최종 위치 오차 분포');

sgtitle(sprintf('ES-EKF Consistency  [OpenRocket truth, N=%d]', N_TRIALS));

% ═══════════════════════════════════════════════════════════════════════
% 시각화 2: 명목 상태 변수 (Trial 1 vs Truth)
% ═══════════════════════════════════════════════════════════════════════
figure('Name', 'Nominal State Variables [Trial 1]', 'Position', [100, 100, 1200, 850]);

% 1. 위치 (Position NED)
subplot(2,3,1); hold on; grid on;
plot(t, p_true(1,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'True N');
plot(t, p_true(2,:), 'g--', 'LineWidth', 1.5, 'DisplayName', 'True E');
plot(t, p_true(3,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'True D');
plot(t, state_p_sim(1,:), 'r', 'LineWidth', 1.0, 'DisplayName', 'Est N');
plot(t, state_p_sim(2,:), 'g', 'LineWidth', 1.0, 'DisplayName', 'Est E');
plot(t, state_p_sim(3,:), 'b', 'LineWidth', 1.0, 'DisplayName', 'Est D');
legend('Location','best','FontSize',7); xlabel('Time [s]'); ylabel('Position [m]'); title('위치 (NED)');

% 2. 속도 (Velocity NED)
subplot(2,3,2); hold on; grid on;
plot(t, v_true(1,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'True Vn');
plot(t, v_true(2,:), 'g--', 'LineWidth', 1.5, 'DisplayName', 'True Ve');
plot(t, v_true(3,:), 'b--', 'LineWidth', 1.5, 'DisplayName', 'True Vd');
plot(t, state_v_sim(1,:), 'r', 'LineWidth', 1.0, 'DisplayName', 'Est Vn');
plot(t, state_v_sim(2,:), 'g', 'LineWidth', 1.0, 'DisplayName', 'Est Ve');
plot(t, state_v_sim(3,:), 'b', 'LineWidth', 1.0, 'DisplayName', 'Est Vd');
legend('Location','best','FontSize',7); xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('속도 (NED)');

% 3. 오일러 각 (Roll, Pitch, Yaw)
subplot(2,3,3); hold on; grid on;
euler_true = zeros(3, N);
euler_est  = zeros(3, N);
for k = 1:N
    euler_true(:,k) = rad2deg(NavUtils.quat2euler(q_true(:,k)));
    euler_est(:,k)  = rad2deg(NavUtils.quat2euler(state_q_sim(:,k)));
end
plot(t, euler_true(1,:), 'r--', 'DisplayName', 'True Roll');
plot(t, euler_true(2,:), 'g--', 'DisplayName', 'True Pitch');
plot(t, euler_true(3,:), 'b--', 'DisplayName', 'True Yaw');
plot(t, euler_est(1,:), 'r', 'DisplayName', 'Est Roll');
plot(t, euler_est(2,:), 'g', 'DisplayName', 'Est Pitch');
plot(t, euler_est(3,:), 'b', 'DisplayName', 'Est Yaw');
legend('Location','best','FontSize',7); xlabel('Time [s]'); ylabel('Angle [deg]'); title('자세 (Euler)');

% 4. 가속도 바이어스
subplot(2,3,4); hold on; grid on;
yline(true_ba(1), 'r--', 'DisplayName', 'True ba_x');
yline(true_ba(2), 'g--', 'DisplayName', 'True ba_y');
yline(true_ba(3), 'b--', 'DisplayName', 'True ba_z');
plot(t, state_ba_sim(1,:), 'r', 'DisplayName', 'Est ba_x');
plot(t, state_ba_sim(2,:), 'g', 'DisplayName', 'Est ba_y');
plot(t, state_ba_sim(3,:), 'b', 'DisplayName', 'Est ba_z');
legend('Location','best','FontSize',7); xlabel('Time [s]'); ylabel('Bias [m/s²]'); title('가속도 바이어스');

% 5. 자이로 바이어스
subplot(2,3,5); hold on; grid on;
yline(rad2deg(true_bg(1)), 'r--', 'DisplayName', 'True bg_x');
yline(rad2deg(true_bg(2)), 'g--', 'DisplayName', 'True bg_y');
yline(rad2deg(true_bg(3)), 'b--', 'DisplayName', 'True bg_z');
plot(t, rad2deg(state_bg_sim(1,:)), 'r', 'DisplayName', 'Est bg_x');
plot(t, rad2deg(state_bg_sim(2,:)), 'g', 'DisplayName', 'Est bg_y');
plot(t, rad2deg(state_bg_sim(3,:)), 'b', 'DisplayName', 'Est bg_z');
legend('Location','best','FontSize',7); xlabel('Time [s]'); ylabel('Bias [deg/s]'); title('자이로 바이어스');

% 6. 3D 궤적 비교
subplot(2,3,6); hold on; grid on;
plot3(p_true(2,:), p_true(1,:), -p_true(3,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Truth');
plot3(state_p_sim(2, :), state_p_sim(1, :), -state_p_sim(3, :), 'b', 'LineWidth', 1.0, 'DisplayName', 'ES-EKF');
view(45, 30); axis equal; legend('Location','best');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude [m]'); title('3D 비행 궤적');

sgtitle('ES-EKF 명목 상태 변수 추정 결과 (Trial 1)');

% ═══════════════════════════════════════════════════════════════════════
% 결과 저장
% ═══════════════════════════════════════════════════════════════════════
save('consistance_results.mat', 'err_p', 'err_v', 'err_ba', 'err_bg', ...
     'nees_pv', 't', 'N_TRIALS', 'p_true', 'v_true', 'q_true');
fprintf('\n결과 저장 완료: consistance_results.mat\n');