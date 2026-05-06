clear; clc; close all;
addpath('..');

% ═══════════════════════════════════════════════════════════════════════
% 초기 정렬(ZUPT+ZARU+Acc-static+Mag) 적용 vs 미적용 비교
% ═══════════════════════════════════════════════════════════════════════
CSV_PATH   = 'openrocket.csv';
N_TRIALS   = 50;
BASE_SEED  = 42;

S  = SensorSpec;
dt = S.dt_imu;

% OpenRocket truth
raw       = readmatrix(CSV_PATH, 'CommentStyle', '#');
t_or      = raw(:, 1);
acc_or    = raw(:, 8:10);
rate_or   = raw(:, 15:17);
zenith0   = raw(1, 18);
azimuth0  = raw(1, 19);

acc_mat  = [acc_or(:,3),  acc_or(:,1),  acc_or(:,2)];
rate_mat = deg2rad([rate_or(:,1), rate_or(:,3), rate_or(:,2)]);

t_end = t_or(end);
t     = (0 : dt : t_end)';
N     = length(t);

acc_u  = interp1(t_or, acc_mat,  t, 'linear', 'extrap');
rate_u = interp1(t_or, rate_mat, t, 'linear', 'extrap');

% 초기 자세
yaw_init   = deg2rad(azimuth0);
pitch_init = -deg2rad(zenith0);
roll_init  = 0;
q_init = quaternion([yaw_init, pitch_init, roll_init], 'euler', 'ZYX', 'frame');

% Truth trajectory
traj = kinematicTrajectory('SampleRate', round(1/dt), ...
        'Position', [0 0 0], 'Velocity', [0 0 0], 'Orientation', q_init);
[pos_ned, orient, vel_ned] = traj(acc_u, rate_u);

p_true = pos_ned';
v_true = vel_ned';
q_true = compact(orient)';

fprintf('=== 정렬 적용/미적용 비교 ===\n');
fprintf('  시뮬 시간 : %.2f s  (N = %d)\n', t_end, N);
fprintf('  최대고도  : %.1f m\n', -min(-p_true(3,:)));
fprintf('  반복횟수  : %d trial\n\n', N_TRIALS);

% 측정 인덱스
gps_idx  = 1 : round(S.dt_gps  / dt) : N;
baro_idx = 1 : round(S.dt_baro / dt) : N;
mag_idx  = 1 : round(S.dt_mag  / dt) : N;
m_ref_unit = S.m_ref_ned / norm(S.m_ref_ned);

% 참 바이어스
true_ba = [0.05; -0.02;  0.01  ];
true_bg = [0.001; -0.002; 0.003];

% Specific force truth
g_ned = [0; 0; 9.80665];
f_body_true = zeros(3, N);
for k = 1:N
    R_bn = NavUtils.quat2dcm(q_true(:, k));
    g_body = R_bn' * g_ned;
    f_body_true(:, k) = acc_u(k, :)' - g_body;
end
w_body_true = rate_u';

% ═══════════════════════════════════════════════════════════════════════
% 결과 저장 — 두 케이스 (1: 정렬 적용, 2: 정렬 미적용)
% ═══════════════════════════════════════════════════════════════════════
err_p   = zeros(2, N_TRIALS, N);
err_v   = zeros(2, N_TRIALS, N);
err_ba  = zeros(2, N_TRIALS, N);
err_bg  = zeros(2, N_TRIALS, N);
err_att = zeros(2, N_TRIALS, N);
nees_pv = zeros(2, N_TRIALS, N);

% Trial 1 상태 기록
state_p_sim  = zeros(2, 3, N);
state_v_sim  = zeros(2, 3, N);
state_q_sim  = zeros(2, 4, N);
state_ba_sim = zeros(2, 3, N);
state_bg_sim = zeros(2, 3, N);

cov_p   = zeros(2, N_TRIALS, N);
cov_v   = zeros(2, N_TRIALS, N);
cov_ba  = zeros(2, N_TRIALS, N);
cov_bg  = zeros(2, N_TRIALS, N);

case_name = {'WITH 정렬', 'WITHOUT 정렬'};

% ═══════════════════════════════════════════════════════════════════════
% Monte Carlo
% ═══════════════════════════════════════════════════════════════════════
fprintf('시뮬레이션 진행 중...\n');
for trial = 1:N_TRIALS
    if mod(trial, 10) == 0 || trial == 1
        fprintf('  Trial %d / %d ...\n', trial, N_TRIALS);
    end

    % 같은 시드 → 두 케이스 모두 동일 측정 노이즈 사용
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
        R_nb_k = NavUtils.quat2dcm(q_true(:, mag_idx(k)));
        z_mag(:,k) = R_nb_k' * m_ref_unit + sqrt(S.var_mag) * randn(3,1);
    end

    % TRIAD 초기 자세 (두 케이스 공통)
    R_bn_true0 = NavUtils.quat2dcm(q_true(:,1));
    f_static   = R_bn_true0' * (-g_ned);
    m_static   = R_bn_true0' * m_ref_unit;

    acc_meas_triad = f_static + true_ba + std_acc * randn(3,1);
    mag_meas_triad = m_static + sqrt(S.var_mag) * randn(3,1);
    q0 = ESEKF.run_triad(acc_meas_triad, mag_meas_triad);

    % 동일 초기 위치/속도 perturbation
    p0 = p_true(:,1) + [0.5; 0.5; 1.0] .* randn(3,1);
    v0 = v_true(:,1) + [0.1; 0.1; 0.2] .* randn(3,1);

    for c = 1:2
        ekf = ESEKF(p0, v0, q0);

        if c == 1
            % ── 정렬 적용 (ZUPT+ZARU+Acc-static+Mag) ────────────────
            get_static_imu = @() deal(f_static + true_ba + std_acc * randn(3,1), ...
                                      true_bg + std_gyro * randn(3,1));
            get_static_mag = @() m_static + sqrt(S.var_mag) * randn(3,1);
            [t_align, is_ok] = ekf.run_zupt_alignment(get_static_imu, dt, 1e-4, 30.0, get_static_mag);
            if trial == 1
                fprintf('  [Trial1, WITH 정렬] %.2f s, 수렴=%d\n', t_align, is_ok);
            end
        end
        % c==2: 정렬 없이 바로 비행

        gps_ptr = 1;  baro_ptr = 1;

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
                state_p_sim(c, :, k)  = ekf.nom.p;
                state_v_sim(c, :, k)  = ekf.nom.v;
                state_q_sim(c, :, k)  = ekf.nom.q;
                state_ba_sim(c, :, k) = ekf.nom.b_a;
                state_bg_sim(c, :, k) = ekf.nom.b_g;
            end

            ep  = ekf.nom.p   - p_true(:, k);
            ev  = ekf.nom.v   - v_true(:, k);
            eba = ekf.nom.b_a - true_ba;
            ebg = ekf.nom.b_g - true_bg;

            % 자세 오차 (쿼터니언 차이의 회전각)
            qe = NavUtils.quat_mult(NavUtils.quat_conj(q_true(:,k)), ekf.nom.q);
            err_att(c, trial, k) = 2 * acos(min(1, abs(qe(1))));

            err_p(c, trial, k)  = norm(ep);
            err_v(c, trial, k)  = norm(ev);
            err_ba(c, trial, k) = norm(eba);
            err_bg(c, trial, k) = norm(ebg);

            cov_p(c, trial, k)  = trace(ekf.par.P(1:3, 1:3));
            cov_v(c, trial, k)  = trace(ekf.par.P(4:6, 4:6));
            cov_ba(c, trial, k) = trace(ekf.par.P(10:12, 10:12));
            cov_bg(c, trial, k) = trace(ekf.par.P(13:15, 13:15));

            e_pv = [ep; ev];
            P_pv = ekf.par.P(1:6, 1:6);
            nees_pv(c, trial, k) = e_pv' * (P_pv \ e_pv);
        end
    end
end
fprintf('완료.\n');

% ═══════════════════════════════════════════════════════════════════════
% 통계
% ═══════════════════════════════════════════════════════════════════════
mean_p   = squeeze(mean(err_p,   2));
mean_v   = squeeze(mean(err_v,   2));
mean_ba  = squeeze(mean(err_ba,  2));
mean_bg  = squeeze(mean(err_bg,  2));
mean_att = squeeze(mean(err_att, 2));
mean_nees= squeeze(mean(nees_pv, 2));

bound_p  = 3 * sqrt(squeeze(mean(cov_p,  2)));
bound_v  = 3 * sqrt(squeeze(mean(cov_v,  2)));

last_n   = round(5 / dt);
idx_last = (N - last_n + 1) : N;

fprintf('\n─── 마지막 5초 평균 성능 ───\n');
fprintf('              %12s   %12s\n', case_name{1}, case_name{2});
fprintf('  pos err  :  %10.3f m   %10.3f m\n', mean(mean_p(1,idx_last)),  mean(mean_p(2,idx_last)));
fprintf('  vel err  :  %10.4f m/s %10.4f m/s\n', mean(mean_v(1,idx_last)),  mean(mean_v(2,idx_last)));
fprintf('  att err  :  %10.4f deg %10.4f deg\n', rad2deg(mean(mean_att(1,idx_last))), rad2deg(mean(mean_att(2,idx_last))));
fprintf('  ba  err  :  %10.4f     %10.4f\n', mean(mean_ba(1,idx_last)), mean(mean_ba(2,idx_last)));
fprintf('  bg  err  :  %10.4e    %10.4e\n',  mean(mean_bg(1,idx_last)), mean(mean_bg(2,idx_last)));
fprintf('  NEES(6)  :  %10.2f     %10.2f   (이상값=6)\n', mean(mean_nees(1,idx_last)), mean(mean_nees(2,idx_last)));

% ═══════════════════════════════════════════════════════════════════════
% 시각화 1: 오차 비교
% ═══════════════════════════════════════════════════════════════════════
col_w = [0.10 0.45 0.85];   % WITH (파랑)
col_o = [0.85 0.30 0.15];   % WITHOUT (주황)

figure('Name', 'Alignment ON/OFF — Error Comparison', 'Position',[50 50 1300 800]);

subplot(2,3,1); hold on; grid on;
plot(t, mean_p(1,:), 'Color', col_w, 'LineWidth', 2.0, 'DisplayName', case_name{1});
plot(t, mean_p(2,:), 'Color', col_o, 'LineWidth', 2.0, 'DisplayName', case_name{2});
legend('Location','best'); xlabel('Time [s]'); ylabel('|p_{err}| [m]');
title('위치 오차 (앙상블 평균)');

subplot(2,3,2); hold on; grid on;
plot(t, mean_v(1,:), 'Color', col_w, 'LineWidth', 2.0, 'DisplayName', case_name{1});
plot(t, mean_v(2,:), 'Color', col_o, 'LineWidth', 2.0, 'DisplayName', case_name{2});
legend('Location','best'); xlabel('Time [s]'); ylabel('|v_{err}| [m/s]');
title('속도 오차 (앙상블 평균)');

subplot(2,3,3); hold on; grid on;
plot(t, rad2deg(mean_att(1,:)), 'Color', col_w, 'LineWidth', 2.0, 'DisplayName', case_name{1});
plot(t, rad2deg(mean_att(2,:)), 'Color', col_o, 'LineWidth', 2.0, 'DisplayName', case_name{2});
legend('Location','best'); xlabel('Time [s]'); ylabel('|\theta_{err}| [deg]');
title('자세 오차 (회전각 norm)');

subplot(2,3,4); hold on; grid on;
plot(t, mean_ba(1,:), 'Color', col_w, 'LineWidth', 2.0, 'DisplayName', case_name{1});
plot(t, mean_ba(2,:), 'Color', col_o, 'LineWidth', 2.0, 'DisplayName', case_name{2});
legend('Location','best'); xlabel('Time [s]'); ylabel('|b_a err| [m/s²]');
title('가속도 바이어스 오차');

subplot(2,3,5); hold on; grid on;
plot(t, rad2deg(mean_bg(1,:)), 'Color', col_w, 'LineWidth', 2.0, 'DisplayName', case_name{1});
plot(t, rad2deg(mean_bg(2,:)), 'Color', col_o, 'LineWidth', 2.0, 'DisplayName', case_name{2});
legend('Location','best'); xlabel('Time [s]'); ylabel('|b_g err| [deg/s]');
title('자이로 바이어스 오차');

subplot(2,3,6); hold on; grid on;
plot(t, mean_nees(1,:), 'Color', col_w, 'LineWidth', 2.0, 'DisplayName', case_name{1});
plot(t, mean_nees(2,:), 'Color', col_o, 'LineWidth', 2.0, 'DisplayName', case_name{2});
yline(6,  'k--', 'LineWidth', 1.5, 'DisplayName', '이상값=6');
ylim([0, 30]); legend('Location','best'); xlabel('Time [s]'); ylabel('NEES');
title('NEES 일관성');

sgtitle(sprintf('초기 정렬 효과 비교 [N=%d trials]', N_TRIALS));

% ═══════════════════════════════════════════════════════════════════════
% 시각화 2: Trial 1 상태 변수 비교
% ═══════════════════════════════════════════════════════════════════════
figure('Name', 'Alignment ON/OFF — State Trace [Trial 1]', 'Position', [80 80 1300 850]);

% 자세 오일러
subplot(2,3,1); hold on; grid on;
eu_true = zeros(3, N); eu_w = zeros(3, N); eu_o = zeros(3, N);
for k = 1:N
    eu_true(:,k) = rad2deg(NavUtils.quat2euler(q_true(:,k)));
    eu_w(:,k)    = rad2deg(NavUtils.quat2euler(squeeze(state_q_sim(1,:,k))'));
    eu_o(:,k)    = rad2deg(NavUtils.quat2euler(squeeze(state_q_sim(2,:,k))'));
end
plot(t, eu_true(2,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Pitch');
plot(t, eu_w(2,:),    'Color', col_w, 'LineWidth', 1.0, 'DisplayName', 'WITH');
plot(t, eu_o(2,:),    'Color', col_o, 'LineWidth', 1.0, 'DisplayName', 'WITHOUT');
legend('Location','best'); xlabel('Time [s]'); ylabel('Pitch [deg]'); title('Pitch (Trial 1)');

subplot(2,3,2); hold on; grid on;
plot(t, eu_true(3,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Yaw');
plot(t, eu_w(3,:),    'Color', col_w, 'LineWidth', 1.0, 'DisplayName', 'WITH');
plot(t, eu_o(3,:),    'Color', col_o, 'LineWidth', 1.0, 'DisplayName', 'WITHOUT');
legend('Location','best'); xlabel('Time [s]'); ylabel('Yaw [deg]'); title('Yaw (Trial 1)');

% 가속도 바이어스 X
subplot(2,3,3); hold on; grid on;
yline(true_ba(1), 'k--', 'LineWidth', 1.5, 'DisplayName', 'True ba_x');
plot(t, squeeze(state_ba_sim(1,1,:)), 'Color', col_w, 'LineWidth', 1.0, 'DisplayName', 'WITH');
plot(t, squeeze(state_ba_sim(2,1,:)), 'Color', col_o, 'LineWidth', 1.0, 'DisplayName', 'WITHOUT');
legend('Location','best'); xlabel('Time [s]'); ylabel('b_{a,x} [m/s²]');
title('가속도 바이어스 X (Trial 1)');

% 자이로 바이어스 X
subplot(2,3,4); hold on; grid on;
yline(rad2deg(true_bg(1)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'True bg_x');
plot(t, rad2deg(squeeze(state_bg_sim(1,1,:))), 'Color', col_w, 'LineWidth', 1.0, 'DisplayName', 'WITH');
plot(t, rad2deg(squeeze(state_bg_sim(2,1,:))), 'Color', col_o, 'LineWidth', 1.0, 'DisplayName', 'WITHOUT');
legend('Location','best'); xlabel('Time [s]'); ylabel('b_{g,x} [deg/s]');
title('자이로 바이어스 X (Trial 1)');

% 위치 D (고도)
subplot(2,3,5); hold on; grid on;
plot(t, -p_true(3,:),                       'k--', 'LineWidth', 1.5, 'DisplayName', 'Truth');
plot(t, -squeeze(state_p_sim(1,3,:)),       'Color', col_w, 'LineWidth', 1.0, 'DisplayName', 'WITH');
plot(t, -squeeze(state_p_sim(2,3,:)),       'Color', col_o, 'LineWidth', 1.0, 'DisplayName', 'WITHOUT');
legend('Location','best'); xlabel('Time [s]'); ylabel('Altitude [m]'); title('고도 (Trial 1)');

% 3D 궤적
subplot(2,3,6); hold on; grid on;
plot3(p_true(2,:), p_true(1,:), -p_true(3,:), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Truth');
plot3(squeeze(state_p_sim(1,2,:)), squeeze(state_p_sim(1,1,:)), -squeeze(state_p_sim(1,3,:)), ...
    'Color', col_w, 'LineWidth', 1.0, 'DisplayName', 'WITH');
plot3(squeeze(state_p_sim(2,2,:)), squeeze(state_p_sim(2,1,:)), -squeeze(state_p_sim(2,3,:)), ...
    'Color', col_o, 'LineWidth', 1.0, 'DisplayName', 'WITHOUT');
view(45, 30); axis equal; legend('Location','best');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Alt [m]'); title('3D 궤적');

sgtitle('초기 정렬 효과 — 명목 상태 (Trial 1)');

% ═══════════════════════════════════════════════════════════════════════
% 시각화 3: 최종 위치 오차 히스토그램
% ═══════════════════════════════════════════════════════════════════════
figure('Name', 'Final Position Error Histogram', 'Position', [120 120 900 400]);

final_p_w = squeeze(err_p(1, :, end));
final_p_o = squeeze(err_p(2, :, end));

subplot(1,2,1); hold on; grid on;
histogram(final_p_w, 15, 'FaceColor', col_w, 'FaceAlpha', 0.7);
xline(mean(final_p_w), 'k-', 'LineWidth', 2, 'DisplayName', sprintf('평균=%.2f m', mean(final_p_w)));
xlabel('|p_{err}| @ end [m]'); ylabel('빈도'); legend; title(case_name{1});

subplot(1,2,2); hold on; grid on;
histogram(final_p_o, 15, 'FaceColor', col_o, 'FaceAlpha', 0.7);
xline(mean(final_p_o), 'k-', 'LineWidth', 2, 'DisplayName', sprintf('평균=%.2f m', mean(final_p_o)));
xlabel('|p_{err}| @ end [m]'); ylabel('빈도'); legend; title(case_name{2});

sgtitle(sprintf('최종 위치 오차 분포 [N=%d]', N_TRIALS));

% ═══════════════════════════════════════════════════════════════════════
% 결과 저장
% ═══════════════════════════════════════════════════════════════════════
save('compare_alignment_results.mat', 'err_p', 'err_v', 'err_att', ...
     'err_ba', 'err_bg', 'nees_pv', 't', 'N_TRIALS', ...
     'p_true', 'v_true', 'q_true', 'case_name');
fprintf('\n결과 저장: compare_alignment_results.mat\n');
