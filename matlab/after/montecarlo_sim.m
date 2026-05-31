%% montecarlo_sim.m — 1D KF Monte Carlo with NAV.cpp pipeline
%
%  Reproduces the exact firmware path:
%    1. Gyro (+ noise) → quaternion integration  (NAV::integrateQuaternion)
%    2. body specific force (+ noise) → R(q̂) → NED kinematic acc
%    3. acc_up = -a_NED(3)  →  KF1D::predict(acc_up, dt)
%    4. alt (+ baro noise)  →  KF1D::update(z)
%
%  Ground truth is built from OpenRocket (noise-free):
%    - q_true  : noise-free gyro integration
%    - f_body  : NED acc → body frame via q_true
%    - alt, vel: spline-resampled from CSV

clear; close all; clc;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..'));   % KF1D.m
CSV_PATH = fullfile(this_dir, 'openrocket.csv');

%% ── Config ───────────────────────────────────────────────────────────────
F_IMU    = 416;          % IMU update rate [Hz]
F_BMP    = 50;           % Baro update rate [Hz]
N_MC     = 200;          % Monte Carlo runs
G        = 9.80665;
g_NED    = [0; 0; G];   % gravity in NED frame (down = +Z)
USE_KINEMATIC_2STAGE = true;
% Realistic validation presets:
%   baseline            : nominal flight + conservative white noise
%   energy_uncertainty  : lower-energy/high-drag flight + sensor bias
%   delayed_stage       : late 2nd-stage ignition + scale/misalignment error
%   aero_tilt           : pitch/lateral disturbance + baro lag/drift
%   stress              : harsh staging transient + combined worst-case errors
% Use 'custom' only when directly setting KINEMATIC_CASE and DISTURBANCE_CASE.
SCENARIO_PRESET = 'stress';
KINEMATIC_CASE = '';      % used only when SCENARIO_PRESET = 'custom'
DISTURBANCE_CASE = '';    % used only when SCENARIO_PRESET = 'custom'
if ~strcmpi(SCENARIO_PRESET, 'custom')
    [KINEMATIC_CASE, DISTURBANCE_CASE] = scenario_preset(SCENARIO_PRESET);
end

global SIGMA_A_SQ SIGMA_B_SQ
SIGMA_A_SQ = 1.0e-3;    % per-sample accel variance [(m/s²)²]  — LSM6DSO32 spec
SIGMA_B_SQ = 0.089;     % per-sample baro  variance [m²]       — measured
SIGMA_G_SQ = 1.2e-6; % per-sample gyro  variance [(rad/s)²] — LSM6DSO32 spec

sa = sqrt(SIGMA_A_SQ);
sb = sqrt(SIGMA_B_SQ);
sg = sqrt(SIGMA_G_SQ);
D = disturbance_config(DISTURBANCE_CASE);
fprintf('Scenario preset: %s\n', SCENARIO_PRESET);
fprintf('Kinematic case: %s\n', KINEMATIC_CASE);
fprintf('Disturbance case: %s\n', DISTURBANCE_CASE);

%% ── Load & resample ──────────────────────────────────────────────────────
if USE_KINEMATIC_2STAGE
    fprintf('Loading kinematic 2-stage trajectory: %s ...\n', KINEMATIC_CASE);
    raw = trajectory_2stage_kinematic(1 / F_IMU, KINEMATIC_CASE);
else
    fprintf('Loading %s ...\n', CSV_PATH);
    raw = load_openrocket(CSV_PATH);
end

dt       = 1 / F_IMU;
t        = (raw.t(1) : dt : raw.t(end)).';
N        = numel(t);
bmpEvery = round(F_IMU / F_BMP);

% spline for smooth signals, pchip for signals with abrupt steps
% (pchip = monotone cubic: no ringing/overshoot at parachute deployment)
rsp_s = @(x) interp1(raw.t, x, t, 'spline');   % smooth signals
rsp_p = @(x) interp1(raw.t, x, t, 'pchip');    % signals with impulses

alt  = rsp_s(raw.alt);         % truth altitude AGL [m]
vel  = rsp_s(raw.vvel);        % truth vertical velocity [m/s]
va   = rsp_p(raw.vacc);        % vert. kinematic acc — pchip (step at chute deploy)
la   = rsp_p(raw.lacc);        % lateral acc          — pchip
ld_d = rsp_p(raw.lat_dir);     % lateral direction [deg]
wx   = rsp_s(raw.gx_rad);      % body angular rate X [rad/s]
wy   = rsp_s(raw.gy_rad);      %                   Y
wz   = rsp_s(raw.gz_rad);      %                   Z

fprintf('  Rows=%d  t_end=%.1fs  alt_max=%.1fm\n', N, t(end), max(alt));
if isfield(raw, 'events')
    fprintf('  Events: stage1 burnout %.2fs, stage2 ignition %.2fs, stage2 burnout %.2fs, apogee %.2fs\n', ...
        raw.events.stage1_burnout, raw.events.stage2_ignite, raw.events.stage2_burnout, raw.events.apogee);
end

%% ── Initial attitude (matches NAV::kfBegin fromTwoVectors) ───────────────
% body X = nosecone direction in NED, from zenith/azimuth
zen0 = deg2rad(raw.zenith0);
azi0 = deg2rad(raw.azimuth0);
body_X_NED = [cos(zen0)*cos(azi0);
              cos(zen0)*sin(azi0);
              -sin(zen0)];        % zenith=90° → (0,0,-1) = NED up ✓
q0 = quat_from_two_vec([1;0;0], body_X_NED);

%% ── Truth quaternion (noise-free gyro) ───────────────────────────────────
fprintf('Building truth trajectory ...\n');
q_true = zeros(4, N);
q = q0;
q_true(:,1) = q;
for k = 2:N
    q = quat_integ(q, [wx(k); wy(k); wz(k)], dt);
    q_true(:,k) = q;
end

%% ── Truth body-frame specific force ─────────────────────────────────────
% f_NED  = a_kinematic_NED - g_NED
% f_body = R_body2NED^T  *  f_NED
f_body_true = zeros(3, N);
for k = 1:N
    ld_rad = deg2rad(ld_d(k));
    a_NED  = [la(k)*cos(ld_rad);   % North
              la(k)*sin(ld_rad);   % East
              -va(k)];             % Down (NED +Z = down, kinematic up → negative)
    f_NED  = a_NED - g_NED;
    R      = quat2rot(q_true(:,k));          % body → NED
    f_body_true(:,k) = R.' * f_NED;         % NED → body
end

%% ── Verify consistency (f_body → NED → acc_up vs truth vel diff) ────────
acc_up_truth = zeros(N,1);
for k = 1:N
    R = quat2rot(q_true(:,k));
    a_ned_k = R * f_body_true(:,k) + g_NED;
    acc_up_truth(k) = -a_ned_k(3);
end
vel_from_acc = cumsum([vel(1); acc_up_truth(2:end)*dt]);
diff_ms = max(abs(vel_from_acc - vel));
fprintf('  Consistency check: max |vel_integrated - vel_truth| = %.4f m/s\n', diff_ms);

%% ── Monte Carlo ──────────────────────────────────────────────────────────
fprintf('Running %d Monte Carlo runs ...\n', N_MC);
pos_mc = zeros(N, N_MC);
vel_mc = zeros(N, N_MC);
rng(42);

for mc = 1:N_MC
    kf    = KF1D(alt(1), vel(1));
    q_hat = q0;

    accel_bias = D.accel_bias_std * randn(3,1);
    gyro_bias  = D.gyro_bias_std  * randn(3,1);
    accel_scale = 1.0 + D.accel_scale_std * randn(3,1);
    gyro_scale  = 1.0 + D.gyro_scale_std  * randn(3,1);
    C_misalign = small_angle_dcm(D.imu_misalign_std_rad * randn(3,1));

    baro_bias = D.baro_bias_std * randn();
    baro_drift_rate = D.baro_drift_std * randn() / max(t(end), 1.0);
    baro_delay = max(0.0, D.baro_delay_s + D.baro_delay_jitter_s * randn());
    baro_delay_samples = round(baro_delay / dt);
    baro_lag_state = alt(1);

    for k = 1:N
        %──── IMU sim (matches IMU_Task → NAV::updateIMU) ────
        % 1. Noisy gyro → attitude estimate
        w_true = [wx(k); wy(k); wz(k)];
        w_noisy = gyro_scale .* w_true + gyro_bias + sg * randn(3,1);
        if k > 1
            q_hat = quat_integ(q_hat, w_noisy, dt);
        end

        % 2. Noisy specific force in body frame
        f_meas = C_misalign * (accel_scale .* f_body_true(:,k)) ...
               + accel_bias + sa * randn(3,1);
        f_meas = min(max(f_meas, -D.accel_clip_mps2), D.accel_clip_mps2);

        % 3. body → NED using estimated attitude, subtract gravity
        R_hat  = quat2rot(q_hat);
        a_NED  = R_hat * f_meas + g_NED;    % kinematic acc in NED
        acc_up = -a_NED(3);                  % up = -NED_Z

        %──── KF (matches KF1D::predict / update) ────
        if k > 1
            kf.predict(acc_up, dt);
        end
        if mod(k-1, bmpEvery) == 0
            if rand() >= D.baro_dropout_prob
                kd = max(1, k - baro_delay_samples);
                alt_delayed = alt(kd);
                if D.baro_lag_tau_s > 0
                    alpha = 1.0 - exp(-(bmpEvery * dt) / D.baro_lag_tau_s);
                    baro_lag_state = baro_lag_state + alpha * (alt_delayed - baro_lag_state);
                    alt_baro_truth = baro_lag_state;
                else
                    alt_baro_truth = alt_delayed;
                end

                z_baro = alt_baro_truth + baro_bias + baro_drift_rate * t(k) + sb * randn();
                if rand() < D.baro_outlier_prob
                    z_baro = z_baro + D.baro_outlier_std * randn();
                end
                kf.update(z_baro);
            end
        end

        pos_mc(k, mc) = kf.x(1);
        vel_mc(k, mc) = kf.x(2);
    end
end

%% ── Stats ────────────────────────────────────────────────────────────────
pos_err = pos_mc - alt;
vel_err = vel_mc - vel;
pos_3s  = 3 * std(pos_err, 0, 2);
vel_3s  = 3 * std(vel_err, 0, 2);

[a_apg, i_apg] = max(alt);
fprintf('\n=== Monte Carlo Summary ===\n');
fprintf('  Apogee  %.1f m  t=%.2fs   pos_3σ=%.2f m   vel_3σ=%.2f m/s\n', ...
        a_apg, t(i_apg), pos_3s(i_apg), vel_3s(i_apg));
fprintf('  Final                     pos_3σ=%.2f m   vel_3σ=%.2f m/s\n', ...
        pos_3s(end), vel_3s(end));

%% ── Plot ─────────────────────────────────────────────────────────────────
figure('Name','1D KF Monte Carlo','Color','w','Position',[100 100 1100 750]);

subplot(2,2,1); plot_traj(t, alt, mean(pos_mc,2), pos_3s);
ylabel('Altitude [m]'); title('Position');

subplot(2,2,2); plot_traj(t, vel, mean(vel_mc,2), vel_3s);
ylabel('Velocity [m/s]'); title('Velocity');

subplot(2,2,3); plot_err(t, pos_err, pos_3s);
ylabel('Pos err [m]'); xlabel('t [s]'); title('Position error ±3σ');

subplot(2,2,4); plot_err(t, vel_err, vel_3s);
ylabel('Vel err [m/s]'); xlabel('t [s]'); title('Velocity error ±3σ');

sgtitle(sprintf( ...
    '1D KF Monte Carlo  (N_{MC}=%d,  σ_a=%.3f m/s²,  σ_b=%.3f m,  σ_g=%.4f rad/s)', ...
    N_MC, sa, sb, sg));


%% =====================================================================
%%  LOCAL FUNCTIONS
%% =====================================================================

% ── Quaternion math (toolbox-free, matches NAV.cpp / Eigen) ────────────

function [kinematic_case, disturbance_case] = scenario_preset(name)
    switch lower(string(name))
        case "baseline"
            % Acceptance baseline: nominal flight with conservative sensor white noise.
            kinematic_case = 'nominal_800m';
            disturbance_case = 'white_noise';
        case "energy_uncertainty"
            % Motor/drag uncertainty: lower apogee profile plus sensor biases.
            kinematic_case = 'energy_envelope';
            disturbance_case = 'bias';
        case "delayed_stage"
            % Two-stage timing sensitivity: late ignition plus scale/mounting errors.
            kinematic_case = 'delayed_ignition';
            disturbance_case = 'scale_misalignment';
        case "aero_tilt"
            % Flight dynamics and pressure-port sensitivity: tilt plus baro lag/drift.
            kinematic_case = 'aero_tilt';
            disturbance_case = 'baro_lag_drift';
        case "stress"
            % Qualification-style upper bound: harsh staging plus combined errors.
            kinematic_case = 'harsh_transient';
            disturbance_case = 'worst_case';
        otherwise
            error('montecarlo_sim:UnknownScenarioPreset', ...
                'Unknown scenario preset: %s', name);
    end
end

function D = disturbance_config(case_name)
    D.case_name = lower(string(case_name));

    D.accel_bias_std = 0.0;
    D.gyro_bias_std = 0.0;
    D.accel_scale_std = 0.0;
    D.gyro_scale_std = 0.0;
    D.imu_misalign_std_rad = 0.0;
    D.accel_clip_mps2 = 32.0 * 9.80665;

    D.baro_bias_std = 0.0;
    D.baro_drift_std = 0.0;
    D.baro_delay_s = 0.0;
    D.baro_delay_jitter_s = 0.0;
    D.baro_lag_tau_s = 0.0;
    D.baro_dropout_prob = 0.0;
    D.baro_outlier_prob = 0.0;
    D.baro_outlier_std = 0.0;

    switch D.case_name
        case {"white_noise", "nominal"}
        case "bias"
            D.accel_bias_std = 0.05;
            D.gyro_bias_std = 0.003;
            D.baro_bias_std = 1.0;
            D.baro_drift_std = 0.8;
        case "scale_misalignment"
            D.accel_scale_std = 0.010;
            D.gyro_scale_std = 0.005;
            D.imu_misalign_std_rad = deg2rad(1.0);
        case "baro_lag_drift"
            D.baro_bias_std = 1.0;
            D.baro_drift_std = 2.0;
            D.baro_delay_s = 0.080;
            D.baro_delay_jitter_s = 0.020;
            D.baro_lag_tau_s = 0.060;
            D.baro_outlier_prob = 0.003;
            D.baro_outlier_std = 4.0;
        case "worst_case"
            D.accel_bias_std = 0.06;
            D.gyro_bias_std = 0.004;
            D.accel_scale_std = 0.012;
            D.gyro_scale_std = 0.006;
            D.imu_misalign_std_rad = deg2rad(1.2);
            D.baro_bias_std = 1.5;
            D.baro_drift_std = 2.5;
            D.baro_delay_s = 0.100;
            D.baro_delay_jitter_s = 0.030;
            D.baro_lag_tau_s = 0.080;
            D.baro_dropout_prob = 0.005;
            D.baro_outlier_prob = 0.005;
            D.baro_outlier_std = 5.0;
        otherwise
            error('montecarlo_sim:UnknownDisturbanceCase', ...
                'Unknown disturbance case: %s', case_name);
    end
end

function C = small_angle_dcm(theta)
    S = [ 0.0,      -theta(3),  theta(2);
          theta(3),  0.0,      -theta(1);
         -theta(2),  theta(1),  0.0 ];
    C = eye(3) + S;
end

function q = quat_integ(q, w, dt)
    % Matches NAV::integrateQuaternion()
    % q = [w; x; y; z], angular rate w in body frame [rad/s]
    w_norm = norm(w);
    if w_norm > 1e-6
        ha = w_norm * dt / 2;
        ax = w / w_norm;
        dq = [cos(ha); sin(ha) * ax];
    else
        dq = [1; 0.5 * w * dt];
    end
    q = quat_mul(q, dq / norm(dq));
    q = q / norm(q);
end

function r = quat_mul(p, q)
    % Hamilton product p * q   ([w; x; y; z])
    pw=p(1); px=p(2); py=p(3); pz=p(4);
    qw=q(1); qx=q(2); qy=q(3); qz=q(4);
    r = [pw*qw - px*qx - py*qy - pz*qz;
         pw*qx + px*qw + py*qz - pz*qy;
         pw*qy - px*qz + py*qw + pz*qx;
         pw*qz + px*qy - py*qx + pz*qw];
end

function R = quat2rot(q)
    % body → NED rotation matrix from [w; x; y; z]
    w=q(1); x=q(2); y=q(3); z=q(4);
    R = [1-2*(y^2+z^2),  2*(x*y-w*z),  2*(x*z+w*y);
          2*(x*y+w*z), 1-2*(x^2+z^2),  2*(y*z-w*x);
          2*(x*z-w*y),  2*(y*z+w*x), 1-2*(x^2+y^2)];
end

function q = quat_from_two_vec(a, b)
    % Shortest rotation from unit vector a to unit vector b
    % Matches Eigen Quaternionf::FromTwoVectors()
    a = a / norm(a);  b = b / norm(b);
    c = cross(a, b);
    d = dot(a, b);
    q = [1 + d; c];
    if norm(q) < 1e-10          % anti-parallel: 180° about any perp axis
        perp = [1;0;0] - a(1)*a;
        if norm(perp) < 1e-6,  perp = [0;1;0] - a(2)*a; end
        q = [0; perp / norm(perp)];
    else
        q = q / norm(q);
    end
end

% ── CSV loader ─────────────────────────────────────────────────────────

function tr = load_openrocket(path)
    if exist(path, 'file') ~= 2, error('CSV not found: %s', path); end
    fid = fopen(path, 'r');
    if fid < 0, error('Cannot open: %s', path); end
    hdr_line = '';
    while true
        line = fgetl(fid);
        if ~ischar(line), break; end
        if startsWith(strtrim(line), '#')
            if contains(line, 'Time (s)'), hdr_line = line; end
            continue;
        end
        break;
    end
    fclose(fid);
    if isempty(hdr_line), error('Header not found in CSV'); end

    raw = readmatrix(path, 'CommentStyle', '#');
    raw(~isfinite(raw)) = 0;

    hdr_line = regexprep(hdr_line, '^\s*#\s*', '');
    cols = strtrim(strsplit(hdr_line, ','));
    fc   = @(varargin) col_idx(cols, varargin{:});

    tr.t       = raw(:, fc('Time'));
    tr.alt     = raw(:, fc('Altitude (m)'));
    tr.vvel    = raw(:, fc('Vertical velocity'));
    tr.vacc    = raw(:, fc('Vertical acceleration'));
    tr.lacc    = raw(:, fc('Lateral acceleration'));
    tr.lat_dir = raw(:, fc('Lateral direction'));       % [deg]

    % Angular rates: OpenRocket body frame → match NAV.cpp body frame
    % OpenRocket: Z-axis = longitudinal (nosecone), X/Y = lateral
    % NAV body:   X-axis = nosecone,  Y/Z = lateral
    % Mapping: OR_Roll(Z) → gx, OR_Pitch(Y) → gy, OR_Yaw(X) → gz
    tr.gx_rad = deg2rad(raw(:, fc('Roll rate')));
    tr.gy_rad = deg2rad(raw(:, fc('Pitch rate')));
    tr.gz_rad = deg2rad(raw(:, fc('Yaw rate')));

    % Initial attitude
    tr.zenith0  = raw(1, fc('zenith'));
    tr.azimuth0 = raw(1, fc('azimuth'));
end

function idx = col_idx(cols, varargin)
    for c = 1:numel(cols)
        for k = 1:numel(varargin)
            if contains(cols{c}, varargin{k}, 'IgnoreCase', true)
                idx = c; return;
            end
        end
    end
    error('Column not found: %s', strjoin(varargin, '/'));
end

% ── Plot helpers ────────────────────────────────────────────────────────

function plot_traj(t, truth, est, sigma3)
    hold on; grid on;
    fill([t; flipud(t)], [est+sigma3; flipud(est-sigma3)], ...
         [1 0.85 0.85], 'EdgeColor', 'none', 'DisplayName', '±3σ');
    plot(t, truth, 'k',  'LineWidth', 1.5, 'DisplayName', 'Truth');
    plot(t, est,   'b--','LineWidth', 1.2, 'DisplayName', 'KF mean');
    legend('Location', 'best');
end

function plot_err(t, err, sigma3)
    err_mean = mean(err, 2);
    hold on; grid on;
    plot(t, err, 'Color', [0.85 0.85 0.85], 'HandleVisibility', 'off');
    plot(t, err_mean, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Mean error');
    plot(t, err_mean + sigma3, 'r', 'LineWidth', 1.3, 'DisplayName', 'Mean \pm3\sigma');
    plot(t, err_mean - sigma3, 'r', 'LineWidth', 1.3, 'HandleVisibility', 'off');
    legend('Location', 'best');
end
