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
addpath(fullfile(this_dir, '..'));   % KF1D.m
CSV_PATH = fullfile(this_dir, 'openrocket.csv');

%% ── Config ───────────────────────────────────────────────────────────────
F_IMU    = 416;          % IMU update rate [Hz]
F_BMP    = 50;           % Baro update rate [Hz]
N_MC     = 200;          % Monte Carlo runs
G        = 9.80665;
g_NED    = [0; 0; G];   % gravity in NED frame (down = +Z)

global SIGMA_A_SQ SIGMA_B_SQ
SIGMA_A_SQ = 6.3e-4;    % per-sample accel variance [(m/s²)²]  — measured
SIGMA_B_SQ = 0.089;     % per-sample baro  variance [m²]       — measured
SIGMA_G_SQ = (0.005)^2; % per-sample gyro  variance [(rad/s)²] — LSM6DSO32 spec

sa = sqrt(SIGMA_A_SQ);
sb = sqrt(SIGMA_B_SQ);
sg = sqrt(SIGMA_G_SQ);

%% ── Load & resample ──────────────────────────────────────────────────────
fprintf('Loading %s ...\n', CSV_PATH);
raw = load_openrocket(CSV_PATH);

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

for mc = 1:N_MC
    kf    = KF1D(alt(1), vel(1));
    q_hat = q0;

    for k = 1:N
        %──── IMU sim (matches IMU_Task → NAV::updateIMU) ────
        % 1. Noisy gyro → attitude estimate
        w_noisy = [wx(k); wy(k); wz(k)] + sg * randn(3,1);
        if k > 1
            q_hat = quat_integ(q_hat, w_noisy, dt);
        end

        % 2. Noisy specific force in body frame
        f_meas = f_body_true(:,k) + sa * randn(3,1);

        % 3. body → NED using estimated attitude, subtract gravity
        R_hat  = quat2rot(q_hat);
        a_NED  = R_hat * f_meas + g_NED;    % kinematic acc in NED
        acc_up = -a_NED(3);                  % up = -NED_Z

        %──── KF (matches KF1D::predict / update) ────
        if k > 1
            kf.predict(acc_up, dt);
        end
        if mod(k-1, bmpEvery) == 0
            kf.update(alt(k) + sb * randn());
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
    hold on; grid on;
    plot(t, err,      'Color', [0.85 0.85 0.85]);
    plot(t, +sigma3, 'r', 'LineWidth', 1.3);
    plot(t, -sigma3, 'r', 'LineWidth', 1.3);
end
