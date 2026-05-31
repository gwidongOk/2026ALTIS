clear; clc; close all;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..'));

data_file = fullfile(this_dir, 'flight.xlsx');
if ~isfile(data_file)
    error('Data file not found: %s', data_file);
end

G = 9.80665;
GYRO_MDPS_TO_RAD = pi / 180 / 1000;
ACCEL_MG_TO_MPS2 = G / 1000;

global SIGMA_A_SQ SIGMA_B_SQ
SIGMA_A_SQ = 1.0e-3;
SIGMA_B_SQ = 0.089;

state_raw = readtable(data_file, 'Sheet', 'state_pkt', 'VariableNamingRule', 'preserve');
baro_raw  = readtable(data_file, 'Sheet', 'baro_pkt',  'VariableNamingRule', 'preserve');
event_raw = readtable(data_file, 'Sheet', 'event_pkt', 'VariableNamingRule', 'preserve');
imu_raw   = readtable(data_file, 'Sheet', 'imu_pkt',   'VariableNamingRule', 'preserve');

t_event_raw = double(pickColumn(event_raw, "t"));
event_id = uint8(pickColumn(event_raw, "event_id"));

launch_idx = find(event_id == 1, 1, 'first');
if isempty(launch_idx)
    error('event_id == 1 (Launch) event was not found.');
end

launch_time = t_event_raw(launch_idx);
t_end_event = max(t_event_raw(t_event_raw >= launch_time));

state_raw = cleanTimeTable(state_raw, "t", launch_time);
baro_raw  = cleanTimeTable(baro_raw,  "t", launch_time);
imu_raw   = cleanTimeTable(imu_raw,   "t", launch_time);

t_state_raw = double(pickColumn(state_raw, "t"));
t_baro_raw  = double(pickColumn(baro_raw,  "t"));
t_imu_raw   = double(pickColumn(imu_raw,   "t"));

mission_end_time = min([max(t_state_raw), max(t_baro_raw), max(t_imu_raw)]);
mission_end_time = max(mission_end_time, t_end_event);

[state_tbl, t_state] = cutTableByTime(state_raw, "t", launch_time, mission_end_time);
[baro_tbl,  t_baro]  = cutTableByTime(baro_raw,  "t", launch_time, mission_end_time);
[imu_tbl,   t_imu]   = cutTableByTime(imu_raw,   "t", launch_time, mission_end_time);

event_keep = t_event_raw >= launch_time & t_event_raw <= mission_end_time;
t_event = (t_event_raw(event_keep) - launch_time) * 1e-6;
event_id = event_id(event_keep);
event_names = ["", "Launch", "Burnout", "Apogee", "Landing", "NotStageCondition", "Stage2Ignition"];

q_init = normalizeQuat([
    firstValue(state_raw, "qw");
    firstValue(state_raw, "qx");
    firstValue(state_raw, "qy");
    firstValue(state_raw, "qz")]);

q_launch_log = normalizeQuat([
    firstValue(state_tbl, "qw");
    firstValue(state_tbl, "qx");
    firstValue(state_tbl, "qy");
    firstValue(state_tbl, "qz")]);

gx = double(pickColumn(imu_tbl, "gx")) * GYRO_MDPS_TO_RAD;
gy = double(pickColumn(imu_tbl, "gy")) * GYRO_MDPS_TO_RAD;
gz = double(pickColumn(imu_tbl, "gz")) * GYRO_MDPS_TO_RAD;
ax = double(pickColumn(imu_tbl, "ax")) * ACCEL_MG_TO_MPS2;
ay = double(pickColumn(imu_tbl, "ay")) * ACCEL_MG_TO_MPS2;
az = double(pickColumn(imu_tbl, "az")) * ACCEL_MG_TO_MPS2;
t_imu_abs = double(pickColumn(imu_tbl, "t"));

baro_alt = double(pickColumn(baro_tbl, "alt"));
t_baro_abs = double(pickColumn(baro_tbl, "t"));

log_alt = -double(pickColumn(state_tbl, "pD"));
log_vup = -double(pickColumn(state_tbl, "vD"));
log_q = [
    double(pickColumn(state_tbl, "qw")).';
    double(pickColumn(state_tbl, "qx")).';
    double(pickColumn(state_tbl, "qy")).';
    double(pickColumn(state_tbl, "qz")).'];

init_alt = log_alt(1);
init_vup = log_vup(1);

[kf_alt, kf_vel, acc_up, q_sim] = simulateKF(t_imu, t_imu_abs, gx, gy, gz, ax, ay, az, ...
                                             t_baro_abs, baro_alt, q_init, init_alt, init_vup);

tilt_log = quatTiltDeg(log_q);
tilt_sim = quatTiltDeg(q_sim);

fig = figure('Name', 'Flight XLS KF Replay', 'NumberTitle', 'off');
tabs = uitabgroup(fig);

alt_tab = uitab(tabs, 'Title', 'Altitude');
alt_layout = tiledlayout(alt_tab, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_alt = nexttile(alt_layout);
plot(ax_alt, t_state, log_alt, 'LineWidth', 1.1, 'DisplayName', 'Logged KF');
hold(ax_alt, 'on');
plot(ax_alt, t_imu, kf_alt, 'LineWidth', 1.2, 'DisplayName', 'Replay KF');
plot(ax_alt, t_baro, baro_alt, '.', 'DisplayName', 'Baro');
grid(ax_alt, 'on');
ylabel(ax_alt, 'Altitude [m]');
title(ax_alt, 'Launch-after KF Replay');
legend(ax_alt, 'Location', 'best');
addEventLines(ax_alt, t_event, event_id, event_names, true);

ax_vel = nexttile(alt_layout);
plot(ax_vel, t_state, log_vup, 'LineWidth', 1.1, 'DisplayName', 'Logged V_{up}');
hold(ax_vel, 'on');
plot(ax_vel, t_imu, kf_vel, 'LineWidth', 1.2, 'DisplayName', 'Replay KF V_{up}');
grid(ax_vel, 'on');
xlabel(ax_vel, 'Time from Launch [s]');
ylabel(ax_vel, 'Velocity [m/s]');
legend(ax_vel, 'Location', 'best');
addEventLines(ax_vel, t_event, event_id, event_names, false);
linkaxes([ax_alt, ax_vel], 'x');

att_tab = uitab(tabs, 'Title', 'Attitude');
att_layout = tiledlayout(att_tab, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_quat = nexttile(att_layout);
plot(ax_quat, t_state, log_q.', 'LineWidth', 1.0);
hold(ax_quat, 'on');
plot(ax_quat, t_imu, q_sim.', '--', 'LineWidth', 1.0);
grid(ax_quat, 'on');
ylabel(ax_quat, 'Quaternion');
title(ax_quat, 'Logged Attitude vs Replay Attitude');
legend(ax_quat, 'log qw', 'log qx', 'log qy', 'log qz', ...
                'sim qw', 'sim qx', 'sim qy', 'sim qz', 'Location', 'best');
addEventLines(ax_quat, t_event, event_id, event_names, false);

ax_tilt = nexttile(att_layout);
plot(ax_tilt, t_state, tilt_log, 'LineWidth', 1.1, 'DisplayName', 'Logged');
hold(ax_tilt, 'on');
plot(ax_tilt, t_imu, tilt_sim, 'LineWidth', 1.1, 'DisplayName', 'Replay');
grid(ax_tilt, 'on');
xlabel(ax_tilt, 'Time from Launch [s]');
ylabel(ax_tilt, 'Tilt [deg]');
title(ax_tilt, 'Tilt From Vertical');
legend(ax_tilt, 'Location', 'best');
addEventLines(ax_tilt, t_event, event_id, event_names, false);
linkaxes([ax_quat, ax_tilt], 'x');

imu_tab = uitab(tabs, 'Title', 'IMU');
imu_layout = tiledlayout(imu_tab, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_gyro = nexttile(imu_layout);
plot(ax_gyro, t_imu, [gx, gy, gz], 'LineWidth', 1.0);
grid(ax_gyro, 'on');
ylabel(ax_gyro, 'Gyro [rad/s]');
title(ax_gyro, 'Gyroscope');
legend(ax_gyro, 'gx', 'gy', 'gz', 'Location', 'best');
addEventLines(ax_gyro, t_event, event_id, event_names, false);

ax_accel = nexttile(imu_layout);
plot(ax_accel, t_imu, [ax, ay, az], 'LineWidth', 1.0);
grid(ax_accel, 'on');
ylabel(ax_accel, 'Accel [m/s^2]');
title(ax_accel, 'Accelerometer');
legend(ax_accel, 'ax', 'ay', 'az', 'Location', 'best');
addEventLines(ax_accel, t_event, event_id, event_names, false);

ax_acc_up = nexttile(imu_layout);
plot(ax_acc_up, t_imu, acc_up, 'LineWidth', 1.0);
grid(ax_acc_up, 'on');
xlabel(ax_acc_up, 'Time from Launch [s]');
ylabel(ax_acc_up, 'Up accel [m/s^2]');
title(ax_acc_up, 'KF Input Acceleration');
addEventLines(ax_acc_up, t_event, event_id, event_names, false);
linkaxes([ax_gyro, ax_accel, ax_acc_up], 'x');

fprintf('Loaded: %s\n', data_file);
fprintf('Launch time: %.0f us\n', launch_time);
fprintf('Replay window: %.3f s\n', (mission_end_time - launch_time) * 1e-6);
fprintf('Initial attitude source: state_pkt row 1\n');
fprintf('q_init       = [% .6f % .6f % .6f % .6f]\n', q_init);
fprintf('q_launch_log = [% .6f % .6f % .6f % .6f]\n', q_launch_log);
fprintf('q drift angle before launch: %.3f deg\n', quatAngleDiffDeg(q_init, q_launch_log));
fprintf('KF init [alt, vup] = [% .3f m, % .3f m/s]\n', init_alt, init_vup);
fprintf('Samples: imu=%d, baro=%d, state=%d\n', numel(t_imu), numel(t_baro), numel(t_state));

function [kf_alt, kf_vel, acc_up, q_hist] = simulateKF(t_imu, t_imu_abs, gx, gy, gz, ax, ay, az, ...
                                                       t_baro_abs, baro_alt, q0, p0, v0)
    G = 9.80665;
    g_ned = [0; 0; G];

    kf = KF1D(p0, v0);
    q = q0;
    baro_idx = 1;

    n = numel(t_imu);
    kf_alt = zeros(n, 1);
    kf_vel = zeros(n, 1);
    acc_up = zeros(n, 1);
    q_hist = zeros(4, n);

    for k = 1:n
        if k > 1
            dt = t_imu(k) - t_imu(k - 1);
            q = quatInteg(q, [gx(k); gy(k); gz(k)], dt);
        else
            dt = 0;
        end

        R = quatToRot(q);
        f_body = [ax(k); ay(k); az(k)];
        a_ned = R * f_body + g_ned;
        acc_up(k) = -a_ned(3);

        if k > 1
            kf.predict(acc_up(k), dt);
        end

        while baro_idx <= numel(t_baro_abs) && t_baro_abs(baro_idx) <= t_imu_abs(k)
            kf.update(baro_alt(baro_idx));
            baro_idx = baro_idx + 1;
        end

        kf_alt(k) = kf.x(1);
        kf_vel(k) = kf.x(2);
        q_hist(:, k) = q;
    end
end

function tbl = cleanTimeTable(tbl, time_col, launch_time)
    t = double(pickColumn(tbl, time_col));
    finite = isfinite(t);
    tbl = tbl(finite, :);
    t = t(finite);

    if isempty(t)
        error('Empty table after finite time filtering.');
    end

    dt = [Inf; diff(t)];
    valid_order = dt > 0 | (t >= launch_time & t < launch_time + 120e6);
    valid_range = t < launch_time + 120e6;
    tbl = tbl(valid_order & valid_range, :);
end

function [cut_tbl, t_s] = cutTableByTime(tbl, time_col, launch_time, end_time)
    t = double(pickColumn(tbl, time_col));
    keep = t >= launch_time & t <= end_time;
    if ~any(keep)
        error('No samples exist between %.0f and %.0f us.', launch_time, end_time);
    end
    cut_tbl = tbl(keep, :);
    t_s = (t(keep) - launch_time) * 1e-6;
end

function values = pickColumn(tbl, names)
    var_names = string(tbl.Properties.VariableNames);
    for name = string(names)
        idx = find(strcmpi(var_names, name), 1, 'first');
        if ~isempty(idx)
            values = tbl{:, idx};
            return;
        end
    end
    error('Missing column. Tried: %s', strjoin(string(names), ', '));
end

function value = firstValue(tbl, name)
    values = pickColumn(tbl, name);
    value = double(values(1));
end

function q = normalizeQuat(q)
    q = double(q(:));
    q = q / norm(q);
end

function q = quatInteg(q, w, dt)
    w_norm = norm(w);
    if w_norm > 1e-6
        half_angle = 0.5 * w_norm * dt;
        axis = w / w_norm;
        dq = [cos(half_angle); sin(half_angle) * axis];
    else
        dq = [1; 0.5 * w * dt];
    end
    q = quatMul(q, dq / norm(dq));
    q = q / norm(q);
end

function r = quatMul(p, q)
    pw = p(1); px = p(2); py = p(3); pz = p(4);
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    r = [pw*qw - px*qx - py*qy - pz*qz;
         pw*qx + px*qw + py*qz - pz*qy;
         pw*qy - px*qz + py*qw + pz*qx;
         pw*qz + px*qy - py*qx + pz*qw];
end

function R = quatToRot(q)
    q = q / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    R = [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y);
         2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x);
         2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)];
end

function tilt = quatTiltDeg(q)
    n = size(q, 2);
    tilt = zeros(n, 1);
    for k = 1:n
        qk = q(:, k) / norm(q(:, k));
        cos_tilt = 2 * (qk(1) * qk(3) - qk(2) * qk(4));
        cos_tilt = min(max(cos_tilt, -1), 1);
        tilt(k) = acosd(cos_tilt);
    end
end

function angle_deg = quatAngleDiffDeg(q0, q1)
    q0 = q0(:) / norm(q0);
    q1 = q1(:) / norm(q1);
    dot_q = abs(dot(q0, q1));
    dot_q = min(max(dot_q, -1), 1);
    angle_deg = 2 * acosd(dot_q);
end

function addEventLines(ax, event_time_s, event_id, event_names, show_labels)
    for i = 1:numel(event_time_s)
        if event_time_s(i) < 0
            continue;
        end

        if event_id(i) >= 1 && event_id(i) < numel(event_names)
            event_name = event_names(double(event_id(i)) + 1);
        else
            event_name = "Event";
        end

        xline(ax, event_time_s(i), '--', 'DisplayName', event_name);
        if show_labels && event_id(i) >= 1 && event_id(i) < numel(event_names)
            limits = ylim(ax);
            text(ax, event_time_s(i), limits(2), event_name, ...
                 'Rotation', 90, 'VerticalAlignment', 'top', ...
                 'HorizontalAlignment', 'right');
        end
    end
end
