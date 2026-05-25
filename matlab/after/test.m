clear; clc; close all;

data_file = fullfile(fileparts(mfilename('fullpath')), '123.xlsx');
if ~isfile(data_file)
    error('Data file not found: %s', data_file);
end

state_tbl = readtable(data_file, 'Sheet', 'state_pkt', 'VariableNamingRule', 'preserve');
baro_tbl  = readtable(data_file, 'Sheet', 'baro_pkt',  'VariableNamingRule', 'preserve');
event_tbl = readtable(data_file, 'Sheet', 'event_pkt', 'VariableNamingRule', 'preserve');
imu_tbl   = readtable(data_file, 'Sheet', 'imu_pkt',   'VariableNamingRule', 'preserve');

t_event  = pickColumn(event_tbl, "t_event");
event_id = uint8(pickColumn(event_tbl, "event_id"));

launch_idx = find(event_id == 1, 1, 'first');
if isempty(launch_idx)
    error('event_id == 1 (Launch) event was not found.');
end
launch_time = t_event(launch_idx);

raw_t_state = pickColumn(state_tbl, "t_kf");
raw_t_baro  = pickColumn(baro_tbl, "t_alt");
raw_t_imu   = pickColumn(imu_tbl, "t_imu");
mission_end_time = max([ ...
    max(raw_t_state(raw_t_state >= launch_time)); ...
    max(raw_t_baro(raw_t_baro >= launch_time)); ...
    max(t_event(t_event >= launch_time))]);

launch_state_idx = find(raw_t_state >= launch_time, 1, 'first');
if isempty(launch_state_idx)
    error('No state_pkt sample exists after Launch time %.0f.', launch_time);
end
printStateIndexComparison(state_tbl, raw_t_state, launch_state_idx, launch_time);

[state_tbl, t_state, state_kept, state_dropped] = cutTableByTime(state_tbl, "t_kf", launch_time, mission_end_time, "state_pkt");
[baro_tbl,  t_baro,  baro_kept,  baro_dropped]  = cutTableByTime(baro_tbl,  "t_alt", launch_time, mission_end_time, "baro_pkt");
[imu_tbl,   t_imu,   imu_kept,   imu_dropped]   = cutTableByTime(imu_tbl,   "t_imu", launch_time, mission_end_time, "imu_pkt");

event_keep = t_event >= launch_time;
t_event = t_event(event_keep);
event_id = event_id(event_keep);
event_time_s = (t_event - launch_time) * 1e-6;
event_names = ["", "Launch", "Burnout", "Apogee", "Landing", "NotStageCondition", "Stage2Ignition"];

P_N  = pickColumn(state_tbl, "P_N");
P_E  = pickColumn(state_tbl, "P_E");
P_D  = pickColumn(state_tbl, "P_D");
V_N  = pickColumn(state_tbl, "V_N");
V_E  = pickColumn(state_tbl, "V_E");
V_D  = pickColumn(state_tbl, "V_D");
qw   = pickColumn(state_tbl, "qw");
qx   = pickColumn(state_tbl, "qx");
qy   = pickColumn(state_tbl, "qy");
qz   = pickColumn(state_tbl, "qz");

alt   = pickColumn(baro_tbl, "alt");

gx    = pickColumn(imu_tbl, "gx");
gy    = pickColumn(imu_tbl, "gy");
gz    = pickColumn(imu_tbl, "gz");
ax    = pickColumn(imu_tbl, "ax");
ay    = pickColumn(imu_tbl, "ay");
az    = pickColumn(imu_tbl, "az");

pos_N = P_N;
pos_E = P_E;
pos_D = P_D;
vel_N = V_N;
vel_E = V_E;
vel_D = V_D;
quat_w = qw;
quat_x = qx;
quat_y = qy;
quat_z = qz;

baro_alt = alt;

gyro_x = gx;
gyro_y = gy;
gyro_z = gz;
accel_x = ax;
accel_y = ay;
accel_z = az;
accel_mag = sqrt(double(accel_x).^2 + double(accel_y).^2 + double(accel_z).^2);

cos_tilt = 2 .* (quat_w .* quat_y - quat_x .* quat_z);
cos_tilt = min(max(cos_tilt, -1), 1);
tilt_deg = acosd(cos_tilt);

fig = figure('Name', 'Flight Analysis', 'NumberTitle', 'off');
tabs = uitabgroup(fig);

alt_tab = uitab(tabs, 'Title', 'Altitude');
alt_layout = tiledlayout(alt_tab, 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_alt = nexttile(alt_layout);
plot(ax_alt, t_state, -pos_D, 'LineWidth', 1.2, 'DisplayName', 'KF -P_D');
hold(ax_alt, 'on');
plot(ax_alt, t_baro, baro_alt, 'LineWidth', 1.2, 'DisplayName', 'Barometer alt');
grid(ax_alt, 'on');
xlabel(ax_alt, 'Time from Launch [s]');
ylabel(ax_alt, 'Altitude [m]');
title(ax_alt, 'Flight Altitude After Launch');
addEventLines(ax_alt, event_time_s, event_id, event_names, true);
legend(ax_alt, 'Location', 'best');

state_tab = uitab(tabs, 'Title', 'Position / Velocity');
state_layout = tiledlayout(state_tab, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_pos = nexttile(state_layout);
plot(ax_pos, t_state, [pos_N, pos_E, -pos_D], 'LineWidth', 1.1);
grid(ax_pos, 'on');
ylabel(ax_pos, 'Position [m]');
title(ax_pos, 'Position After Launch');
legend(ax_pos, 'P_N', 'P_E', '-P_D', 'Location', 'best');
addEventLines(ax_pos, event_time_s, event_id, event_names, false);

ax_vel = nexttile(state_layout);
plot(ax_vel, t_state, [vel_N, vel_E, -vel_D], 'LineWidth', 1.1);
grid(ax_vel, 'on');
xlabel(ax_vel, 'Time from Launch [s]');
ylabel(ax_vel, 'Velocity [m/s]');
title(ax_vel, 'Velocity After Launch');
legend(ax_vel, 'V_N', 'V_E', '-V_D', 'Location', 'best');
addEventLines(ax_vel, event_time_s, event_id, event_names, false);
linkaxes([ax_pos, ax_vel], 'x');

att_tab = uitab(tabs, 'Title', 'Attitude');
att_layout = tiledlayout(att_tab, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_quat = nexttile(att_layout);
plot(ax_quat, t_state, [quat_w, quat_x, quat_y, quat_z], 'LineWidth', 1.1);
grid(ax_quat, 'on');
ylabel(ax_quat, 'Quaternion');
title(ax_quat, 'Quaternion After Launch');
legend(ax_quat, 'qw', 'qx', 'qy', 'qz', 'Location', 'best');
addEventLines(ax_quat, event_time_s, event_id, event_names, false);

ax_tilt = nexttile(att_layout);
plot(ax_tilt, t_state, tilt_deg, 'LineWidth', 1.2);
grid(ax_tilt, 'on');
xlabel(ax_tilt, 'Time from Launch [s]');
ylabel(ax_tilt, 'Tilt [deg]');
title(ax_tilt, 'Tilt From Vertical');
addEventLines(ax_tilt, event_time_s, event_id, event_names, false);
linkaxes([ax_quat, ax_tilt], 'x');

imu_tab = uitab(tabs, 'Title', 'IMU');
imu_layout = tiledlayout(imu_tab, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ax_gyro = nexttile(imu_layout);
plot(ax_gyro, t_imu, [gyro_x, gyro_y, gyro_z], 'LineWidth', 1.0);
grid(ax_gyro, 'on');
ylabel(ax_gyro, 'Gyro raw');
title(ax_gyro, 'Gyroscope After Launch');
legend(ax_gyro, 'gx', 'gy', 'gz', 'Location', 'best');
addEventLines(ax_gyro, event_time_s, event_id, event_names, false);

ax_accel = nexttile(imu_layout);
plot(ax_accel, t_imu, [accel_x, accel_y, accel_z], 'LineWidth', 1.0);
grid(ax_accel, 'on');
ylabel(ax_accel, 'Accel raw');
title(ax_accel, 'Accelerometer After Launch');
legend(ax_accel, 'ax', 'ay', 'az', 'Location', 'best');
addEventLines(ax_accel, event_time_s, event_id, event_names, false);

ax_accel_mag = nexttile(imu_layout);
plot(ax_accel_mag, t_imu, accel_mag, 'LineWidth', 1.0);
grid(ax_accel_mag, 'on');
xlabel(ax_accel_mag, 'Time from Launch [s]');
ylabel(ax_accel_mag, '|Accel| raw');
title(ax_accel_mag, 'Accelerometer Magnitude');
addEventLines(ax_accel_mag, event_time_s, event_id, event_names, false);
linkaxes([ax_gyro, ax_accel, ax_accel_mag], 'x');

fprintf('Loaded: %s\n', data_file);
fprintf('Launch time: %.0f us\n', launch_time);
fprintf('Mission plot window: %.3f s\n', (mission_end_time - launch_time) * 1e-6);
fprintf('Kept rows: state_pkt=%d, baro_pkt=%d, imu_pkt=%d\n', ...
        state_kept, baro_kept, imu_kept);
fprintf('Dropped rows outside plot window: state_pkt=%d, baro_pkt=%d, imu_pkt=%d\n', ...
        state_dropped, baro_dropped, imu_dropped);
fprintf('Plotted %d state samples, %d baro samples, and %d imu samples after Launch.\n', ...
        numel(t_state), numel(t_baro), numel(t_imu));

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

function [cut_tbl, t_s, kept_count, dropped_count] = cutTableByTime(tbl, time_col, launch_time, end_time, sheet_name)
    t = pickColumn(tbl, time_col);
    keep = t >= launch_time & t <= end_time;
    if ~any(keep)
        error('No %s samples exist between Launch time %.0f and end time %.0f.', ...
              sheet_name, launch_time, end_time);
    end

    cut_tbl = tbl(keep, :);
    t_s = (t(keep) - launch_time) * 1e-6;
    kept_count = height(cut_tbl);
    dropped_count = height(tbl) - kept_count;
end

function printStateIndexComparison(state_tbl, t_state, launch_idx, launch_time)
    pos_N = pickColumn(state_tbl, "P_N");
    pos_E = pickColumn(state_tbl, "P_E");
    pos_D = pickColumn(state_tbl, "P_D");
    vel_N = pickColumn(state_tbl, "V_N");
    vel_E = pickColumn(state_tbl, "V_E");
    vel_D = pickColumn(state_tbl, "V_D");
    qw = pickColumn(state_tbl, "qw");
    qx = pickColumn(state_tbl, "qx");
    qy = pickColumn(state_tbl, "qy");
    qz = pickColumn(state_tbl, "qz");

    idx0 = 1;
    wait_s = (t_state(launch_idx) - t_state(idx0)) * 1e-6;
    launch_offset_s = (t_state(launch_idx) - launch_time) * 1e-6;

    p0 = [pos_N(idx0), pos_E(idx0), pos_D(idx0)];
    pL = [pos_N(launch_idx), pos_E(launch_idx), pos_D(launch_idx)];
    v0 = [vel_N(idx0), vel_E(idx0), vel_D(idx0)];
    vL = [vel_N(launch_idx), vel_E(launch_idx), vel_D(launch_idx)];
    q0 = [qw(idx0), qx(idx0), qy(idx0), qz(idx0)];
    qL = [qw(launch_idx), qx(launch_idx), qy(launch_idx), qz(launch_idx)];

    tilt0 = tiltFromQuat(q0);
    tiltL = tiltFromQuat(qL);
    quat_delta_deg = quatAngleDiffDeg(q0, qL);

    fprintf('\n=== state_pkt index 1 vs Launch sample ===\n');
    fprintf('index 1:      row=%d, t=%.0f us\n', idx0, t_state(idx0));
    fprintf('launch state: row=%d, t=%.0f us, launch_offset=%.6f s\n', ...
            launch_idx, t_state(launch_idx), launch_offset_s);
    fprintf('pre-launch wait from state index 1: %.3f s\n', wait_s);
    fprintf('Position [N E D] index1 = [% .6f % .6f % .6f] m\n', p0);
    fprintf('Position [N E D] launch = [% .6f % .6f % .6f] m\n', pL);
    fprintf('Position drift [N E D]  = [% .6f % .6f % .6f] m, horizontal=%.6f m\n', ...
            pL - p0, hypot(pL(1) - p0(1), pL(2) - p0(2)));
    fprintf('Velocity [N E D] index1 = [% .6f % .6f % .6f] m/s\n', v0);
    fprintf('Velocity [N E D] launch = [% .6f % .6f % .6f] m/s\n', vL);
    fprintf('Velocity drift [N E D]  = [% .6f % .6f % .6f] m/s, horizontal=%.6f m/s\n', ...
            vL - v0, hypot(vL(1) - v0(1), vL(2) - v0(2)));
    fprintf('Quaternion index1 = [% .6f % .6f % .6f % .6f]\n', q0);
    fprintf('Quaternion launch = [% .6f % .6f % .6f % .6f]\n', qL);
    fprintf('Quaternion angle delta = %.3f deg\n', quat_delta_deg);
    fprintf('Tilt index1=%.3f deg, launch=%.3f deg, delta=%.3f deg\n', ...
            tilt0, tiltL, tiltL - tilt0);
    fprintf('==========================================\n\n');
end

function tilt = tiltFromQuat(q)
    q = q ./ norm(q);
    cos_tilt = 2 * (q(1) * q(3) - q(2) * q(4));
    cos_tilt = min(max(cos_tilt, -1), 1);
    tilt = acosd(cos_tilt);
end

function angle_deg = quatAngleDiffDeg(q0, q1)
    q0 = q0 ./ norm(q0);
    q1 = q1 ./ norm(q1);
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
