%% kinematic_case_sweep.m
% Summarize and plot all synthetic two-stage kinematic truth cases.

clear; clc; close all;

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);

F_IMU = 416;
dt = 1 / F_IMU;

cases = {'nominal_800m', ...
         'energy_envelope', ...
         'delayed_ignition', ...
         'aero_tilt', ...
         'harsh_transient'};

summary = table('Size', [numel(cases), 8], ...
    'VariableTypes', {'string','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'case_name','apogee_m','apogee_s','stage2_ignite_s', ...
                      'stage2_burnout_s','max_vel_mps','max_acc_mps2','max_lacc_mps2'});

truths = cell(numel(cases), 1);

fprintf('\n=== Kinematic Two-Stage Case Sweep ===\n');
fprintf('%-24s %10s %10s %10s %10s %10s %10s\n', ...
    'Case', 'Apogee[m]', 't_apg[s]', 'Ign2[s]', 'Burn2[s]', 'Vmax', 'Amax');

for i = 1:numel(cases)
    tr = trajectory_2stage_kinematic(dt, cases{i});
    truths{i} = tr;

    summary.case_name(i) = string(cases{i});
    summary.apogee_m(i) = tr.info.apogee_alt;
    summary.apogee_s(i) = tr.info.apogee_time;
    summary.stage2_ignite_s(i) = tr.events.stage2_ignite;
    summary.stage2_burnout_s(i) = tr.events.stage2_burnout;
    summary.max_vel_mps(i) = max(tr.vvel);
    summary.max_acc_mps2(i) = max(abs(tr.vacc));
    summary.max_lacc_mps2(i) = max(abs(tr.lacc));

    fprintf('%-24s %10.1f %10.2f %10.2f %10.2f %10.1f %10.1f\n', ...
        cases{i}, summary.apogee_m(i), summary.apogee_s(i), ...
        summary.stage2_ignite_s(i), summary.stage2_burnout_s(i), ...
        summary.max_vel_mps(i), summary.max_acc_mps2(i));
end

assignin('base', 'kinematic_case_summary', summary);

figure('Name', 'Kinematic 2-Stage Truth Cases', 'Color', 'w', ...
       'Position', [80 80 1200 780]);
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile; hold on; grid on;
for i = 1:numel(cases)
    tr = truths{i};
    plot(tr.t, tr.alt, 'LineWidth', 1.1, 'DisplayName', cases{i});
end
ylabel('Altitude [m]');
title('Altitude');
legend('Location', 'eastoutside', 'Interpreter', 'none');

nexttile; hold on; grid on;
for i = 1:numel(cases)
    tr = truths{i};
    plot(tr.t, tr.vvel, 'LineWidth', 1.1, 'DisplayName', cases{i});
end
ylabel('Vertical velocity [m/s]');
title('Vertical velocity');

nexttile; hold on; grid on;
for i = 1:numel(cases)
    tr = truths{i};
    plot(tr.t, tr.vacc, 'LineWidth', 1.0, 'DisplayName', cases{i});
end
xlabel('Time [s]');
ylabel('Vertical acceleration [m/s^2]');
title('Vertical acceleration and disturbances');
