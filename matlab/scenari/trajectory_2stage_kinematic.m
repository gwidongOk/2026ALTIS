function tr = trajectory_2stage_kinematic(dt, case_name)
%TRAJECTORY_2STAGE_KINEMATIC  Synthetic two-stage rocket truth model.
%
% tr = trajectory_2stage_kinematic(dt, case_name)
%
% Produces the same fields used by montecarlo_sim.m's OpenRocket loader, but
% from a kinematic two-stage model with explicit staging events.  The cases
% below are intended to stress IMU+baro navigation without GPS/MAG.
%
% Cases:
%   nominal_800m      Baseline 2-stage flight, about 800 m apogee.
%   energy_envelope   Low-energy upper stage plus high-drag trajectory.
%   delayed_ignition  Upper stage ignites later after stage-1 burnout.
%   aero_tilt         Stronger pitch-over, lateral acceleration, and yaw kick.
%   harsh_transient   Strong staging shocks, vibration, and apogee disturbance.

    if nargin < 1 || isempty(dt)
        dt = 1 / 416;
    end
    if nargin < 2 || isempty(case_name)
        case_name = 'nominal_800m';
    end

    cfg = default_cfg();
    cfg.case_name = lower(string(case_name));

    switch cfg.case_name
        case {"nominal", "nominal_800m"}
        case {"energy_envelope", "low_energy", "weak_stage2", "high_drag"}
            cfg.a_stage2 = 34.0;
            cfg.stage2_delay = 0.70;
            cfg.stage2_burn = 2.45;
            cfg.c_drag = 1.45e-3;
        case {"delayed_ignition", "late_ignition"}
            cfg.stage2_delay = 0.90;
            cfg.a_stage2 = 45.0;
        case {"aero_tilt", "tilt_wind"}
            cfg.pitch_final_deg = 16.0;
            cfg.lateral_scale = 2.4;
            cfg.yaw_kick_degps = 28.0;
            cfg.c_drag = 1.20e-3;
        case {"harsh_transient", "harsh_staging", "chute_like_disturbance"}
            cfg.sep_shock = -110.0;
            cfg.ign_shock = 145.0;
            cfg.burnout_shock = -75.0;
            cfg.vibration_scale = 2.2;
            cfg.lateral_scale = 1.7;
            cfg.apogee_disturbance = true;
        otherwise
            error('trajectory_2stage_kinematic:UnknownCase', ...
                'Unknown kinematic case: %s', case_name);
    end

    g = 9.80665;
    t_stage1_burnout = cfg.stage1_burnout;
    t_stage2_ignite  = t_stage1_burnout + cfg.stage2_delay;
    t_stage2_burnout = t_stage2_ignite + cfg.stage2_burn;

    t = (0:dt:cfg.t_max).';
    n = numel(t);

    alt = zeros(n,1);
    vel = zeros(n,1);
    acc_up = zeros(n,1);
    thrust_acc_log = zeros(n,1);

    for k = 1:n-1
        tk = t(k);

        thrust_acc = 0.0;
        if tk < t_stage1_burnout
            thrust_acc = smooth_pulse(tk, 0.0, t_stage1_burnout, cfg.a_stage1);
        elseif tk >= t_stage2_ignite && tk < t_stage2_burnout
            thrust_acc = smooth_pulse(tk, t_stage2_ignite, t_stage2_burnout, cfg.a_stage2);
        end
        thrust_acc_log(k) = thrust_acc;

        drag_acc = cfg.c_drag * vel(k) * abs(vel(k));
        acc_up(k) = thrust_acc - g - drag_acc;

        % Short events designed to excite the adaptive-Q window.
        acc_up(k) = acc_up(k) ...
            + shock(tk, t_stage1_burnout, 0.025, cfg.sep_shock) ...
            + shock(tk, t_stage2_ignite,  0.030, cfg.ign_shock) ...
            + shock(tk, t_stage2_burnout, 0.025, cfg.burnout_shock);

        if tk < t_stage1_burnout
            acc_up(k) = acc_up(k) + cfg.vibration_scale * ...
                (1.8*sin(2*pi*45*tk) + 0.8*sin(2*pi*91*tk));
        elseif tk >= t_stage2_ignite && tk < t_stage2_burnout
            acc_up(k) = acc_up(k) + cfg.vibration_scale * ...
                (1.2*sin(2*pi*38*tk) + 0.5*sin(2*pi*77*tk));
        end

        if cfg.apogee_disturbance
            % A short negative acceleration pulse around the expected apogee
            % region, similar to a deployment-like shock or pressure transient.
            acc_up(k) = acc_up(k) + shock(tk, 13.5, 0.070, -38.0);
        end

        alt(k+1) = alt(k) + vel(k) * dt + 0.5 * acc_up(k) * dt^2;
        vel(k+1) = vel(k) + acc_up(k) * dt;

        if alt(k+1) < 0 && tk > 5.0
            t = t(1:k+1);
            alt = alt(1:k+1);
            vel = vel(1:k+1);
            acc_up = acc_up(1:k+1);
            thrust_acc_log = thrust_acc_log(1:k+1);
            alt(end) = 0.0;
            break;
        end
    end

    acc_up(end) = acc_up(max(end-1, 1));
    thrust_acc_log(end) = thrust_acc_log(max(end-1, 1));

    pitch_progress = smoothstep01(t / 3.0) + smoothstep01((t - t_stage2_ignite) / 5.0);
    zenith = cfg.zenith0_deg ...
        - 0.38 * cfg.pitch_final_deg * smoothstep01(t / 3.0) ...
        - 0.62 * cfg.pitch_final_deg * smoothstep01((t - t_stage2_ignite) / 5.0) ...
        + cfg.tilt_osc_deg * exp(-((t - t_stage2_ignite) / 0.20).^2) .* sin(2*pi*12*t);
    zenith = max(min(zenith, 90.0), 60.0);

    azimuth = 90.0 + cfg.azimuth_swing_deg * sin(0.35*t) ...
        + 0.8 * cfg.lateral_scale * smoothstep01((t - t_stage2_ignite) / 4.0);
    lat_dir = azimuth;
    lacc = cfg.lateral_scale * 0.12 * abs(acc_up) .* sind(90.0 - zenith);
    lacc = lacc ...
        + cfg.lateral_scale * 2.0 * exp(-((t - t_stage1_burnout) / 0.05).^2) ...
        + cfg.lateral_scale * 2.8 * exp(-((t - t_stage2_ignite)  / 0.06).^2);

    roll_rate  = cfg.roll_rate_degps + 8.0*cfg.vibration_scale*sin(2*pi*5.0*t);
    pitch_rate = gradient(90.0 - zenith, dt);
    yaw_rate   = 0.7 * sin(0.9*t) ...
        + cfg.yaw_kick_degps * exp(-((t - t_stage2_ignite) / 0.08).^2);

    [apogee_alt, i_apogee] = max(alt);

    tr.t       = t;
    tr.alt     = alt;
    tr.vvel    = vel;
    tr.vacc    = acc_up;
    tr.lacc    = lacc;
    tr.lat_dir = lat_dir;
    tr.gx_rad  = deg2rad(roll_rate);
    tr.gy_rad  = deg2rad(pitch_rate);
    tr.gz_rad  = deg2rad(yaw_rate);
    tr.zenith0  = zenith(1);
    tr.azimuth0 = azimuth(1);
    tr.zenith   = zenith;
    tr.azimuth  = azimuth;
    tr.thrust_acc = thrust_acc_log;
    tr.events = struct( ...
        'stage1_burnout', t_stage1_burnout, ...
        'stage2_ignite',  t_stage2_ignite, ...
        'stage2_burnout', t_stage2_burnout, ...
        'apogee', t(i_apogee));
    tr.info = struct( ...
        'case_name', char(cfg.case_name), ...
        'apogee_alt', apogee_alt, ...
        'apogee_time', t(i_apogee), ...
        'stage2_delay', cfg.stage2_delay, ...
        'stage2_burn', cfg.stage2_burn, ...
        'c_drag', cfg.c_drag);
    tr.source = 'kinematic_2stage';
end

function cfg = default_cfg()
    cfg.stage1_burnout = 1.70;
    cfg.stage2_delay = 0.50;
    cfg.stage2_burn = 2.60;
    cfg.t_max = 35.0;

    cfg.a_stage1 = 60.0;
    cfg.a_stage2 = 43.3;
    cfg.c_drag = 1.0e-3;

    cfg.sep_shock = -45.0;
    cfg.ign_shock = 65.0;
    cfg.burnout_shock = -30.0;
    cfg.vibration_scale = 1.0;

    cfg.zenith0_deg = 88.0;
    cfg.pitch_final_deg = 8.0;
    cfg.tilt_osc_deg = 0.6;
    cfg.azimuth_swing_deg = 1.0;
    cfg.lateral_scale = 1.0;
    cfg.roll_rate_degps = 80.0;
    cfg.yaw_kick_degps = 10.0;
    cfg.apogee_disturbance = false;
end

function y = smooth_pulse(t, t0, t1, amp)
    u = (t - t0) / (t1 - t0);
    u = min(max(u, 0.0), 1.0);
    edge = 0.08;
    rise = smoothstep01(u / edge);
    fall = 1.0 - smoothstep01((u - (1.0 - edge)) / edge);
    y = amp * rise * fall;
end

function y = shock(t, t0, width, amp)
    y = amp * exp(-((t - t0) / width).^2);
end

function y = smoothstep01(x)
    x = min(max(x, 0.0), 1.0);
    y = x .* x .* (3.0 - 2.0 .* x);
end
