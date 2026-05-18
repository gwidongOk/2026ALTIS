clear; clc; close all;

% Hop to script's directory so relative paths work from anywhere.
this_dir = fileparts(mfilename('fullpath'));
if ~isempty(this_dir), cd(this_dir); end
addpath(fullfile(this_dir, '..'));    % KF1D.m (optional, for native compare)

% ── Configuration ───────────────────────────────────────────────────
CSV_PATH    = 'openrocket.csv';
PORT        = "COM7";
BAUD        = 921600;
SEED        = 42;
ACCEL_NOISE = 0.025;                  % 1-σ m/s²
GYRO_NOISE  = 0.005;                  % 1-σ rad/s
BARO_NOISE  = 0.30;                   % 1-σ m
DUMP_FILE   = 'flight.bin';
DO_ERASE    = true;

% Firmware-matched constants
ACCEL_SCALE = 0.976e-3 * 9.80665;     % m/s² per LSB (LSM6DSO32 ±32g)
GYRO_SCALE  = 70.0e-3 * pi/180;       % rad/s per LSB (LSM6DSO32 ±2000dps)
G           = 9.80665;
IMU_RATE_HZ = 416;
BMP_RATE_HZ = 50;

REQ_IMU   = uint8(1);
REQ_BARO  = uint8(2);
DONE_MARK = uint8(253); % 0xFD

% ── 1) Ground truth from OpenRocket ─────────────────────────────────
fprintf('[1/5] Loading %s ...\n', CSV_PATH);
truth = load_openrocket(CSV_PATH);
fprintf('   rows=%d  t_end=%.2fs  alt_max=%.1f m  v_max=%.1f m/s\n', ...
        numel(truth.t), truth.t(end), max(truth.alt), max(truth.vvel));

% ── 2) Build IMU and BARO sample streams (in MATLAB memory, no PSRAM) ──
fprintf('[2/5] Building noisy sample streams ...\n');
rng(SEED);
[imu_lsb, bmp_alt] = build_streams(truth, IMU_RATE_HZ, BMP_RATE_HZ, ...
                                   ACCEL_NOISE, GYRO_NOISE, BARO_NOISE, ...
                                   ACCEL_SCALE, GYRO_SCALE, G);
n_imu = size(imu_lsb, 1);
n_bmp = numel(bmp_alt);
fprintf('   IMU samples=%d   BARO samples=%d\n', n_imu, n_bmp);

% ── 3) Open serial + run lockstep flight ────────────────────────────
fprintf('[3/5] Opening serial %s @ %d ...\n', PORT, BAUD);
sp = serialport(PORT, BAUD, "Timeout", 5);
% Larger MATLAB-side RX buffer prevents byte loss while fprintf runs.
try
    sp.InputBufferSize = 65536;
catch
    % older MATLAB releases — silently ignore
end
flush(sp); pause(0.5); flush(sp);
cleanup = onCleanup(@() delete(sp));

run_lockstep(sp, imu_lsb, bmp_alt, DUMP_FILE, DO_ERASE, ...
             REQ_IMU, REQ_BARO, DONE_MARK);

% ── 4) Parse flash dump ─────────────────────────────────────────────
fprintf('[4/5] Parsing %s ...\n', DUMP_FILE);
[nav, baro, imu_pkts, events] = parse_dump(DUMP_FILE);
fprintf('   BARO=%d  IMU=%d  STATE=%d  EVENT=%d\n', ...
        size(baro,1), size(imu_pkts,1), size(nav,1), size(events,1));

write_nav_csv(nav, 'flight_nav.csv');
write_event_csv(events, 'flight_events.csv');

% ── 5) Compare + plot ───────────────────────────────────────────────
fprintf('[5/5] Comparing vs OpenRocket ...\n');
print_events(events);
compare(nav, truth);
plot_results(nav, truth, events);

fprintf('\nDONE.\n');


% =====================================================================
%  LOCAL FUNCTIONS
% =====================================================================

function tr = load_openrocket(path)
    if exist(path, 'file') ~= 2
        error('CSV not found: "%s" (cwd=%s)', path, pwd);
    end
    fid = fopen(path, 'r');
    if fid < 0, error('Could not open CSV: %s', path); end
    cleanup_fid = onCleanup(@() fclose(fid));
    hdr_line = '';
    while true
        line = fgetl(fid);
        if ~ischar(line), error('No data rows in CSV: %s', path); end
        if startsWith(strtrim(line), '#')
            if contains(line, 'Time (s)'), hdr_line = line; end
            continue;
        end
        break;
    end
    clear cleanup_fid;
    if isempty(hdr_line)
        error('CSV header line containing "Time (s)" not found');
    end
    raw = readmatrix(path, 'CommentStyle', '#');
    raw(~isfinite(raw)) = 0;

    hdr_line = regexprep(hdr_line, '^\s*#\s*', '');
    cols = strtrim(strsplit(hdr_line, ','));

    find_col = @(varargin) local_find(cols, varargin);

    i_t   = find_col('Time');
    i_alt = find_col('Altitude (m)');
    i_vv  = find_col('Vertical velocity');
    i_va  = find_col('Vertical acceleration');
    i_la  = find_col('Lateral acceleration');
    i_rr  = find_col('Roll rate');
    i_pr  = find_col('Pitch rate');
    i_yr  = find_col('Yaw rate');
    i_zen = find_col('Vertical orientation', 'zenith');

    if isempty(i_t) || isempty(i_alt) || isempty(i_va) || isempty(i_zen)
        error('CSV missing required columns (Time/Altitude/Vertical accel/zenith)');
    end

    tr.t      = raw(:, i_t);
    tr.alt    = raw(:, i_alt);
    tr.vvel   = local_col(raw, i_vv);
    tr.vacc   = raw(:, i_va);
    tr.lacc   = local_col(raw, i_la);
    tr.roll   = local_col(raw, i_rr);
    tr.pitch  = local_col(raw, i_pr);
    tr.yaw    = local_col(raw, i_yr);
    tr.zenith = raw(:, i_zen);
end

function i = local_find(cols, names)
    i = [];
    for c = 1:numel(cols)
        for k = 1:numel(names)
            if contains(cols{c}, names{k}, 'IgnoreCase', true)
                i = c; return;
            end
        end
    end
end

function v = local_col(raw, idx)
    if isempty(idx), v = zeros(size(raw,1), 1); else, v = raw(:, idx); end
end


function [imu_lsb, bmp_alt] = build_streams(truth, imu_hz, bmp_hz, ...
                                            an, gn, bn, ascale, gscale, g)
    % World→body specific force.
    % Firmware assumes Body X points out the nosecone (UP).
    % When zenith = 0, nose points straight up.
    theta  = deg2rad(truth.zenith);
    f_vert = truth.vacc + g;
    
    % Rotate specific force into body frame.
    % If zenith=0: ax = f_vert, az = lacc
    % If zenith=90: ax = lacc, az = -f_vert
    ax = cos(theta) .* f_vert + sin(theta) .* truth.lacc;
    ay = zeros(size(ax));
    az = cos(theta) .* truth.lacc - sin(theta) .* f_vert;

    gx_rad = deg2rad(truth.roll);
    gy_rad = deg2rad(truth.pitch);
    gz_rad = deg2rad(truth.yaw);

    t_end = truth.t(end);

    % IMU @ imu_hz
    n_imu = floor(t_end * imu_hz);
    t_imu = (0:n_imu-1).' / imu_hz;
    ax_i = interp1(truth.t, ax,     t_imu, 'linear','extrap') + an*randn(n_imu,1);
    ay_i = interp1(truth.t, ay,     t_imu, 'linear','extrap') + an*randn(n_imu,1);
    az_i = interp1(truth.t, az,     t_imu, 'linear','extrap') + an*randn(n_imu,1);
    gx_i = interp1(truth.t, gx_rad, t_imu, 'linear','extrap') + gn*randn(n_imu,1);
    gy_i = interp1(truth.t, gy_rad, t_imu, 'linear','extrap') + gn*randn(n_imu,1);
    gz_i = interp1(truth.t, gz_rad, t_imu, 'linear','extrap') + gn*randn(n_imu,1);

    clip16 = @(x) int16(min(max(round(x), -32768), 32767));
    imu_lsb = zeros(n_imu, 6, 'int16');
    imu_lsb(:,1) = clip16(gx_i / gscale);
    imu_lsb(:,2) = clip16(gy_i / gscale);
    imu_lsb(:,3) = clip16(gz_i / gscale);
    imu_lsb(:,4) = clip16(ax_i / ascale);
    imu_lsb(:,5) = clip16(ay_i / ascale);
    imu_lsb(:,6) = clip16(az_i / ascale);

    % BARO @ bmp_hz
    n_bmp = floor(t_end * bmp_hz);
    t_bmp = (0:n_bmp-1).' / bmp_hz;
    bmp_alt = single(interp1(truth.t, truth.alt, t_bmp, 'linear','extrap') ...
                     + bn*randn(n_bmp,1));
end


function run_lockstep(sp, imu_lsb, bmp_alt, dump_path, do_erase, ...
                      ID_IMU, ID_BARO, DONE_MARK)
    if do_erase
        send_cmd(sp, "ERASE");
        fprintf('   (chip erase 1-3 min for 32MB)\n');
        if ~wait_for(sp, 'DONE.', 300), error('ERASE timeout'); end
    end

    send_cmd(sp, "HIL");
    if ~wait_for(sp, 'WAITING FOR HEADER', 10), error('HIL prompt timeout'); end

    % Send 8-byte header [n_imu, n_bmp]
    n_imu = uint32(size(imu_lsb, 1));
    n_bmp = uint32(numel(bmp_alt));
    write(sp, typecast([n_imu n_bmp], 'uint8'), 'uint8');
    fprintf('   header sent: imu=%d  bmp=%d\n', n_imu, n_bmp);
    
    if ~wait_for(sp, 'HIL READY', 30), error('HIL READY timeout'); end

    send_cmd(sp, "CALIBRATE");
    if ~wait_for(sp, 'CALIBRATION SKIPPED', 5), error('CALIBRATE timeout'); end

    send_cmd(sp, "START");
    % Do NOT wait for 'FLIGHT ACTIVE' here. ESP32's kfBegin() waits for 
    % IMU samples before it sends 'FLIGHT ACTIVE'. If we wait here, we deadlock.

    fprintf('   Pushing samples (imu=%d, bmp=%d)...\n', n_imu, n_bmp);

    % Precompute byte payloads
    imu_bytes = typecast(reshape(imu_lsb.', 1, []), 'uint8');   % 12B per sample
    bmp_bytes = typecast(bmp_alt(:).', 'uint8');                % 4B per sample

    t0 = tic;
    last_print = tic;
    
    imu_ptr = 0;
    bmp_ptr = 0;
    text_buf = '';
    
    while imu_ptr < n_imu || bmp_ptr < n_bmp
        % Typical flight: 8 IMU then 1 BARO
        for i = 1:8
            if imu_ptr < n_imu
                off = imu_ptr * 12;
                write(sp, [ID_IMU, imu_bytes(off+1 : off+12)], 'uint8');
                imu_ptr = imu_ptr + 1;
            end
        end
        
        if bmp_ptr < n_bmp
            off = bmp_ptr * 4;
            write(sp, [ID_BARO, bmp_bytes(off+1 : off+4)], 'uint8');
            bmp_ptr = bmp_ptr + 1;
        end
        
        % Capture text output from firmware (KF READY, FLIGHT ACTIVE, etc.)
        while sp.NumBytesAvailable > 0
            b = read(sp, 1, 'uint8');
            if b == 10
                if ~isempty(text_buf)
                    fprintf('   < %s\n', text_buf);
                    text_buf = '';
                end
            elseif b ~= 13
                text_buf = [text_buf, char(b)];
            end
        end

        if toc(last_print) > 2
            elapsed = toc(t0);
            pct = 100 * max(double(imu_ptr)/double(n_imu), double(bmp_ptr)/double(n_bmp));
            fprintf('   ... imu %d/%d  bmp %d/%d  (%.1f%%, %.0fs elapsed)\n', ...
                    imu_ptr, n_imu, bmp_ptr, n_bmp, pct, elapsed);
            last_print = tic;
        end
        
        % Small pause to allow ESP32 to process and avoid serial buffer overflow
        % 921600 baud is ~115KB/s. 109 bytes (8 IMU + 1 BARO) takes ~1ms.
        % We send ~400 IMU/s, so we should take ~2ms for this loop.
        pause(0.001); 
    end

    fprintf('   Sending DONE mark...\n');
    write(sp, DONE_MARK, 'uint8');
    
    if ~wait_for(sp, 'HIL DONE', 10), fprintf('   [HIL DONE timeout]\n'); end
    
    fprintf('   Simulation complete in %.1fs\n', toc(t0));

    send_cmd(sp, "STOP");
    wait_for(sp, 'STOPPED.', 10);

    send_cmd(sp, "PARSE");
    collect_dump(sp, dump_path);
end


function send_cmd(sp, cmd)
    fprintf('   > %s\n', cmd);
    writeline(sp, cmd);
end

function ok = wait_for(sp, needle, timeout_s)
    if nargin < 3, timeout_s = 30; end
    buf = char.empty(1, 0);          % always 1×N char row
    t_end = tic;
    ok = false;
    while toc(t_end) < timeout_s
        n = sp.NumBytesAvailable;
        if n > 0
            data = read(sp, n, 'uint8');
            if iscolumn(data), data = data.'; end   % force row
            buf = [buf, char(data)]; %#ok<AGROW>
            nl_pos = find(buf == newline);
            while ~isempty(nl_pos)
                line = strtrim(buf(1:nl_pos(1)-1));
                buf  = buf(nl_pos(1)+1:end);
                if ~isempty(line)
                    fprintf('   < %s\n', line);
                    if contains(string(line), needle)
                        ok = true; return;
                    end
                end
                nl_pos = find(buf == newline);
            end
        end
    end
end


function collect_dump(sp, out_path)
    fprintf('   Capturing dump to %s ...\n', out_path);
    fid = fopen(out_path, 'wb');
    cleanup_fid = onCleanup(@() fclose(fid));
    buf = uint8([]);
    state = "wait_start";
    t_start_deadline = tic;
    t_last = tic;
    idle_timeout = 2.0;
    while true
        if sp.NumBytesAvailable > 0
            data = read(sp, sp.NumBytesAvailable, 'uint8');
            if isrow(data), data = data.'; end       % force column uint8
            buf = [buf; data]; %#ok<AGROW>
            t_last = tic;
            t_start_deadline = tic;
        elseif state == "wait_start" && toc(t_start_deadline) > 60
            error('DUMP START not received');
        elseif state == "capturing" && toc(t_last) > idle_timeout
            break;
        end

        if state == "wait_start"
            idx = strfind(char(buf(:).'), 'DUMP START');
            if ~isempty(idx)
                nl = find(buf(idx(1):end) == uint8(10), 1, 'first');
                if ~isempty(nl)
                    buf(1 : idx(1)+nl-1) = [];
                    state = "capturing";
                    fprintf('   DUMP START — capturing ...\n');
                end
            end
        end

        if state == "capturing"
            idx = strfind(char(buf(:).'), 'DUMP DONE');
            if ~isempty(idx)
                payload = buf(1 : idx(1)-1);
                while ~isempty(payload) && (payload(end) == 10 || payload(end) == 13)
                    payload(end) = [];
                end
                fwrite(fid, payload, 'uint8');
                fprintf('   DUMP DONE: %d bytes\n', numel(payload));
                return;
            end
        end
    end
    if state == "capturing"
        fwrite(fid, buf, 'uint8');
        fprintf('   (idle timeout) saved %d bytes\n', numel(buf));
    end
end


function [nav, baro, imu_pkts, events] = parse_dump(path)
    fid = fopen(path, 'rb');
    raw = fread(fid, inf, '*uint8');
    fclose(fid);

    SYNC = uint8(0xAA);
    ID_BARO=1; SZ_BARO=11;
    ID_IMU=2;  SZ_IMU=19;
    ID_STATE=5; SZ_STATE=47;
    ID_EVENT=6; SZ_EVENT=9;
    expected = containers.Map({ID_BARO,ID_IMU,ID_STATE,ID_EVENT}, ...
                              {SZ_BARO,SZ_IMU,SZ_STATE,SZ_EVENT});

    % Pre-allocate generously (trim at the end) — dynamic growth via end+1 is
    % O(N^2) and chokes on multi-MB dumps.
    n      = numel(raw);
    cap_n  = max(1, n);                       % upper bound: one packet per byte
    nav      = zeros(cap_n, 11);
    baro     = zeros(cap_n, 2);
    imu_pkts = zeros(cap_n, 7);
    events   = zeros(cap_n, 3);
    n_nav = 0; n_baro = 0; n_imu = 0; n_ev = 0;

    i = 1;
    while i + 2 <= n
        if raw(i) ~= SYNC, i = i + 1; continue; end
        pid  = double(raw(i+1));
        plen = double(raw(i+2));
        if ~isKey(expected, pid) || plen ~= expected(pid)
            i = i + 1; continue;
        end
        if i + plen - 1 > n, break; end
        pkt = raw(i : i + plen - 1);
        if pid == ID_BARO
            t = double(typecast(uint8(pkt(4:7)), 'uint32'));
            v = double(typecast(uint8(pkt(8:11)),'single'));
            n_baro = n_baro + 1;
            baro(n_baro,:) = [t, v];
        elseif pid == ID_IMU
            t  = double(typecast(uint8(pkt(4:7)),  'uint32'));
            iv = double(typecast(uint8(pkt(8:19)), 'int16'));
            n_imu = n_imu + 1;
            imu_pkts(n_imu,:) = [t, iv(:).'];   % force row
        elseif pid == ID_STATE
            t = double(typecast(uint8(pkt(4:7)),   'uint32'));
            p = double(typecast(uint8(pkt(8:19)),  'single'));
            v = double(typecast(uint8(pkt(20:31)), 'single'));
            q = double(typecast(uint8(pkt(32:47)), 'single'));
            n_nav = n_nav + 1;
            nav(n_nav,:) = [t, p(:).', v(:).', q(:).'];
        elseif pid == ID_EVENT
            t  = double(typecast(uint8(pkt(4:7)), 'uint32'));
            ph = double(pkt(8));
            ev = double(pkt(9));
            n_ev = n_ev + 1;
            events(n_ev,:) = [t, ph, ev];
        end
        i = i + plen;
    end

    % Trim
    nav      = nav(1:n_nav,      :);
    baro     = baro(1:n_baro,    :);
    imu_pkts = imu_pkts(1:n_imu, :);
    events   = events(1:n_ev,    :);

    if ~isempty(nav),      nav(:,1)      = (nav(:,1)      - nav(1,1))      * 1e-6; end
    if ~isempty(baro),     baro(:,1)     = (baro(:,1)     - baro(1,1))     * 1e-6; end
    if ~isempty(imu_pkts), imu_pkts(:,1) = (imu_pkts(:,1) - imu_pkts(1,1)) * 1e-6; end
    if ~isempty(events),   events(:,1)   = (events(:,1)   - events(1,1))   * 1e-6; end
end


function write_nav_csv(nav, path)
    if isempty(nav), fprintf('   (no STATE — skip nav CSV)\n'); return; end
    T = array2table(nav, 'VariableNames', ...
        {'t','pN','pE','pD','vN','vE','vD','qw','qx','qy','qz'});
    writetable(T, path);
    fprintf('   wrote %s (%d rows)\n', path, height(T));
end

function write_event_csv(events, path)
    if isempty(events), return; end
    phase_names = {'PRE_FLIGHT','POWERED_FLIGHT','COASTING','DESCENT','LANDED'};
    event_names = {'LAUNCH','BURNOUT','APOGEE','LAND','NSC'};
    pn = cell(size(events,1),1);
    en = cell(size(events,1),1);
    for k = 1:size(events,1)
        p = events(k,2); e = events(k,3);
        pn{k} = ternary(p>=0 && p<numel(phase_names), phase_names{p+1}, '?');
        en{k} = ternary(e>=1 && e<=numel(event_names), event_names{e}, '?');
    end
    T = table(events(:,1), events(:,2), events(:,3), pn, en, ...
              'VariableNames', {'t','phase','event_id','phase_name','event_name'});
    writetable(T, path);
    fprintf('   wrote %s (%d rows)\n', path, height(T));
end

function s = ternary(cond, a, b), if cond, s = a; else, s = b; end, end


function print_events(events)
    if isempty(events), return; end
    phase_names = {'PRE_FLIGHT','POWERED_FLIGHT','COASTING','DESCENT','LANDED'};
    event_names = {'LAUNCH','BURNOUT','APOGEE','LAND','NSC'};
    fprintf('\n  === FSM events (firmware) ===\n');
    fprintf('  %8s  %-15s  %s\n', 't [s]', 'phase', 'event');
    for k = 1:size(events,1)
        p = events(k,2); e = events(k,3);
        pn = ternary(p>=0 && p<numel(phase_names), phase_names{p+1}, '?');
        en = ternary(e>=1 && e<=numel(event_names), event_names{e}, '?');
        fprintf('  %8.3f  %-15s  %s (%d)\n', events(k,1), pn, en, e);
    end
end


function compare(nav, truth)
    if isempty(nav), fprintf('   (no STATE — skip compare)\n'); return; end
    t       = nav(:,1);
    alt_kf  = -nav(:,4);
    vvel_kf = -nav(:,7);
    qw=nav(:,8); qx=nav(:,9); qy=nav(:,10); qz=nav(:,11);
    cos_tilt_kf = 2*(qw.*qy - qx.*qz);

    alt_t  = interp1(truth.t, truth.alt,  t, 'linear', 'extrap');
    vvel_t = interp1(truth.t, truth.vvel, t, 'linear', 'extrap');
    cos_t  = sin(deg2rad(interp1(truth.t, truth.zenith, t, 'linear','extrap')));

    err_a = alt_kf  - alt_t;
    err_v = vvel_kf - vvel_t;
    err_c = cos_tilt_kf - cos_t;

    fprintf('\n  === Accuracy vs OpenRocket ===\n');
    fprintf('   Altitude  RMSE: %7.2f m   (max %.2f m)\n',     sqrt(mean(err_a.^2)), max(abs(err_a)));
    fprintf('   Vert vel  RMSE: %7.2f m/s (max %.2f m/s)\n',   sqrt(mean(err_v.^2)), max(abs(err_v)));
    fprintf('   cos(tilt) RMSE: %7.3f\n',                       sqrt(mean(err_c.^2)));

    T = table(t, alt_kf, alt_t, err_a, vvel_kf, vvel_t, err_v, cos_tilt_kf, cos_t, err_c, ...
              'VariableNames', {'t','alt_kf','alt_truth','err_alt', ...
                                'vvel_kf','vvel_truth','err_vvel', ...
                                'cos_tilt_kf','cos_tilt_truth','err_cos_tilt'});
    writetable(T, 'flight_compare.csv');
    fprintf('   wrote flight_compare.csv\n');
end


function plot_results(nav, truth, events)
    if isempty(nav), return; end
    t       = nav(:,1);
    alt_kf  = -nav(:,4);
    vvel_kf = -nav(:,7);
    qw=nav(:,8); qx=nav(:,9); qy=nav(:,10); qz=nav(:,11);
    cos_tilt_kf = 2*(qw.*qy - qx.*qz);

    figure('Name','HIL: Altitude','Position',[80 80 1100 350]);
    subplot(1,2,1); hold on; grid on;
    plot(truth.t, truth.alt, 'k--', 'LineWidth',1.0, 'DisplayName','OpenRocket');
    plot(t, alt_kf, 'b', 'LineWidth',1.2, 'DisplayName','KF');
    overlay_events(events);
    xlabel('Time [s]'); ylabel('Altitude [m]'); title('Altitude'); legend('Location','best');
    subplot(1,2,2); hold on; grid on;
    plot(t, alt_kf - interp1(truth.t, truth.alt, t, 'linear', 'extrap'), 'r','LineWidth',1.1);
    overlay_events(events);
    xlabel('Time [s]'); ylabel('Error [m]'); title('Altitude error');

    figure('Name','HIL: Vertical velocity','Position',[80 460 1100 350]);
    subplot(1,2,1); hold on; grid on;
    plot(truth.t, truth.vvel, 'k--', 'LineWidth',1.0, 'DisplayName','OpenRocket');
    plot(t, vvel_kf, 'b', 'LineWidth',1.2, 'DisplayName','KF');
    overlay_events(events);
    xlabel('Time [s]'); ylabel('v_{up} [m/s]'); title('Vertical velocity'); legend('Location','best');
    subplot(1,2,2); hold on; grid on;
    plot(t, vvel_kf - interp1(truth.t, truth.vvel, t, 'linear', 'extrap'), 'r','LineWidth',1.1);
    overlay_events(events);
    xlabel('Time [s]'); ylabel('Error [m/s]'); title('Vertical-velocity error');

    figure('Name','HIL: Tilt','Position',[600 80 700 350]);
    hold on; grid on;
    plot(truth.t, sin(deg2rad(truth.zenith)), 'k--','LineWidth',1.0,'DisplayName','sin(zenith)');
    plot(t, cos_tilt_kf, 'b','LineWidth',1.2,'DisplayName','KF cos(tilt)');
    overlay_events(events);
    xlabel('Time [s]'); ylabel('cos(tilt)');
    title('Attitude (1 = vertical)'); legend('Location','best'); ylim([-1.1 1.1]);

    figure('Name','HIL: Quaternion','Position',[80 80 1100 600]);
    qlbl = {'w','x','y','z'};
    for i = 1:4
        subplot(2,2,i); hold on; grid on;
        plot(t, nav(:, 7+i), 'b','LineWidth',1.1);
        overlay_events(events);
        xlabel('Time [s]'); ylabel(sprintf('q_%s',qlbl{i}));
        title(sprintf('Quaternion %s',qlbl{i}));
    end
    sgtitle('Quaternion (firmware)');

    figure('Name','HIL: NED Position','Position',[100 100 1200 350]);
    lbl = {'N','E','D'};
    for i = 1:3
        subplot(1,3,i); hold on; grid on;
        plot(t, nav(:, 1+i), 'b','LineWidth',1.1);
        overlay_events(events);
        xlabel('Time [s]'); ylabel(sprintf('p_%s [m]',lbl{i}));
        title(sprintf('Position %s',lbl{i}));
    end
    sgtitle('Position NED (NE dead-reckoned)');
end


function overlay_events(events)
    if isempty(events), return; end
    event_names = {'LAUNCH','BURNOUT','APOGEE','LAND','NSC'};
    colors = {'g','m','r','c','y'};
    yl = ylim;
    for k = 1:size(events,1)
        e = events(k,3);
        if e < 1 || e > 5, continue; end
        line([events(k,1) events(k,1)], yl, ...
             'Color', colors{e}, 'LineStyle', ':', 'LineWidth', 0.8, ...
             'HandleVisibility','off');
        text(events(k,1), yl(2)*0.95, event_names{e}, ...
             'Color', colors{e}, 'FontSize', 8, 'Rotation', 90, ...
             'VerticalAlignment','bottom', 'HandleVisibility','off');
    end
end
