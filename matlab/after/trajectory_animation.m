%% trajectory_animation.m
% =========================================================================
% STATE / IMU / BARO 패킷을 엑셀에서 읽어
%   (1) 전체 비행 궤적을 정적으로 표시
%   (2) KF 성능 비교
%         고도 : KF  vs  Baro 원본       vs  Acc 2중 적분
%         속도 : KF  vs  Baro 수치 미분  vs  Acc 적분
%   (3) 시간에 따라 동체 좌표축(R/G/B = body X/Y/Z) 이 회전하며
%       궤적을 따라가는 애니메이션을 표시
%
% 입력 파일
%   SW/parse/main.py 의 "Save XLSX" 로 저장한 .xlsx
%   시트 : state_pkt  (필수, 트랙/애니메이션/KF 비교 모두 사용)
%          baro_pkt   (KF 비교 시 필요)
%          imu_pkt    (KF 비교 시 필요)
%
% 좌표계
%   비행 데이터        : NED  (N-X, E-Y, D-Z, 하향이 양)
%   화면 표시          : ENU  (E-X, N-Y, U-Z, 상향이 양)   ← 가독성 위해 변환
%   쿼터니언 [w x y z] : Hamilton 규약,  R(q) = R_nb  (body → NED)
%   동체 축 의미       : body-X = 로켓 종축(노즈 방향), Y/Z = 반경 방향
%
% 사용 절차
%   1) SW/parse 도구로 비행 종료 후  PARSE → Save XLSX  로 .xlsx 저장
%   2) 본 스크립트 상단의 xlsxFile 변수에 해당 파일 경로 지정
%   3) F5 또는 Run 으로 실행
% =========================================================================

clear; clc; close all;

% ───────── 사용자 설정 ─────────
xlsxFile      = fullfile(fileparts(mfilename('fullpath')), 'flight.xlsx');
sheetName     = 'state_pkt';
axisLen       = 5;        % 동체 좌표축 길이 [m]
playbackSpeed = 5;        % 1.0 = 실시간, 5.0 = 5배속
targetFps     = 30;       % 애니메이션 목표 프레임률
showTrail     = true;     % 지나온 경로를 잔상으로 표시
saveVideo     = false;    % true → .mp4 저장
videoFile     = fullfile(fileparts(mfilename('fullpath')), 'trajectory.mp4');

showKfCompare = true;     % KF 성능 비교 그래프 표시 여부
G_MPS2        = 9.80665;  % 중력 가속도 [m/s²]
% LSM6DSO32 ±32 g 레인지 감도 : 0.976 mg / LSB  →  m/s²
ACC_LSB2MPS2  = 0.976 * 1e-3 * G_MPS2;

% ───────── 데이터 로드 ─────────
if ~isfile(xlsxFile)
    error(['엑셀 파일을 찾을 수 없습니다.\n  경로 : %s\n', ...
           '스크립트 상단의 xlsxFile 변수를 실제 파일 경로로 수정하세요.'], ...
          xlsxFile);
end

sheets = sheetnames(xlsxFile);
if ~ismember(sheetName, sheets)
    error(['엑셀 파일에 "%s" 시트가 없습니다.\n', ...
           '  존재 시트 : %s\n', ...
           '  비행 중 KF 가 준비되지 않았다면 STATE 패킷이 기록되지 않습니다.'], ...
          sheetName, strjoin(string(sheets), ', '));
end

T = readtable(xlsxFile, 'Sheet', sheetName);
N = height(T);
if N < 2
    error('STATE 샘플 수가 부족합니다 (N = %d). 비행 로그를 확인하세요.', N);
end
fprintf('STATE 샘플 %d 개 로드 (%s / %s)\n', N, xlsxFile, sheetName);

t  = double(T.t - T.t(1)) / 1e6;            % [s] 시작 0 기준
pN = T.pN; pE = T.pE; pD = T.pD;            % NED 위치 [m]
qw = T.qw; qx = T.qx; qy = T.qy; qz = T.qz;

% 표시용 ENU 위치
xE = pE;  yN = pN;  zU = -pD;

% 쿼터니언 정규화
qn = sqrt(qw.^2 + qx.^2 + qy.^2 + qz.^2);
qw = qw./qn; qx = qx./qn; qy = qy./qn; qz = qz./qn;

% ───────── (1) 전체 궤적 정적 표시 ─────────
[apogee, kApg] = max(zU);

figure('Color','w','Name','전체 궤적','Position',[60 60 900 720]);
plot3(xE, yN, zU, 'b-', 'LineWidth', 1.4); hold on;
plot3(xE(1),    yN(1),    zU(1),    'go', 'MarkerSize',10,'MarkerFaceColor','g');
plot3(xE(end),  yN(end),  zU(end),  'rs', 'MarkerSize',10,'MarkerFaceColor','r');
plot3(xE(kApg), yN(kApg), apogee,   'm^', 'MarkerSize',10,'MarkerFaceColor','m');
grid on; axis equal; view(40, 25);
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
title(sprintf(['전체 비행 궤적   |   최고 고도 %.1f m   |   ', ...
               '비행 시간 %.1f s'], apogee, t(end)));
legend({'궤적','발사 지점','마지막 샘플','정점'}, 'Location','best');

% ───────── (2) KF 성능 비교 (고도 / 속도) ─────────
if showKfCompare
    needSheets = {'imu_pkt','baro_pkt'};
    missing    = needSheets(~ismember(needSheets, sheets));
    if ~isempty(missing)
        warning(['KF 비교 그래프를 건너뜁니다. 누락 시트 : %s\n', ...
                 '(엑셀에 imu_pkt, baro_pkt 시트가 모두 있어야 합니다.)'], ...
                strjoin(missing, ', '));
    else
        Tb  = readtable(xlsxFile, 'Sheet', 'baro_pkt');
        Ti  = readtable(xlsxFile, 'Sheet', 'imu_pkt');

        % 공통 t0 : 세 시트 중 가장 이른 타임스탬프
        t0_us       = double(min([T.t(1), Tb.t(1), Ti.t(1)]));
        t_state_abs = (double(T.t)  - t0_us) / 1e6;
        t_baro_abs  = (double(Tb.t) - t0_us) / 1e6;
        t_imu_abs   = (double(Ti.t) - t0_us) / 1e6;

        % KF 결과 (상향이 양)
        alt_kf = -T.pD;
        vel_kf = -T.vD;

        % BARO
        alt_baro     = Tb.alt;
        dt_baro      = diff(t_baro_abs);
        v_baro_diff  = diff(alt_baro) ./ dt_baro;
        t_baro_v     = (t_baro_abs(1:end-1) + t_baro_abs(2:end)) / 2;

        % IMU body acc → m/s²
        ax_b = double(Ti.ax) * ACC_LSB2MPS2;
        ay_b = double(Ti.ay) * ACC_LSB2MPS2;
        az_b = double(Ti.az) * ACC_LSB2MPS2;

        % 쿼터니언을 IMU 시간축으로 선형 보간 후 재정규화
        qw_i = interp1(t_state_abs, qw, t_imu_abs, 'linear', 'extrap');
        qx_i = interp1(t_state_abs, qx, t_imu_abs, 'linear', 'extrap');
        qy_i = interp1(t_state_abs, qy, t_imu_abs, 'linear', 'extrap');
        qz_i = interp1(t_state_abs, qz, t_imu_abs, 'linear', 'extrap');
        qni  = sqrt(qw_i.^2 + qx_i.^2 + qy_i.^2 + qz_i.^2);
        qw_i = qw_i./qni; qx_i = qx_i./qni; qy_i = qy_i./qni; qz_i = qz_i./qni;

        % a_up = -[R_nb(3,:) · f_body] - g    (NED 기준 → 상향이 양)
        %   R(3,1) = 2(qx·qz − qy·qw)
        %   R(3,2) = 2(qy·qz + qx·qw)
        %   R(3,3) = 1 − 2(qx² + qy²)
        R31  = 2*(qx_i.*qz_i - qy_i.*qw_i);
        R32  = 2*(qy_i.*qz_i + qx_i.*qw_i);
        R33  = 1 - 2*(qx_i.^2 + qy_i.^2);
        a_up = -(R31.*ax_b + R32.*ay_b + R33.*az_b) - G_MPS2;

        % cumtrapz 로 적분 (불균등 시간 간격 안전)
        v_acc_int     = cumtrapz(t_imu_abs, a_up);          % acc 적분    [m/s]
        alt_acc_intint= cumtrapz(t_imu_abs, v_acc_int);     % acc 2중적분 [m]

        % 그래프
        figKf = figure('Color','w','Name','KF 성능 비교', ...
                       'Position',[80 80 1100 800]);
        tl = tiledlayout(figKf, 2, 1, 'Padding','compact', 'TileSpacing','compact');
        title(tl, sprintf('KF 성능 비교   (파일 : %s)', xlsxFile), ...
              'Interpreter','none', 'FontWeight','bold');

        % --- 고도 ---
        ax1 = nexttile;
        plot(ax1, t_state_abs, alt_kf,         'b-', 'LineWidth', 2.0); hold(ax1,'on');
        plot(ax1, t_baro_abs,  alt_baro,       'Color',[0 0.55 0], 'LineWidth', 1.0);
        plot(ax1, t_imu_abs,   alt_acc_intint, 'r-', 'LineWidth', 0.8);
        grid(ax1,'on');
        xlabel(ax1,'time [s]'); ylabel(ax1,'고도 (상향) [m]');
        title(ax1,'고도 비교');
        legend(ax1, {'KF (state\_pkt)', 'Baro 원본', 'Acc 2중 적분'}, ...
               'Location','best');

        % --- 속도 ---
        ax2 = nexttile;
        plot(ax2, t_state_abs, vel_kf,      'b-', 'LineWidth', 2.0); hold(ax2,'on');
        plot(ax2, t_baro_v,    v_baro_diff, 'Color',[0 0.55 0], 'LineWidth', 0.6);
        plot(ax2, t_imu_abs,   v_acc_int,   'r-', 'LineWidth', 0.8);
        grid(ax2,'on');
        xlabel(ax2,'time [s]'); ylabel(ax2,'속도 (상향) [m/s]');
        title(ax2,'속도 비교');
        legend(ax2, {'KF (state\_pkt)', 'Baro 수치 미분', 'Acc 적분'}, ...
               'Location','best');

        linkaxes([ax1 ax2], 'x');

        fprintf(['KF 비교 그래프 생성 완료\n', ...
                 '  IMU 샘플  : %d 개  (≈ %.0f Hz)\n', ...
                 '  Baro 샘플 : %d 개  (≈ %.0f Hz)\n', ...
                 '  비행 시간 : %.1f s\n'], ...
                height(Ti), 1/mean(diff(t_imu_abs)), ...
                height(Tb), 1/mean(diff(t_baro_abs)), ...
                t_state_abs(end));
    end
end

% ───────── (3) 자세 + 궤적 애니메이션 ─────────
dt_data = mean(diff(t));
if ~isfinite(dt_data) || dt_data <= 0
    dt_data = 1/100;
end
dataFps  = 1 / dt_data;
stride   = max(1, round(dataFps * playbackSpeed / targetFps));
pauseDur = 1 / targetFps;

figAnim = figure('Color','w','Name','자세+궤적 애니메이션','Position',[100 60 1000 800]);
axAnim  = axes('Parent', figAnim);
plot3(axAnim, xE, yN, zU, 'Color',[0.78 0.78 0.78], 'LineWidth',0.8); hold(axAnim,'on');
grid(axAnim,'on'); axis(axAnim,'equal');

xRng = max(xE)-min(xE);  yRng = max(yN)-min(yN);  zRng = max(zU)-min(zU);
pad  = max(axisLen*1.2, 0.05*max([xRng, yRng, zRng, eps]));
xlim(axAnim, [min(xE)-pad, max(xE)+pad]);
ylim(axAnim, [min(yN)-pad, max(yN)+pad]);
zlim(axAnim, [min(zU)-pad, max(zU)+pad]);
xlabel(axAnim,'East [m]'); ylabel(axAnim,'North [m]'); zlabel(axAnim,'Up [m]');
view(axAnim, 40, 25);
title(axAnim, sprintf('자세 + 궤적 애니메이션   (%.1f배속, %.0f FPS)', ...
                       playbackSpeed, targetFps));

hTrail = plot3(axAnim, NaN,NaN,NaN, 'b-',  'LineWidth', 1.6);
hPos   = plot3(axAnim, NaN,NaN,NaN, 'ko', 'MarkerFaceColor','y','MarkerSize',8);

hAxX = quiver3(axAnim, 0,0,0, 0,0,0, 'AutoScale','off', 'Color','r',        'LineWidth',2.5, 'MaxHeadSize',0.7);
hAxY = quiver3(axAnim, 0,0,0, 0,0,0, 'AutoScale','off', 'Color',[0 .65 0],  'LineWidth',2.5, 'MaxHeadSize',0.7);
hAxZ = quiver3(axAnim, 0,0,0, 0,0,0, 'AutoScale','off', 'Color','b',        'LineWidth',2.5, 'MaxHeadSize',0.7);

% 정보 HUD (고정 위치)
hTxt = annotation(figAnim, 'textbox', [0.02 0.85 0.32 0.12], ...
                  'String','', 'BackgroundColor','w', 'EdgeColor',[0.5 0.5 0.5], ...
                  'FontSize',11, 'FontName','FixedWidth', 'FitBoxToText','off', ...
                  'VerticalAlignment','top', 'Margin',4);

% 비디오 저장 준비
vw = [];
if saveVideo
    vw = VideoWriter(videoFile, 'MPEG-4');
    vw.FrameRate = targetFps;
    open(vw);
end

% 메인 루프
frameNo = 0;
tic;
for k = 1:stride:N
    frameNo = frameNo + 1;

    R_nb   = quat2rotm_local(qw(k), qx(k), qy(k), qz(k));   % body → NED
    bX_enu = ned2enu(R_nb(:,1));
    bY_enu = ned2enu(R_nb(:,2));
    bZ_enu = ned2enu(R_nb(:,3));

    if showTrail
        set(hTrail,'XData',xE(1:k),'YData',yN(1:k),'ZData',zU(1:k));
    end
    set(hPos,'XData',xE(k),'YData',yN(k),'ZData',zU(k));

    set(hAxX,'XData',xE(k),'YData',yN(k),'ZData',zU(k), ...
             'UData',axisLen*bX_enu(1),'VData',axisLen*bX_enu(2),'WData',axisLen*bX_enu(3));
    set(hAxY,'XData',xE(k),'YData',yN(k),'ZData',zU(k), ...
             'UData',axisLen*bY_enu(1),'VData',axisLen*bY_enu(2),'WData',axisLen*bY_enu(3));
    set(hAxZ,'XData',xE(k),'YData',yN(k),'ZData',zU(k), ...
             'UData',axisLen*bZ_enu(1),'VData',axisLen*bZ_enu(2),'WData',axisLen*bZ_enu(3));

    set(hTxt,'String', sprintf(['t   = %6.2f s\n', ...
                                'Alt = %6.2f m\n', ...
                                'R = body X (노즈)   G = body Y   B = body Z'], ...
                                t(k), zU(k)));

    drawnow limitrate;

    if saveVideo
        writeVideo(vw, getframe(figAnim));
    else
        target  = frameNo * pauseDur;
        elapsed = toc;
        if elapsed < target
            pause(target - elapsed);
        end
    end
end

if saveVideo
    close(vw);
    fprintf('비디오 저장 완료 : %s\n', videoFile);
end

% ───────── 로컬 함수 ─────────
function R = quat2rotm_local(qw, qx, qy, qz)
% Hamilton 쿼터니언 [w x y z] → 회전행렬 R_nb (body → NED)
    R = [1-2*(qy^2+qz^2),   2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw);
         2*(qx*qy + qz*qw), 1-2*(qx^2+qz^2),   2*(qy*qz - qx*qw);
         2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx^2+qy^2)];
end

function v_enu = ned2enu(v_ned)
% NED → ENU 벡터 변환
%   ENU.x = NED.E,  ENU.y = NED.N,  ENU.z = -NED.D
    v_enu = [v_ned(2); v_ned(1); -v_ned(3)];
end
