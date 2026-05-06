classdef ESEKF < handle
%ESEKF  Error-State Extended Kalman Filter (ES-EKF)
%
% 프로퍼티:
%   nom   NominalState  — 명목 상태 (p, v, q, b_a, b_g)
%   par   FilterParams  — 필터 행렬 (P, Q, R_gps, R_baro)
%   g     [3x1]         — 중력벡터 NED [m/s²]
%
% 사용법:
%   % 1. 초기 자세 결정 (TRIAD)
%   q0 = ESEKF.run_triad(acc_init, mag_init, lat, lon);
%
%   % 2. 필터 초기화
%   ekf = ESEKF(p0, v0, q0);
%
%   % 3. 예측 및 업데이트
%   ekf.predict(a_m, w_m, dt);
%   ekf.update_gps(z_gps);
%   ekf.update_baro(z_baro);
%   ekf.update_zupt();

    properties
        nom   NominalState     % 명목 상태 x
        par   FilterParams     % 필터 행렬 (P, Q, R)
        g     (3,1) double     % 중력벡터 NED [m/s²]
        last_accel_mag (1,1) double = 0  % 최근 predict의 bias 보정된 specific force 크기 [m/s²]
    end

    methods (Access = public)

        %% ── 생성자 ───────────────────────────────────────────────────────
        function obj = ESEKF(p0, v0, q0)
        %   p0 [3x1] 초기 위치 NED [m]
        %   v0 [3x1] 초기 속도 NED [m/s]
        %   q0 [4x1] 초기 쿼터니언 [q0;q1;q2;q3]

            % ── 명목 상태 ─────────────────────────────────────────────
            obj.nom = NominalState();
            obj.nom.p = p0;
            obj.nom.v = v0;
            obj.nom.q = q0;

            % ── 물리 상수 (SensorSpec 참조) ──────────────────────────
            S = SensorSpec;
            obj.g     = S.g_ned;

            % ── 필터 행렬 ─────────────────────────────────────────────
            obj.par = FilterParams();

            % P0 — 초기 오차 공분산
            P0_pos = 3;        % [m]
            P0_vel = 1;        % [m/s]
            P0_att = 0.087;    % [rad] ≈ 5°
            P0_ba  = 0.5;      % [m/s²]
            P0_bg  = 0.01;     % [rad/s]
            obj.par.P = diag([P0_pos^2 * ones(1,3), ...
                              P0_vel^2 * ones(1,3), ...
                              P0_att^2 * ones(1,3), ...
                              P0_ba^2  * ones(1,3), ...
                              P0_bg^2  * ones(1,3)]);

            % Q — 프로세스 노이즈 (연속시간 파라미터 저장, predict에서 이산화)
            % var_acc/var_gyro: 샘플 분산 [단위²·s⁻¹]
            % var_ba/var_bg:    랜덤워크 분산/dt [단위²·s⁻¹]
            obj.par.var_acc  = S.var_acc;
            obj.par.var_gyro = S.var_gyro;
            obj.par.var_ba   = S.var_ba;
            obj.par.var_bg   = S.var_bg;

            % R — 측정 노이즈
            obj.par.R_gps  = diag([S.var_gps_pos_h, S.var_gps_pos_h, ...
                                   S.var_gps_pos_v, ...
                                   S.var_gps_vel_h, S.var_gps_vel_h, ...
                                   S.var_gps_vel_v]);
            obj.par.R_baro = S.var_baro;
        end

        %% ── predict ──────────────────────────────────────────────────────
        function predict(obj, a_m, w_m, dt)
        %   a_m [3x1] 가속도계 측정값 [m/s²]
        %   w_m [3x1] 자이로 측정값 [rad/s]
        %   dt  [1x1] IMU 샘플 간격 [s]

            a_hat = a_m - obj.nom.b_a;
            w_hat = w_m - obj.nom.b_g;

            % bias 보정된 specific force 크기 저장 (GPS high-G 페널티에서 사용)
            obj.last_accel_mag = norm(a_hat);

            % 명목 상태 전파
            R_nb  = NavUtils.quat2dcm(obj.nom.q);
            a_ned = R_nb * a_hat + obj.g;

            obj.nom.p = obj.nom.p + obj.nom.v*dt + 0.5*a_ned*dt^2;
            obj.nom.v = obj.nom.v + a_ned*dt;

            theta = w_hat * dt;
            th_n  = norm(theta);
            if th_n > 1e-10
                dq = [cos(th_n/2); sin(th_n/2)/th_n * theta];
            else
                dq = [1; theta/2];
            end
            obj.nom.q = NavUtils.quat_mult(obj.nom.q, dq);
            obj.nom.q = obj.nom.q / norm(obj.nom.q);

            % 오차 공분산 전파
            Fc = zeros(15);
            Fc(1:3,  4:6)  = eye(3);
            Fc(4:6,  7:9)  = -R_nb * NavUtils.skew(a_hat);
            Fc(4:6,  10:12)= -R_nb;
            Fc(7:9,  7:9)  = -NavUtils.skew(w_hat);
            Fc(7:9,  13:15)= -eye(3);
            %Fc = 연속시간 시스템 행렬 (미분 방정식의 계수) F  = e^(Fc·dt) = 이산시간 상태천이행렬
            Fdt       = Fc * dt;
            F       = eye(15) + Fdt + Fdt^2/2 + Fdt^3/6; % 테일러 급수 3차

            % 이산 프로세스 노이즈: Qd = G·Qc·G'·dt
            %   속도 ← n_a·dt  →  Qd(v) = var_acc · dt²
            %   자세 ← n_g·dt  →  Qd(θ) = var_gyro · dt²
            %   바이어스 ← 랜덤워크 →  Qd(b) = var_b  (이미 per-step)
            Qd = zeros(15);
            Qd(4:6,   4:6)   = obj.par.var_acc  * dt * eye(3);
            Qd(7:9,   7:9)   = obj.par.var_gyro * dt * eye(3);
            Qd(10:12, 10:12) = obj.par.var_ba * dt * eye(3);
            Qd(13:15, 13:15) = obj.par.var_bg * dt * eye(3);

            obj.par.P = F * obj.par.P * F' + Qd;
            obj.par.P = 0.5*(obj.par.P + obj.par.P');
        end

        %% ── update_gps ───────────────────────────────────────────────────
        function update_gps(obj, z, hAcc, vAcc)
        %   z         [6x1] = [pN;pE;pD;vN;vE;vD]
        %   hAcc      [scalar] (선택) GPS 수평 정확도 [m]
        %   vAcc      [scalar] (선택) GPS 수직 정확도 [m]
        %   hAcc/vAcc 미지정 시 정적 R_gps 사용 (기존 동작 유지)
        %   가속도 크기는 obj.last_accel_mag (predict에서 갱신) 사용 — 4g 초과 시 GPS 신뢰도 감쇠

            H = zeros(6,15);
            H(1:3,1:3) = eye(3);
            H(4:6,4:6) = eye(3);

            y = z - [obj.nom.p; obj.nom.v];

            if nargin >= 4 && ~isempty(hAcc) && ~isempty(vAcc)
                % 1. 기본 오차 가중치 (3배)
                multiplier = 3.0;

                % 2. 가속도 기반 동적 스케일링 (4g 한계 대응)
                G4_LIMIT = 4.0 * 9.80665;
                high_g_penalty = 1.0;
                if obj.last_accel_mag > G4_LIMIT
                    excess = obj.last_accel_mag - G4_LIMIT;
                    high_g_penalty = 1.0 + (excess * excess * 10.0);
                    if high_g_penalty > 1000.0
                        high_g_penalty = 1000.0;
                    end
                end

                final_multiplier = multiplier * high_g_penalty;
                var_h  = (hAcc * final_multiplier)^2;
                var_v  = (vAcc * final_multiplier)^2;
                var_vh = var_h * 0.2;
                var_vv = var_v * 0.2;

                R = diag([var_h, var_h, var_v, var_vh, var_vh, var_vv]);
            else
                R = obj.par.R_gps;
            end

            obj.measurement_update(H, y, R);
        end

        %% ── update_baro ──────────────────────────────────────────────────
        function update_baro(obj, z)
        %   z [scalar] 고도 = -pD [m]

            H      = zeros(1,15);
            H(1,3) = -1;

            y = z - (-obj.nom.p(3));

            obj.measurement_update(H, y, obj.par.R_baro);
        end

        %% ── update_zupt ─────────────────────────────────────────────────
        function update_zupt(obj)
            % 1. 정지 상태이므로 관측된 참 속도는 0
            z_vel = [0; 0; 0];
            
            % 2. 예측된 속도
            v_pred = obj.nom.v;
            
            % 3. 속도 관측 오차 (Innovation: z - 예측값)
            y = z_vel - v_pred;
            
            % 4. 관측 행렬 (H) 생성
            % ESEKF 상태 벡터가 [dp, dv, dth, dba, dbg] 순서인 15차원일 때,
            % 속도 오차(dv)는 4번째부터 6번째 인덱스에 위치합니다.
            H = zeros(3, 15);
            H(1:3, 4:6) = eye(3);
            
            % 5. ZUPT 전용 노이즈 공분산 (R)
            % GPS 속도 노이즈보다 훨씬 작은 값(예: 1e-4)을 부여합니다.
            R_zupt = eye(3) * 1e-4; 
            
            % 6. 공통 측정 업데이트 함수 호출
            obj.measurement_update(H, y, R_zupt);
        end

        %% ── update_acc_static (Inclinometer Update) ─────────────────────
        function update_acc_static(obj, a_m)
            % 정지 상태에서 가속도계가 중력만을 측정해야 함을 이용해 자세와 바이어스 보정
            % a_m: [3x1] 가속도계 측정값 [m/s²]
            
            R_nb = NavUtils.quat2dcm(obj.nom.q);
            % 예측된 비력: R_nb' * (-g_ned) + b_a
            z_pred = R_nb' * (-obj.g) + obj.nom.b_a;
            y = a_m(:) - z_pred;
            
            % 야코비안 H
            % δθ에 대한 미분: R_nb' * skew(g_ned)  또는 skew(R_nb' * -g_ned)
            % δba에 대한 미분: eye(3)
            H = zeros(3, 15);
            H(:, 7:9)   = NavUtils.skew(R_nb' * (-obj.g));
            H(:, 10:12) = eye(3);
            
            % 측정 노이즈 (가속도 분산)
            S = SensorSpec;
            R_acc = eye(3) * (S.var_acc / S.dt_imu); 
            
            obj.measurement_update(H, y, R_acc);
        end

        %% ── update_gyro_static (ZARU: Zero Angular Rate Update) ──────────
        function update_gyro_static(obj, w_m)
            % 정지 상태에서 자이로 측정값이 0(바이어스 제외)이어야 함을 이용해 바이어스 보정
            % w_m: [3x1] 자이로 측정값 [rad/s]
            
            % 예측된 회전율: b_g
            z_pred = obj.nom.b_g;
            y = w_m(:) - z_pred;
            
            % 야코비안 H (δbg에 대한 미분: eye(3))
            H = zeros(3, 15);
            H(:, 13:15) = eye(3);
            
            % 측정 노이즈 (자이로 분산)
            S = SensorSpec;
            R_gyro = eye(3) * (S.var_gyro / S.dt_imu);
            
            obj.measurement_update(H, y, R_gyro);
        end

        %% ── update_mag ──────────────────────────────────────────────────
        function update_mag(obj, z_m)
            % z_m: [3x1] 정규화된 지자기 측정 벡터 (body frame)
            %      또는 [3x1] Gauss 단위 측정값 (내부에서 정규화)
            
            if norm(z_m) < 1e-4, return; end
            z_m = z_m(:) / norm(z_m);
            
            % 1. 예측된 지자기 벡터 (NED -> Body)
            R_nb = NavUtils.quat2dcm(obj.nom.q);
            S = SensorSpec;
            m_ref = S.m_ref_ned / norm(S.m_ref_ned);
            z_pred = R_nb' * m_ref;
            
            % 2. 혁신 (Innovation)
            y = z_m - z_pred;
            
            % 3. 야코비안 H (자세 오차 [7:9]에 대해서만 존재)
            H = zeros(3, 15);
            H(:, 7:9) = NavUtils.skew(z_pred);
            
            % 4. 측정 노이즈 (SensorSpec 참조)
            R_mag = eye(3) * S.var_mag;
            
            % 5. 업데이트
            obj.measurement_update(H, y, R_mag);
        end

        %% ── run_zupt_alignment (자동 수렴 루프) ──────────────────────────
        function [t_spent, converged] = run_zupt_alignment(obj, imu_provider, dt, threshold, max_time, mag_provider)
            arguments
                obj
                imu_provider
                dt (1,1) double
                threshold (1,1) double = 1e-4
                max_time (1,1) double = 30.0
                mag_provider = [] % 선택적 지자기 데이터 공급자
            end

            t = 0;
            converged = false;
            steps_per_update = round(0.1 / dt); % 10Hz 업데이트 주기를 모사

            % 루프 시작
            while t < max_time
                % 1. 0.1초 동안 예측(Predict) 수행
                for i = 1:steps_per_update
                    [a_m, w_m] = imu_provider();
                    obj.predict(a_m, w_m, dt);
                    t = t + dt;
                end
                
                % 2. 정적 업데이트 수행 (P 변화 감지용)
                P_before = sum(diag(obj.par.P(7:15, 7:15))); % 자세, 바이어스 영역 감시
                
                % [필수 업데이트] 속도(0), 가속도(중력), 자이로(0)
                obj.update_zupt();          % 속도 오차 제거
                obj.update_acc_static(a_m); % Pitch/Roll 오차 및 가속도 바이어스 제거
                obj.update_gyro_static(w_m);% 자이로 바이어스 제거
                
                % [선택 업데이트] 지자기
                if ~isempty(mag_provider)
                    z_m = mag_provider();
                    obj.update_mag(z_m);    % Yaw(Heading) 오차 제거
                end
                
                P_after = sum(diag(obj.par.P(7:15, 7:15)));
                
                % 3. 수렴 판단
                if P_before > 0 && abs(P_before - P_after) / P_before < threshold
                    converged = true;
                    break;
                end
            end
            t_spent = t;
        end

    end % methods (Access = public)

    methods (Static)
        %% ── run_triad ──────────────────────────────────────────────────
        function q = run_triad(acc, mag, lat, lon)
        %   TRIAD 알고리즘을 통한 초기 자세 결정
        %   acc [3x1] 초기 정지 상태 가속도 측정값
        %   mag [3x1] 초기 지자기 측정값
        %   lat, lon (선택) 위도/경도 [deg]
        
            acc = acc(:);
            mag = mag(:);
            
            % 1. NED 기준벡터
            r1 = [0; 0; 1]; % Down
            
            if nargin >= 4 && ~isempty(lat) && ~isempty(lon)
                [D_deg, I_deg] = ESEKF.wmm_korea(lat, lon);
                cD = cosd(D_deg); sD = sind(D_deg);
                cI = cosd(I_deg); sI = sind(I_deg);
                r2 = [cI*cD; cI*sD; sI];
            else
                S  = SensorSpec;
                r2 = S.m_ref_ned / norm(S.m_ref_ned);
            end

            % 2. body 관측벡터
            b1 = -acc / norm(acc);
            b2 =  mag / norm(mag);

            if abs(dot(b1, b2)) > 0.999
                warning('ESEKF.run_triad: 측정 벡터가 거의 평행하여 자세 추정이 불안정할 수 있습니다.');
            end

            % 3. 직교 triad 구성
            u1 = r1;
            u2 = cross(u1, r2) / norm(cross(u1, r2));
            u3 = cross(u1, u2);
            M_ref = [u1, u2, u3];

            v1 = b1;
            v2 = cross(v1, b2) / norm(cross(v1, b2));
            v3 = cross(v1, v2);
            M_body = [v1, v2, v3];

            % 4. R_b2n = M_ref * M_body'
            C_bn = M_ref * M_body';
            q    = NavUtils.dcm2quat(C_bn);
        end
    end

    %% ── 내부 공통 함수 ──────────────────────────────────────────────────
    methods (Access = private)

        function measurement_update(obj, H, y, R)

            S  = H * obj.par.P * H' + R;
            K  = obj.par.P * H' / S;
            dx = K * y;

            % P 업데이트 (Joseph form)
            IKH       = eye(15) - K*H;
            obj.par.P = IKH * obj.par.P * IKH' + K * R * K';
            obj.par.P = 0.5*(obj.par.P + obj.par.P');

            % 오차 dx를 명목 상태에 주입
            obj.nom.p   = obj.nom.p   + dx(1:3);
            obj.nom.v   = obj.nom.v   + dx(4:6);
            obj.nom.b_a = obj.nom.b_a + dx(10:12);
            obj.nom.b_g = obj.nom.b_g + dx(13:15);

            dth   = dx(7:9);
            dth_n = norm(dth);
            if dth_n > 1e-10
                dq = [cos(dth_n/2); sin(dth_n/2)/dth_n * dth];
            else
                dq = [1; dth/2];
            end
            obj.nom.q = NavUtils.quat_mult(obj.nom.q, dq/norm(dq));
            obj.nom.q = obj.nom.q / norm(obj.nom.q);

            % 리셋 야코비안
            %른 상태변수는 NED 기준좌표계 값이기에 오차공분산을 보정할 필요가 없지만 
            % 자세는 body축과 ned의 관계이기에 P의 좌표계를 보정된 새 자세로 바꿀 필요 있음.
            % δθ가 관여하는 행 전체와 열 전체가 변화함
            G          = eye(15);
            G(7:9,7:9) = eye(3) - NavUtils.skew(dth/2);
            obj.par.P  = G * obj.par.P * G';
            obj.par.P  = 0.5*(obj.par.P + obj.par.P');
        end

    end % methods (Access = private)
    
    methods (Static, Access = private)
        function [D, I] = wmm_korea(lat, lon)
        %   한국 영역 자기 편각/복각 선형 근사
            if lat < 32 || lat > 40 || lon < 124 || lon > 132
                warning('ESEKF.wmm_korea: 한국 영역 밖 — 정확도 저하 가능');
            end
            D = -7.9 - 0.35*(lat - 36) + 0.15*(lon - 127);
            I = 51.0 + 1.40*(lat - 36) + 0.05*(lon - 127);
        end
    end

end % classdef
