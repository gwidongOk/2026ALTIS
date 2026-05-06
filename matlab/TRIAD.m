function q = TRIAD(acc, mag, lat, lon)
    acc = acc(:);
    mag = mag(:);
    
    %% 1. NED 기준벡터 (진북 기준)
    r1 = [0; 0; 1];                            % Down (gravity 방향)

    if nargin >= 4 && ~isempty(lat) && ~isempty(lon)
        % GPS lat/lon으로 WMM-Korea 선형근사 → 단위벡터 m_ref
        [D_deg, I_deg] = wmm_korea(lat, lon);
        cD = cosd(D_deg); sD = sind(D_deg);
        cI = cosd(I_deg); sI = sind(I_deg);
        r2 = [cI*cD; cI*sD; sI];
    else
        % SensorSpec 디폴트 (서울 기준)
        S  = SensorSpec;
        r2 = S.m_ref_ned / norm(S.m_ref_ned);
    end

    %% 2. body 관측벡터
    %  정지 상태: acc_body = R_n2b * (-g_ned)  →  -acc 방향 = Down을 body에서 본 것
    b1 = -acc / norm(acc);                     % Down (in body)
    b2 =  mag / norm(mag);                     % 지자기 (in body)

    %  평행성 검사 (Korea: r1·r2 ≈ 0.78~0.82, 안전)
    if abs(dot(b1, b2)) > 0.999
        warning('TRIAD: 측정 acc/mag 벡터가 거의 평행 — 자세 추정 불안정.');
    end

    %% 3. 직교 triad 구성
    % NED frame
    u1 = r1;
    u2 = cross(u1, r2) / norm(cross(u1, r2));
    u3 = cross(u1, u2);
    M_ref = [u1, u2, u3];

    % body frame
    v1 = b1;
    v2 = cross(v1, b2) / norm(cross(v1, b2));
    v3 = cross(v1, v2);
    M_body = [v1, v2, v3];

    %% 4. R_b2n = M_ref * M_body'   →  쿼터니언
    %  유도: M_body = R_n2b * M_ref  ⇒  R_n2b = M_body * M_ref'
    %        R_b2n = R_n2b' = M_ref * M_body'
    C_bn = M_ref * M_body';
    q    = NavUtils.dcm2quat(C_bn);            % [4x1] column
end


%% ─────────────────────────────────────────────────────────────────
function [D, I] = wmm_korea(lat, lon)
%WMM_KOREA  한국 영역 자기 편각/복각 선형 근사 (WMM-2025 기준, 2026년)
%
%   유효 범위: 33°N ~ 39°N, 125°E ~ 131°E
%   정확도   : Declination ±0.3°,  Inclination ±0.5°
%
%   Center : (36°N, 127°E)에서 D = -7.9°, I = 51.0°
%   기울기 : ∂D/∂lat ≈ -0.35,  ∂D/∂lon ≈ +0.15
%            ∂I/∂lat ≈ +1.4,   ∂I/∂lon ≈ +0.05

    if lat < 32 || lat > 40 || lon < 124 || lon > 132
        warning('TRIAD/wmm_korea: 위/경도 (%.2f, %.2f)가 한국 영역 밖 — 정확도 저하 가능', lat, lon);
    end

    D = -7.9 - 0.35*(lat - 36) + 0.15*(lon - 127);   % declination [deg]
    I = 51.0 + 1.40*(lat - 36) + 0.05*(lon - 127);   % inclination [deg]
end
