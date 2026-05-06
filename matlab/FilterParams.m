classdef FilterParams
%FILTERPARAMS  필터 행렬 (P, Q, R)

    properties
        P      (15,15) double = zeros(15)    % 오차 공분산
        R_gps  (6,6)   double = zeros(6)     % GPS 측정 노이즈
        R_baro (1,1)   double = 0            % 기압 측정 노이즈
        R_mag  (3,3)   double = zeros(3)     % 지자기 측정 노이즈 (3축 단위벡터)
        % 프로세스 노이즈 파라미터 (predict에서 Qd 계산용)
        var_acc  (1,1) double = 0
        var_gyro (1,1) double = 0
        var_ba   (1,1) double = 0
        var_bg   (1,1) double = 0
    end

end
