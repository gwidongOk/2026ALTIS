classdef NominalState
%NOMINALSTATE  명목 상태변수 x
%   C 대응: typedef struct { float p[3]; ... } NominalState;

    properties
        p   (3,1) double = zeros(3,1)    % 위치 NED [m]
        v   (3,1) double = zeros(3,1)    % 속도 NED [m/s]
        q   (4,1) double = [1;0;0;0]    % 쿼터니언 Body→NED [q0;q1;q2;q3]
        b_a (3,1) double = zeros(3,1)    % 가속도 바이어스 [m/s²]
        b_g (3,1) double = zeros(3,1)    % 자이로 바이어스 [rad/s]
    end

end
