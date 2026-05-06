classdef NavUtils
% NavUtils  항법용 유틸리티 함수 모음
%   NavUtils.quat2dcm, NavUtils.dcm2quat, NavUtils.quat_mult, NavUtils.skew

    methods (Static)

        function R = quat2dcm(q)
        % 쿼터니언 [q0;q1;q2;q3] -> 3x3 DCM (Body->NED)
            q = q / norm(q);
            q0=q(1); q1=q(2); q2=q(3); q3=q(4);
            R = [1-2*(q2^2+q3^2),  2*(q1*q2-q0*q3),  2*(q1*q3+q0*q2);
                 2*(q1*q2+q0*q3),  1-2*(q1^2+q3^2),  2*(q2*q3-q0*q1);
                 2*(q1*q3-q0*q2),  2*(q2*q3+q0*q1),  1-2*(q1^2+q2^2)];
        end

        function q = dcm2quat(R)
        % 3x3 DCM -> 쿼터니언 [q0;q1;q2;q3] (Shepperd's method)
            tr = trace(R);
            [~, idx] = max([tr, R(1,1), R(2,2), R(3,3)]);
            switch idx
                case 1
                    q0 = 0.5*sqrt(1+tr);       k = 0.25/q0;
                    q1 = k*(R(3,2)-R(2,3));     q2 = k*(R(1,3)-R(3,1));
                    q3 = k*(R(2,1)-R(1,2));
                case 2
                    q1 = 0.5*sqrt(1+2*R(1,1)-tr); k = 0.25/q1;
                    q0 = k*(R(3,2)-R(2,3));     q2 = k*(R(1,2)+R(2,1));
                    q3 = k*(R(1,3)+R(3,1));
                case 3
                    q2 = 0.5*sqrt(1+2*R(2,2)-tr); k = 0.25/q2;
                    q0 = k*(R(1,3)-R(3,1));     q1 = k*(R(1,2)+R(2,1));
                    q3 = k*(R(2,3)+R(3,2));
                case 4
                    q3 = 0.5*sqrt(1+2*R(3,3)-tr); k = 0.25/q3;
                    q0 = k*(R(2,1)-R(1,2));     q1 = k*(R(1,3)+R(3,1));
                    q2 = k*(R(2,3)+R(3,2));
            end
            q = [q0; q1; q2; q3];
            if q(1) < 0, q = -q; end
            q = q / norm(q);
        end

        function q_out = quat_mult(p, r)
        % 쿼터니언 곱 (Hamilton Product): q_out = p ⊗ r
            p0=p(1); p1=p(2); p2=p(3); p3=p(4);
            Q = [p0,-p1,-p2,-p3;
                 p1, p0,-p3, p2;
                 p2, p3, p0,-p1;
                 p3,-p2, p1, p0];
            q_out = Q * r;
        end

        function S = skew(v)
        % 3x1 벡터 -> 3x3 반대칭 행렬 [v]×
            S = [  0,  -v(3),  v(2);
                 v(3),    0,  -v(1);
                -v(2),  v(1),    0 ];
        end

        function qc = quat_conj(q)
        % 쿼터니언 켤레: [q0; -q1; -q2; -q3]
            qc = [q(1); -q(2); -q(3); -q(4)];
        end
        function euler = quat2euler(q)
            % QUAT2EULER 쿼터니언을 오일러 각으로 변환 (ZYX 순서)
            % 입력: q [4x1] 쿼터니언 [q0; q1; q2; q3] (q0가 스칼라)
            % 출력: euler [3x1] 오일러 각 [Roll; Pitch; Yaw] (단위: rad)
            
            % 정규화 (안전장치)
            q = q / norm(q);
            
            q0 = q(1); % 스칼라
            q1 = q(2); % x
            q2 = q(3); % y
            q3 = q(4); % z
            
            % 1. Roll (x축 회전)
            sinr_cosp = 2 * (q0 * q1 + q2 * q3);
            cosr_cosp = 1 - 2 * (q1^2 + q2^2);
            roll = atan2(sinr_cosp, cosr_cosp);
            
            % 2. Pitch (y축 회전)
            sinp = 2 * (q0 * q2 - q3 * q1);
            % 짐벌락(Gimbal Lock) 방지를 위한 클램핑
            if abs(sinp) >= 1
                pitch = sign(sinp) * pi / 2; % 90도 또는 -90도
            else
                pitch = asin(sinp);
            end
            
            % 3. Yaw (z축 회전)
            siny_cosp = 2 * (q0 * q3 + q1 * q2);
            cosy_cosp = 1 - 2 * (q2^2 + q3^2);
            yaw = atan2(siny_cosp, cosy_cosp);
            
            euler = [roll; pitch; yaw];
        end
    end
end
