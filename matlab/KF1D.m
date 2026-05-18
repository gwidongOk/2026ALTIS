classdef KF1D < handle
% KF1D — 1D vertical KF with Acceleration-Variance Adaptive Q.
%
%   State:   x = [pos; vel]^T
%   Predict: x = A·x + B·a,      P = A·P·A' + Q(adaptive)
%   Update:  Joseph-form KF update
%
%   Adaptive Q:
%     Rolling window of the last WIN acc_up samples.
%     The sample variance of this window becomes σ_a²  → Q.
%
%     Normal flight  : low variance  → small Q → baro-dominated
%     Chute deploy   : variance spikes at 416 Hz → Q inflates instantly
%     After settling : variance drops → Q returns to baseline
%
%   Globals (set before use):
%     SIGMA_A_SQ  baseline accel variance [(m/s²)²]  default 6.3e-4
%     SIGMA_B_SQ  baro variance [m²]                  default 0.089

    properties
        x       % [pos; vel]
        cov     % 2×2 covariance
    end

    properties (Constant)
        WIN = 20   % rolling window length (samples at IMU rate)
    end

    properties (Access = private)
        win        % circular buffer of acc_up samples
        win_idx    % next write position (1-based)
        win_count  % number of valid samples so far
        win_var    % current σ_a² estimate
    end

    methods
        function obj = KF1D(p0, v0)
            global SIGMA_A_SQ SIGMA_B_SQ
            if isempty(SIGMA_A_SQ), SIGMA_A_SQ = 6.3e-4; end
            if isempty(SIGMA_B_SQ), SIGMA_B_SQ = 0.089;  end
            if nargin < 1, p0 = 0; end
            if nargin < 2, v0 = 0; end

            obj.x         = [p0; v0];
            obj.cov       = 10 * eye(2);
            obj.win       = zeros(1, obj.WIN);
            obj.win_idx   = 1;
            obj.win_count = 0;
            obj.win_var   = SIGMA_A_SQ;
        end

        function predict(obj, acc, dt)
            global SIGMA_A_SQ

            % 1. Push sample into rolling window
            obj.win(obj.win_idx) = acc;
            obj.win_idx   = mod(obj.win_idx, obj.WIN) + 1;
            obj.win_count = min(obj.win_count + 1, obj.WIN);

            % 2. Recompute variance from the window
            n = obj.win_count;
            if n >= 2
                v   = obj.win(1:n);
                obj.win_var = max(SIGMA_A_SQ, var(v, 1));  % var(...,1) = population var
            end

            % 3. Build Q with live σ_a²
            A = [1 dt; 0 1];
            B = [0.5*dt^2; dt];
            Q = obj.win_var * [dt^4/4, dt^3/2;
                               dt^3/2,   dt^2];

            obj.x   = A * obj.x + B * acc;
            obj.cov = A * obj.cov * A.' + Q;
        end

        function update(obj, z)
            global SIGMA_B_SQ
            H   = [1, 0];
            S   = H * obj.cov * H.' + SIGMA_B_SQ;
            K   = obj.cov * H.' / S;

            obj.x   = obj.x + K * (z - H * obj.x);
            IKH     = eye(2) - K * H;
            obj.cov = IKH * obj.cov * IKH.' + SIGMA_B_SQ * (K * K.');
        end
    end
end
