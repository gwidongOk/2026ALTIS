#include "KF1D.h"

using namespace Eigen;

KF1D::KF1D() {
    init(0.0f, 0.0f);
}

void KF1D::init(float p0, float v0) {
    _x << p0, v0;
    _P = Matrix2f::Identity() * 10.0f; // Initial uncertainty
}

void KF1D::predict(float acc_up, float dt) {
    float dt2 = dt * dt;
    
    // A = [1 dt; 0 1] (Report Page 6)
    Matrix2f A;
    A << 1.0f, dt,
         0.0f, 1.0f;

    // B = [0.5*dt^2; dt] (Report Page 6)
    Vector2f B;
    B << 0.5f * dt2, dt;

    // 1. State Prediction: x = A*x + B*u
    _x = A * _x + B * acc_up;

    // 2. Covariance Prediction: P = A*P*A' + Q
    // Process noise matrix Q (Report Page 6)
    Matrix2f Q;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    Q << 0.25f * dt4, 0.5f * dt3,
         0.5f * dt3,  dt2;
    Q *= sigma_a_sq;

    _P = A * _P * A.transpose() + Q;
}

void KF1D::update(float z_obs) {
    // Measurement matrix H = [1, 0] (Report Page 7)
    Matrix<float, 1, 2> H;
    H << 1.0f, 0.0f;

    // 3. Kalman Gain: K = P*H' * inv(H*P*H' + R)
    float S = H * _P * H.transpose() + sigma_b_sq;
    Vector2f K = _P * H.transpose() / S;

    // 4. State Update: x = x + K*(z - H*x)
    _x = _x + K * (z_obs - _x(0));

    // 5. Covariance Update: P = (I - K*H)*P
    _P = (Matrix2f::Identity() - K * H) * _P;
    
    // Ensure symmetry to prevent numerical drift
    _P = 0.5f * (_P + _P.transpose());
}
