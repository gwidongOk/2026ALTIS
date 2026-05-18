#include "KF1D.h"

using namespace Eigen;

KF1D::KF1D() {
    init(0.0f, 0.0f);
}

void KF1D::init(float p0, float v0) {
    _x << p0, v0;
    _cov      = Matrix2f::Identity() * 10.0f;
    _win_idx  = 0;
    _win_full = false;
    _win_var  = sigma_a_base;
    for (int i = 0; i < WIN; i++) _win[i] = 0.0f;
}

// ── Rolling window helpers ────────────────────────────────────────────────

void KF1D::pushSample(float acc_up) {
    _win[_win_idx] = acc_up;
    _win_idx = (_win_idx + 1) % WIN;
    if (_win_idx == 0) _win_full = true;
}

void KF1D::updateVariance() {
    int n = _win_full ? WIN : _win_idx;
    if (n < 2) return;                 // not enough samples yet

    float sum = 0.0f, sum_sq = 0.0f;
    for (int i = 0; i < n; i++) {
        sum    += _win[i];
        sum_sq += _win[i] * _win[i];
    }
    float mean = sum / (float)n;
    float var  = sum_sq / (float)n - mean * mean;

    // Use max(baseline, measured) so Q never collapses in steady state.
    _win_var = fmaxf(sigma_a_base, var);
}

// ── KF steps ─────────────────────────────────────────────────────────────

void KF1D::predict(float acc_up, float dt) {
    // 1. Update the rolling window and recompute σ_a²
    pushSample(acc_up);
    updateVariance();

    // 2. Build motion matrices
    float dt2 = dt * dt;
    Matrix2f A;
    A << 1.0f, dt,
         0.0f, 1.0f;
    Vector2f B;
    B << 0.5f * dt2, dt;

    // 3. Process noise Q using the live variance estimate
    //    Standard CWNA form: Q = σ_a² * [dt⁴/4, dt³/2; dt³/2, dt²]
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    Matrix2f Q;
    Q << 0.25f * dt4, 0.5f * dt3,
         0.5f * dt3,  dt2;
    Q *= _win_var;

    _x   = A * _x + B * acc_up;
    _cov = A * _cov * A.transpose() + Q;
}

void KF1D::update(float z_obs) {
    Matrix<float, 1, 2> H;
    H << 1.0f, 0.0f;

    float S = (H * _cov * H.transpose())(0) + sigma_b_sq;
    Vector2f K = _cov * H.transpose() / S;

    _x = _x + K * (z_obs - _x(0));

    Matrix2f IKH = Matrix2f::Identity() - K * H;
    _cov = IKH * _cov * IKH.transpose() + sigma_b_sq * K * K.transpose();
}
