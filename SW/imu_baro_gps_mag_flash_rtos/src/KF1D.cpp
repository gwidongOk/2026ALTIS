#include "KF1D.h"

KF1D::KF1D() {
    init(0, 0, 0);
}

void KF1D::init(float z0, float v0, float b0) {
    x[0] = z0; x[1] = v0; x[2] = b0;
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    P[0][0] = 10.0f; P[1][1] = 10.0f; P[2][2] = 0.1f;
}

void KF1D::predict(float accel_m_s2, float dt) {
    float dt2 = dt * dt;
    float a = accel_m_s2 - x[2];

    // 1. State Prediction
    x[0] = x[0] + x[1] * dt + 0.5f * a * dt2;
    x[1] = x[1] + a * dt;
    x[2] = x[2]; // Constant bias model

    // 2. Covariance Prediction (P = F*P*F' + Q)
    // F = [1, dt, -0.5*dt^2; 0, 1, -dt; 0, 0, 1]
    float F[3][3] = {
        {1.0f, dt,   -0.5f * dt2},
        {0.0f, 1.0f, -dt},
        {0.0f, 0.0f, 1.0f}
    };

    float PFt[3][3] = {0};
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            for(int k=0; k<3; k++) {
                PFt[i][j] += P[i][k] * F[j][k];
            }
        }
    }

    float newP[3][3] = {0};
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            for(int k=0; k<3; k++) {
                newP[i][j] += F[i][k] * PFt[k][j];
            }
        }
    }

    // Add process noise Q
    newP[0][0] += Q_accel * dt2 * dt2 * 0.25f;
    newP[1][1] += Q_accel * dt2;
    newP[2][2] += Q_bias * dt;

    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            P[i][j] = newP[i][j];
        }
    }
}

void KF1D::updateBaro(float z_obs, float var_z) {
    // H = [1, 0, 0]
    // y = z_obs - H*x
    float y = z_obs - x[0];
    
    // S = H*P*H' + R
    float S = P[0][0] + var_z;
    
    // K = P*H' / S
    float K[3];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    K[2] = P[2][0] / S;
    
    // x = x + K*y
    x[0] += K[0] * y;
    x[1] += K[1] * y;
    x[2] += K[2] * y;
    
    // P = (I - K*H)*P
    float IKH[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * (j == 0 ? 1.0f : 0.0f);
        }
    }
    
    float nextP[3][3] = {0};
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            for(int k=0; k<3; k++) {
                nextP[i][j] += IKH[i][k] * P[k][j];
            }
        }
    }
    
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            P[i][j] = nextP[i][j];
        }
    }
}
