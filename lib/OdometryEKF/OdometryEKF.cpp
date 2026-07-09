#include "OdometryEKF.hpp"

constexpr float OdometryEKF::kQ_[3][3];

OdometryEKF::OdometryEKF() {
    resetPose();
}

void OdometryEKF::resetPose() {
    x_ = 0.0f; y_ = 0.0f; th_ = 0.0f;
    P_[0][0]=0.01f; P_[0][1]=0.0f;  P_[0][2]=0.0f;
    P_[1][0]=0.0f;  P_[1][1]=0.01f; P_[1][2]=0.0f;
    P_[2][0]=0.0f;  P_[2][1]=0.0f;  P_[2][2]=0.05f;
}

void OdometryEKF::step(float dt,
                        float rpmUL, float rpmUR,
                        float rpmLL, float rpmLR,
                        float yawBNO)
{
    const float toRad = 2.0f * M_PI / 60.0f;
    const float w1 = rpmUL * toRad;
    const float w2 = rpmUR * toRad;
    const float w3 = rpmLL * toRad;
    const float w4 = rpmLR * toRad;

    const float k  = kInvSqrt2_ * kRadius_ / 4.0f;
    const float vx = ( w1 + w2 + w3 + w4) * k * kScale_;
    const float vy = (-w1 + w2 - w3 + w4) * k * kScale_;

    // Prediction
    const float c  = cosf(th_);
    const float s  = sinf(th_);
    const float xp = x_  + (vx*c - vy*s) * dt;
    const float yp = y_  + (vx*s + vy*c) * dt;
    const float tp = th_;

    float F[3][3] = {
        {1.0f, 0.0f, (-vx*s - vy*c)*dt},
        {0.0f, 1.0f, ( vx*c - vy*s)*dt},
        {0.0f, 0.0f, 1.0f             }
    };

    float FP[3][3] = {}, Pp[3][3] = {};
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            for (int l=0;l<3;l++)
                FP[i][j] += F[i][l] * P_[l][j];

    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            for (int l=0;l<3;l++)
                Pp[i][j] += FP[i][l] * F[j][l];

    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            Pp[i][j] += kQ_[i][j];

    // Correction
    const float yt = wrapPi(yawBNO - tp);
    const float Sc = Pp[2][2] + kRtheta_;
    const float K[3] = { Pp[0][2]/Sc, Pp[1][2]/Sc, Pp[2][2]/Sc };

    x_  = xp  + K[0] * yt;
    y_  = yp  + K[1] * yt;
    th_ = wrapPi(tp + K[2] * yt);

    for (int i=0;i<3;i++) {
        P_[i][0] = Pp[i][0] - K[i] * Pp[2][0];
        P_[i][1] = Pp[i][1] - K[i] * Pp[2][1];
        P_[i][2] = Pp[i][2] - K[i] * Pp[2][2];
    }
}

float OdometryEKF::wrapPi(float a) const {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}