#pragma once
#include <math.h>

class OdometryEKF {
public:
    OdometryEKF();

    void step(float dt,
              float rpmUL, float rpmUR,
              float rpmLL, float rpmLR,
              float yawBNO);

    void resetPose();

    float getX()    const { return x_; }
    float getY()    const { return y_; }
    float getTheta()const { return th_; }
    float getDist() const { return sqrtf(x_*x_ + y_*y_); }

private:
    float x_  = 0.0f;
    float y_  = 0.0f;
    float th_ = 0.0f;
    float P_[3][3];

    static constexpr float kRadius_   = 0.054f;
    static constexpr float kInvSqrt2_ = 0.70710678f;
    static constexpr float kScale_    = 2.57f;
    static constexpr float kRtheta_   = 0.02f;

    static constexpr float kQ_[3][3] = {
        {0.0005f, 0.0f,    0.0f   },
        {0.0f,    0.0005f, 0.0f   },
        {0.0f,    0.0f,    0.003f }
    };

    float wrapPi(float a) const;
};