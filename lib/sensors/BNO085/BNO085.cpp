/*
 * @file bno085.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Source file for the BNO085 class.
 * @version 0.1
 * @date 2026-01-12
 */

#include "BNO085/BNO085.hpp"
#include <math.h>

static constexpr uint32_t REPORT_PERIOD_US = 10000;

BNO085::BNO085() : initialized(false)
{
}

bool BNO085::begin()
{
    Serial.println("BEGIN >>>> BNO085 <<<<");

    while (!bno.begin_I2C(0x4A, &Wire))
    {
        Serial.println("Failed to BEGIN >>>> BNO085 <<<<");
        delay(1000);
    }
    Serial.println(">>>> BNO085 <<<< BEGINED successfully");

    if (!bno.enableReport(SH2_ROTATION_VECTOR, REPORT_PERIOD_US))
    {
        Serial.println("Failed to enable ROTATION_VECTOR report");
        return false;
    }

    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_PERIOD_US))
    {
        Serial.println("Failed to enable LINEAR_ACCELERATION report");
        return false;
    }

    initialized     = true;
    firstYawSample_ = true;
    filteredYawDeg_ = 0.0f;

    return true;
}

void BNO085::update()
{
    if (!initialized)
        return;

    sh2_SensorValue_t val;
    if (!bno.getSensorEvent(&val))
        return;

    if (val.sensorId == SH2_ROTATION_VECTOR)
    {
        float qw = val.un.rotationVector.real;
        float qx = val.un.rotationVector.i;
        float qy = val.un.rotationVector.j;
        float qz = val.un.rotationVector.k;

        float rawYawDeg, rawPitchDeg, rawRollDeg;
        quaternionToEuler(qw, qx, qy, qz, rawYawDeg, rawPitchDeg, rawRollDeg);

        pitchDeg_ = rawPitchDeg;
        rollDeg_  = rawRollDeg;

        static constexpr float alpha = 0.07f;

        float rawNeg = -wrapAngle(rawYawDeg);

        if (firstYawSample_)
        {
            filteredYawDeg_ = rawNeg;
            firstYawSample_ = false;
        }
        else
        {
            float err       = wrapAngle(rawNeg - filteredYawDeg_);
            filteredYawDeg_ = wrapAngle(filteredYawDeg_ + alpha * err);
        }
    }

    if (val.sensorId == SH2_LINEAR_ACCELERATION)
    {
        linAccX_ = val.un.linearAcceleration.x;
        linAccY_ = val.un.linearAcceleration.y;
        linAccZ_ = val.un.linearAcceleration.z;
    }
}

void BNO085::quaternionToEuler(float qw, float qx, float qy, float qz,
                                float &yaw, float &pitch, float &roll) const
{
    yaw   = atan2f(2.0f * (qw * qz + qx * qy),
                   1.0f - 2.0f * (qy * qy + qz * qz)) * (180.0f / M_PI);

    float sinp = 2.0f * (qw * qy - qz * qx);
    sinp       = fmaxf(-1.0f, fminf(1.0f, sinp));
    pitch      = asinf(sinp) * (180.0f / M_PI);

    roll  = atan2f(2.0f * (qw * qx + qy * qz),
                   1.0f - 2.0f * (qx * qx + qy * qy)) * (180.0f / M_PI);
}

float BNO085::wrapAngle(float angle) const
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f)
        angle += 360.0f;

    if (angle > 180.0f)
        angle -= 360.0f;
    else if (angle < -180.0f)
        angle += 360.0f;

    return angle;
}

float BNO085::getYaw() const
{
    return filteredYawDeg_ * (M_PI / 180.0f);
}

float BNO085::getRoll() const
{
    return rollDeg_;
}

float BNO085::getPitch() const
{
    return pitchDeg_;
}

std::tuple<float, float, float> BNO085::getLinealAcceleration()
{
    update();
    return std::make_tuple(linAccX_, linAccY_, linAccZ_);
}

void BNO085::getAngular()
{
    update();

    float current_yaw   = getYaw();
    float current_roll  = getRoll();
    float current_pitch = getPitch();

    (void)current_yaw;
    (void)current_roll;
    (void)current_pitch;
}

void BNO085::getAngularPrinted()
{
    update();

    float current_yaw   = getYaw();
    float current_roll  = getRoll();
    float current_pitch = getPitch();

    Serial.println("CURRENT ORIENTATION:");
    Serial.print("  Yaw (Z-axis):   ");
    Serial.print(current_yaw * 180.0f / M_PI, 2);
    Serial.println(" deg");
    Serial.print("  Roll (Y-axis):  ");
    Serial.print(current_roll, 2);
    Serial.println(" deg");
    Serial.print("  Pitch (X-axis): ");
    Serial.print(current_pitch, 2);
    Serial.println(" deg");
}