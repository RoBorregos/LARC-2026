/*
 * @file bno.cpp
 *
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Source file for the BNO class using BNO085.
 *
 * @version 0.1
 *
 * @date 2026-01-12
 */

#include "BNO/BNO.hpp"

#include <math.h>

static constexpr uint32_t REPORT_PERIOD_US = 10000;

// BN085 .cpp
BNO::BNO()
    : bno(-1),
      initialized(false)
{
}

bool BNO::begin()
{
    Serial.println("BEGIN >>>> BNO085 <<<<");

    while (!bno.begin_I2C(0x4A, &Wire))
    {
        Serial.println("Failed to BEGIN >>>> BNO085 <<<<");
        delay(1000);
    }

    Serial.println(">>>> BNO085 <<<< BEGAN successfully");

    if (!bno.enableReport(SH2_ROTATION_VECTOR, REPORT_PERIOD_US))
    {
        Serial.println("Failed to enable ROTATION_VECTOR");
        return false;
    }

    if (!bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_PERIOD_US))
    {
        Serial.println("Failed to enable LINEAR_ACCELERATION");
        return false;
    }

    initialized     = true;
    firstYawSample_ = true;
    filteredYawDeg_ = 0.0f;

    return true;
}

void BNO::update()
{
    if (!initialized)
        return;

    sh2_SensorValue_t val;
    bool gotEvent = false;

    while (bno.getSensorEvent(&val))
    {
        gotEvent = true;

        if (val.sensorId == SH2_ROTATION_VECTOR)
        {
            float qw = val.un.rotationVector.real;
            float qx = val.un.rotationVector.i;
            float qy = val.un.rotationVector.j;
            float qz = val.un.rotationVector.k;

            float rawYawDeg;
            float rawPitchDeg;
            float rawRollDeg;

            quaternionToEuler(qw, qx, qy, qz,
                              rawYawDeg,
                              rawPitchDeg,
                              rawRollDeg);

            pitchDeg_ = rawPitchDeg;
            rollDeg_  = rawRollDeg;

            static constexpr float alpha = 0.30f;

            float rawNeg = -wrapAngle(rawYawDeg);

            if (firstYawSample_)
            {
                filteredYawDeg_ = rawNeg;
                firstYawSample_ = false;
            }
            else
            {
                float err = wrapAngle(rawNeg - filteredYawDeg_);
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

    static uint32_t lastEventMs = millis();

    if (gotEvent)
    {
        lastEventMs = millis();
    }
    else if (millis() - lastEventMs > 500)
    {
        Serial.println("BNO085 no new data, restarting reports...");

        bno.enableReport(SH2_ROTATION_VECTOR, REPORT_PERIOD_US);
        bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_PERIOD_US);

        lastEventMs = millis();
    }
}

void BNO::quaternionToEuler(
    float qw,
    float qx,
    float qy,
    float qz,
    float &yaw,
    float &pitch,
    float &roll
) const
{
    yaw =
        atan2f(
            2.0f * (qw * qz + qx * qy),
            1.0f - 2.0f * (qy * qy + qz * qz)
        ) * (180.0f / M_PI);

    float sinp =
        2.0f * (qw * qy - qz * qx);

    sinp =
        fmaxf(-1.0f, fminf(1.0f, sinp));

    pitch =
        asinf(sinp) * (180.0f / M_PI);

    roll =
        atan2f(
            2.0f * (qw * qx + qy * qz),
            1.0f - 2.0f * (qx * qx + qy * qy)
        ) * (180.0f / M_PI);
}

float BNO::wrapAngle(float angle) const
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

float BNO::getYaw() const
{
    return filteredYawDeg_ * (M_PI / 180.0f);
}

float BNO::getRoll() const
{
    return rollDeg_;
}

float BNO::getPitch() const
{
    return pitchDeg_;
}

std::tuple<float, float, float> BNO::getLinealAcceleration()
{
    update();

    return std::make_tuple(
        linAccX_,
        linAccY_,
        linAccZ_
    );
}

void BNO::getAngular()
{
    update();

    float current_yaw   = getYaw();
    float current_roll  = getRoll();
    float current_pitch = getPitch();

    (void)current_yaw;
    (void)current_roll;
    (void)current_pitch;
}

void BNO::getAngularPrinted()
{
    update();

    float current_yaw   = getYaw();
    float current_roll  = getRoll();
    float current_pitch = getPitch();

    Serial.println("CURRENT ORIENTATION:");

    Serial.print("Yaw: ");
    Serial.print(current_yaw * 180.0f / M_PI, 2);
    Serial.println(" deg");

    Serial.print("Roll: ");
    Serial.print(current_roll, 2);
    Serial.println(" deg");

    Serial.print("Pitch: ");
    Serial.print(current_pitch, 2);
    Serial.println(" deg");
}