/*
 * @file bno.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Source file for the BNO class.
 * @version 0.1
 * @date 2026-01-12
*/

#include "BNO/bno.hpp"

BNO::BNO() : bno(55, 0x28, &Wire), initialized(false)
{
}

bool BNO::begin()
{
    Serial.println("BEGIN >>>> BNO055 <<<<");

    while (!bno.begin())
    {
        Serial.println("Failed to BEGIN >>>> BNO055 <<<<");
        delay(1000);
    }
    Serial.println(">>>> BNO055 <<<< BEGINED successfully");

    delay(2000);
    bno.setExtCrystalUse(true);

    initialized = true;
    firstYawSample_ = true;
    filteredYawDeg_ = 0.0f;

    return true;
}

void BNO::update()
{
    if (!initialized)
        return;

    bno.getEvent(&event);

    // ===== filtro de calma =====
    static constexpr float alpha = 0.07f;   // más chico = más calmado

    float rawYawDeg = -wrapAngle(event.orientation.x);

    if (firstYawSample_)
    {
        filteredYawDeg_ = rawYawDeg;
        firstYawSample_ = false;
    }
    else
    {
        float err = wrapAngle(rawYawDeg - filteredYawDeg_);
        filteredYawDeg_ = wrapAngle(filteredYawDeg_ + alpha * err);
    }
}

float BNO::wrapAngle(float angle) const
{
    angle = fmod(angle, 360.0f);
    if (angle < 0)
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
    return event.orientation.y;
}

float BNO::getPitch() const
{
    return event.orientation.z;
}

std::tuple<float, float, float> BNO::getLinealAcceleration()
{
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::adafruit_vector_type_t::VECTOR_LINEARACCEL);

    return std::make_tuple(event.acceleration.x, event.acceleration.y, event.acceleration.z);
}

void BNO::getAngular()
{
    update();

    float current_yaw = getYaw();
    float current_roll = getRoll();
    float current_pitch = getPitch();

    (void)current_yaw;
    (void)current_roll;
    (void)current_pitch;
}

void BNO::getAngularPrinted()
{
    update();

    float current_yaw = getYaw();
    float current_roll = getRoll();
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