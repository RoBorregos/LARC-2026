#ifndef BNO_HPP
#define BNO_HPP

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <tuple>
// BN085 .cpp

class BNO
{
public:
    BNO();

    bool begin();
    void update();

    float getYaw() const;
    float getRoll() const;
    float getPitch() const;

    std::tuple<float, float, float> getLinealAcceleration();

    void getAngular();
    void getAngularPrinted();

private:
    Adafruit_BNO08x bno;
    sh2_SensorValue_t sensorValue;

    bool initialized;

    float yawDeg_   = 0.0f;
    float rollDeg_  = 0.0f;
    float pitchDeg_ = 0.0f;

    float linAccX_ = 0.0f;
    float linAccY_ = 0.0f;
    float linAccZ_ = 0.0f;

    float wrapAngle(float angle) const;

    float filteredYawDeg_ = 0.0f;
    bool firstYawSample_  = true;

    void quaternionToEuler(float qw, float qx, float qy, float qz,
                           float &yaw, float &pitch, float &roll) const;
};

#endif