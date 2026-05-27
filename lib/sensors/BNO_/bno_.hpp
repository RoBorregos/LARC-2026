/*
 * @file motors.hpp
 *
 * @author Ximena Patricia García Magdaleno
 * 
 * @brief Header file for the bno Class.
 *
 * @version 0.1
 * 
 * @date 2026-01-12
 */



 // BNO055 IMU sensor class
 /*
#ifndef BNO_HPP
#define BNO_HPP


#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <tuple>

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
    Adafruit_BNO055 bno;
    sensors_event_t event;
    bool initialized;

    float wrapAngle(float angle) const;

    // filtro de calma
    float filteredYawDeg_ = 0.0f;
    bool firstYawSample_ = true;
};

#endif
*/
