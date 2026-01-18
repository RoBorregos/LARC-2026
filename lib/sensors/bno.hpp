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

#ifndef BNO_HPP
#define BNO_HPP

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <tuple> //optional 


class BNO

{
public:
    BNO();
    bool begin();
    void update();
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    void runCalibration();
    void getAngular();
    std::tuple<float, float, float> getLinealAcceleration();

private:
    Adafruit_BNO055 bno;
    sensors_event_t event;
    bool initialized;

    float wrapAngle(float angle) const;
};

#endif
