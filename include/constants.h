/**
 * @file constants.h
 * @date 12/01/2026
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Constants for the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <math.h>

namespace Constants
{
    namespace SystemConstants
    {
        constexpr float kUpdateInterval = 20.0;
    }

    namespace DriveConstants
    {
        // "omni_motors" class
    static constexpr float kDEG2RAD = PI / 180.0f;

    constexpr float kWheelDiameter = 0.109f;
    constexpr float kWheelRaius = kWheelDiameter/2.0;
    constexpr float kWheelCircumference = 2*M_PI*kWheelRaius;

    
    }

}

#endif

