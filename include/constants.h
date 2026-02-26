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
    } // namespace SystemConstants

    namespace Kinematics
    {
        constexpr float M1_ANGLE =  135.0f; // M1
        constexpr float M2_ANGLE =   45.0f; // M2
        constexpr float M3_ANGLE =  -45.0f; // M3
        constexpr float M4_ANGLE = -135.0f; // M4
    } // namespace Kinematics

    namespace DriveConstants
    {
        static constexpr float kDEG2RAD        = PI / 180.0f;
        constexpr float kWheelDiameter         = 0.109f;
        constexpr float kWheelRaius            = kWheelDiameter / 2.0;
        constexpr float kWheelCircumference    = 2 * M_PI * kWheelRaius;
    } // namespace DriveConstants

    namespace PID
    {
        static constexpr float kKp       = 4.8f;
        static constexpr float kKi       = 0.002f;
        static constexpr float kKd       = 0.06f;
        static constexpr float kOmegaMax = 0.25f;
    } // namespace PID

    namespace UltrasonicConstants
    {
        static constexpr uint32_t kPingPeriodMs  = 50;    // ms entre pings
        static constexpr uint32_t kTrigHighUs    = 10;    // µs que el trigger está en HIGH
        static constexpr uint32_t kEchoTimeoutUs = 25000; // µs (~4 m máx)
    } // namespace UltrasonicConstants

    namespace QTRCalibration
    {
        constexpr size_t kNumSensors = 8;

        struct Profile
        {
            uint16_t min[kNumSensors];
            uint16_t max[kNumSensors];
        };

        // PLACEHOLDERS
        constexpr Profile Front = {
            {800,  800,  800,  800,  800,  800,  800,  800 },
            {1017, 1017, 1017, 1017, 1017, 1017, 1017, 1017}
        };

        constexpr Profile Rear = {
            {120,  130,  115,  140,  150,  135,  128,  122 },
            {3100, 3200, 3050, 3300, 3350, 3250, 3150, 3000}
        };
    } // namespace QTRCalibration

    namespace IRCalibration
    {
        // Umbrales analógicos por sensor (0..1023).
        // Si raw >= umbral = línea detectada (antes de aplicar inversión).

        // PLACEHOLDERS
        static constexpr uint16_t kThreshFL = 512; // Front-Left
        static constexpr uint16_t kThreshFR = 512; // Front-Right
        static constexpr uint16_t kThreshBL = 512; // Back-Left
        static constexpr uint16_t kThreshBR = 512; // Back-Right
    } // namespace IRCalibration

} // namespace Constants

#endif