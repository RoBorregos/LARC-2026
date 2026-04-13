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
        constexpr float kUpdateInterval = 20.0; // ms between main loop updates
    }

    namespace Kinematics
    {
        constexpr float M1_ANGLE = 135.0f;  // Upper Left motor angle
        constexpr float M2_ANGLE = 45.0f;   // Upper Right motor angle
        constexpr float M3_ANGLE = -135.0f; // Lower Left motor angle
        constexpr float M4_ANGLE = -45.0f;  // Lower Right motor angle
    }

    namespace DriveConstants
    {
        static constexpr float kDEG2RAD = PI / 180.0f;
        constexpr float kWheelDiameter      = 0.109f;
        constexpr float kWheelRaius         = kWheelDiameter / 2.0;
        constexpr float kWheelCircumference = 2 * M_PI * kWheelRaius;
    }

    namespace PID
    {
        static constexpr float kKp = 1.15;
        static constexpr float kKi = 0.0f;
        static constexpr float kKd = 0.0012f;
        static constexpr float kOmegaMax = 0.25f;
        static constexpr float kcurrentVelocity = 0.42f;
    }

    namespace UltrasonicConstants
    {
        static constexpr uint32_t kPingPeriodMs  = 50;    // ms between pings
        static constexpr uint32_t kTrigHighUs    = 10;    // trigger pulse width (µs)
        static constexpr uint32_t kEchoTimeoutUs = 25000; // max echo wait (~4 m)
    }

    namespace QTRCalibration
    {
        constexpr size_t kNumSensors = 8;

        struct Profile
        {
            uint16_t min[kNumSensors];
            uint16_t max[kNumSensors];
        };

        constexpr Profile Front = {
            {980, 960, 960, 960, 960, 960, 960, 960},
            {1015, 1015, 1015, 1015, 1015, 1015, 1015, 1015}
        };

        constexpr Profile Rear = {
            {120,  130,  115,  140,  150,  135,  128,  122 },
            {3100, 3200, 3050, 3300, 3350, 3250, 3150, 3000}
        };

        constexpr uint16_t kBinaryThreshold = 600; // 0-1000 normalized
    }

    namespace LineFollower
    {
        constexpr int kSetpoint = 3500; // center of 0-7000 range
    }

    namespace IRCalibration
    {
        static constexpr uint16_t kThreshFL = 112;
        static constexpr uint16_t kThreshFR = 100;
        static constexpr uint16_t kThreshBL = 100;
        static constexpr uint16_t kThreshBR = 112;
        static constexpr uint16_t kHysteresis    = 20;
        static constexpr uint16_t kDebounceCount = 3;
    }

    namespace ServoAngles
    {
        // ── Startup ramp ────────────────────────────────────────
        // Speed (deg/s) for the blocking ramp in begin()
        static constexpr float kStartupSpeed = 70.0f;

        // Where attach() physically puts each servo (measure per servo)
        static constexpr uint8_t kMechStartIntakeUpper = 154;
        static constexpr uint8_t kMechStartIntakeLower = 160;
        static constexpr uint8_t kMechStartSeparator   = 90;
        static constexpr uint8_t kMechStartBenefit     = 90;
        static constexpr uint8_t kMechStartHolder      = 90;

        // ── Servo positions ─────────────────────────────────────
        static constexpr uint8_t kIntakeUpperHome   = 50;
        static constexpr uint8_t kIntakeUpperDeploy = 85;

        static constexpr uint8_t kIntakeLowerHome   = 50;
        static constexpr uint8_t kIntakeLowerDeploy = 80;

        static constexpr uint8_t kSeparatorCenter = 70;
        static constexpr uint8_t kSeparatorLeft   = 100;
        static constexpr uint8_t kSeparatorRight  = 120;

        static constexpr uint8_t kBenefitRed    = 70;
        static constexpr uint8_t kBenefitCenter = 90;
        static constexpr uint8_t kBenefitBlue   = 110;

        static constexpr uint8_t kHolderHold    = 70;
        static constexpr uint8_t kHolderRelease = 50;

        // ── Busy time (ms) — isBusy() returns true for this long after a move
        static constexpr uint32_t kIntakeUpperMoveMs = 200;
        static constexpr uint32_t kIntakeLowerMoveMs = 200;
        static constexpr uint32_t kSeparatorMoveMs   = 50;
        static constexpr uint32_t kBenefitMoveMs     = 50;
        static constexpr uint32_t kHolderMoveMs      = 50;

        // ── Detach time (ms) — PWM cut after this long with no movement
        static constexpr uint32_t kIntakeUpperDetachMs = 10000;
        static constexpr uint32_t kIntakeLowerDetachMs = 10000;
        static constexpr uint32_t kSeparatorDetachMs   = 50;
        static constexpr uint32_t kBenefitDetachMs     = 50;
        static constexpr uint32_t kHolderDetachMs      = 50;

        // Debounce / hold (used by separatorUpdate / intakeUpdate) ──
        // Separator: consecutive calls needed before actually moving
        static constexpr uint8_t  kSepConfirm = 2;
        // Separator: ms of no detection before returning to center
        static constexpr uint32_t kSepClearMs = 300;
        // Intakes: ms to hold deployed after last detection
        static constexpr uint32_t kIntakeHoldMs = 100;

    } // namespace ServoAngles

    namespace ToFConfig
    {
        static constexpr uint16_t kTimeoutMs          = 100;
        static constexpr uint32_t kTimingBudgetUs     = 20000;
        static constexpr uint16_t kContinuousPeriodMs = 20;
        static constexpr uint16_t kMuxSettleDelayMs   = 10;
    }

} // namespace Constants

#endif