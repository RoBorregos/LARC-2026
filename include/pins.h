#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

namespace Pins
{

    // ======== Chassis Motors ========
    constexpr uint8_t kPwmPin[4] = {
        10, // PWM1 UPPER LEFT MOTOR
        2,  // PWM2 UPPER RIGHT MOTOR
        29, // PWM3 BOTTOM LEFT MOTOR
        5,  // PWM4 BOTTOM RIGHT MOTOR
    };

    constexpr uint8_t kUpperMotors[4] = {
        12, // IN1.1 UPPER LEFT MOTOR
        11, // IN1.2 UPPER LEFT MOTOR
        38, // IN2.1 UPPER RIGHT MOTOR
        37, // IN2.2 UPPER RIGHT MOTOR
    };

    constexpr uint8_t kLowerMotors[4] = {
        32, // IN3.1 LOWER LEFT MOTOR
        31, // IN3.2 LOWER LEFT MOTOR
        26, // IN4.1 LOWER RIGHT MOTOR
        9   // IN4.2 LOWER RIGHT MOTOR
    };

    constexpr uint8_t kEncoders[8]{
        40, // ENC1B
        39, // ENC1A
        15, // ENC2A
        14, // ENC2B
        41, // ENC3A
        13, // ENC3B
        17, // ENC4A
        16  // ENC4B
    };

    // ======== Distance Sensors========

    constexpr uint8_t kDistanceSensors[4][2] = {
        {36, 34}, // FRONT LEFT {TRIG, ECHO}
        {35, 33}, // FRONT RIGHT {TRIG, ECHO}
    };

    // ======== Line Sensors ========

    static constexpr uint8_t kLineSensorFL = 28;
    static constexpr uint8_t kLineSensorFC = 0;
    static constexpr uint8_t kLineSensorFR = 27;
    static constexpr uint8_t kLineSensorBL = 21;
    static constexpr uint8_t kLineSensorBR = 20;

    // ======== Sorter ========

    // NOT READY
    constexpr uint8_t kSorterStepPin = 7;
    constexpr uint8_t kSorterDirPin = 23;

    // ======== Elevator ========

    constexpr uint8_t kElevatorINA1 = 1;
    constexpr uint8_t kElevatorINA2 = 8;
    constexpr uint8_t kElevatorPWM = 22;

    // ======== Intake ========

    const uint8_t kUpperIntakeServo = 3;
    const uint8_t kLowerIntakeServo = 6;
    const uint8_t kRampIntakeServo = 4;

    // NOT READY
    const uint8_t kDropperServo = 30;

}

#endif