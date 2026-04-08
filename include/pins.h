#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

namespace Pins
{

    // ======== Motor PWM pins ========
    constexpr uint8_t kPwmPin[4] = {
        10, // PWM1 UPPER LEFT MOTOR
        2,  // PWM2 UPPER RIGHT MOTOR
        29, // PWM3 BOTTOM LEFT MOTOR
        5,  // PWM4 BOTTOM RIGHT MOTOR
    };

    // ======== Motor upper pins ========
    constexpr uint8_t kUpperMotors[4] = {
        12, // IN1.1 UPPER LEFT MOTOR    m1
        11, // IN1.2 UPPER LEFT MOTOR    m1
        38, // IN2.1 UPPER RIGHT MOTOR   m2
        37, // IN2.2 UPPER RIGHT MOTOR   m2
    };

    // ======== Motor lower pins ========
    constexpr uint8_t kLowerMotors[4] = {
        32, // IN3.1 LOWER LEFT MOTOR   m3
        31, // IN3.2 LOWER LEFT MOTOR   m3
        26, // IN4.1 LOWER RIGHT MOTOR  m4
        9   // IN4.2 LOWER RIGHT MOTOR  m4
    };

    // ======== Motor encoder pins ========
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

    // ========= Elevator =======
    constexpr uint8_t kElevatorINA1 = 1;
    constexpr uint8_t kElevatorINA2 = 8;
    constexpr uint8_t kElevatorPWM = 22;

    // ========= Limit Switch =======  
    constexpr uint8_t kLimitSwitch = 28;

    // ======== 74HC4067 Multiplexer ========
    static constexpr uint8_t kMuxSig = 24;
    static constexpr uint8_t kMuxSig2 = 20; // Second mux SIG (IR sensors)
    static constexpr uint8_t kMuxS0  = 27;
    static constexpr uint8_t kMuxS1  = 21;
    static constexpr uint8_t kMuxS2  = 0;
    static constexpr uint8_t kMuxS3  = 25;

    // ======== QTR – Canales en el mux ========
    // First channel on the front array (C0..C7)
    static constexpr uint8_t kQtrFrontFirstCh = 0;
    // First channel on the back array (C8..C15)
    static constexpr uint8_t kQtrRearFirstCh  = 8;

    // ======== IR Line – Channels on mux ========
    static constexpr uint8_t kIrChFL = 10;  // Channel mux - sensor Front-Left
    static constexpr uint8_t kIrChFR = 11;  // Channel mux - sensor Front-Right
    static constexpr uint8_t kIrChBL = 13;  // Channel mux - sensor Back-Left
    static constexpr uint8_t kIrChBR = 12;  // Channel mux - sensor Back-Right

    // ======== ToF sensors ========
    static constexpr uint8_t kToFchFL  = 1; // Channel I2C mux - Sensor Front Left
    static constexpr uint8_t kToFchFR  = 0; // Channel I2C mux - Sensor Front Right
    static constexpr uint8_t kToFchBL  = 2; // PLACEHOLDER
    static constexpr uint8_t kToFchBR  = 3; // PLACEHOLDER

    // ======== Servos ========
    constexpr uint8_t kUpperIntakeServo = 3;
    constexpr uint8_t kLowerIntakeServo = 4;
    constexpr uint8_t kSeparatorServo   = 6;
    constexpr uint8_t kBenefitServo     = 255;
    constexpr uint8_t kHolderServo      = 30;

}

#endif