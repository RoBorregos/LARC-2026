#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

// =========================================================
// PINS CONFIGURATION
// =========================================================

namespace Pins
{
    // =========================================================
    // CHASSIS MOTORS
    // =========================================================
    constexpr uint8_t kPwmPin[4] = {
        8,   // PWM_M1
        9,   // PWM_M2
        10,  // PWM_M3
        11   // PWM_M4
    };

    constexpr uint8_t kUpperMotors[4] = {
        33,  // IN1_M1
        34,  // IN2_M1
        35,  // IN1_M2
        36   // IN2_M2
    };

    constexpr uint8_t kLowerMotors[4] = {
        37,  // IN1_M3
        38,  // IN2_M3
        39,  // IN1_M4
        40   // IN2_M4
    };

    // =========================================================
    // ENCODERS
    // kEncoders = {A1,B1,A2,B2,A3,B3,A4,B4}
    // =========================================================
    constexpr uint8_t kEncoders[8] = {
        1,   // ENC1A
        0,   // ENC1B

        25,  // ENC2A
        24,  // ENC2B

        32,  // ENC3A
        31,  // ENC3B

        21,  // ENC4A
        20   // ENC4B
    };

    // =========================================================
    // ELEVATOR MOTOR (M5)
    // =========================================================
    constexpr uint8_t kElevatorINA1 = 16; // IN1_M5
    constexpr uint8_t kElevatorINA2 = 17; // IN2_M5
    constexpr uint8_t kElevatorPWM  = 12; // PWM_M5

    // =========================================================
    // LIMIT SWITCH
    // =========================================================
    constexpr uint8_t kLimitSwitch = 13;

    // =========================================================
    // 74HC4067 MULTIPLEXER FOR QTR
    // =========================================================
    static constexpr uint8_t kMuxSig = 26; // SIG_A0

    static constexpr uint8_t kMuxS0 = 27; // S0_MUX
    static constexpr uint8_t kMuxS1 = 28; // S1_MUX
    static constexpr uint8_t kMuxS2 = 29; // S2_MUX
    static constexpr uint8_t kMuxS3 = 30; // S3_MUX

    // =========================================================
    // QTR ARRAYS ON MUX
    // =========================================================
    static constexpr uint8_t kQtrFrontFirstCh = 0; // C0-C7
    static constexpr uint8_t kQtrRearFirstCh  = 8; // C8-C15

    // =========================================================
    // IR SENSORS DIRECT TO TEENSY
    // =========================================================
    static constexpr uint8_t kIrChFL = 23;
    static constexpr uint8_t kIrChFR = 15;
    static constexpr uint8_t kIrChBL = 41;
    static constexpr uint8_t kIrChBR = 14;

    // =========================================================
    // I2C
    // =========================================================
    static constexpr uint8_t kSDA = 18;
    static constexpr uint8_t kSCL = 19;

    // =========================================================
    // TOF SENSORS ON I2C MUX
    // =========================================================
    static constexpr uint8_t kToFchFR = 0;
    static constexpr uint8_t kToFchFL = 1;
    static constexpr uint8_t kToFchBL = 2;
    static constexpr uint8_t kToFchBR = 3;

    // =========================================================
    // PCA9685 SERVO CHANNELS
    // =========================================================
    constexpr uint8_t kUpperIntakeServo = 15;
    constexpr uint8_t kLowerIntakeServo = 13;
    constexpr uint8_t kSeparatorServo   = 12;
    constexpr uint8_t kBenefitServo     = 11;
    constexpr uint8_t kHolderServo      = 7;

    // =========================================================
    // OPTIONAL / LEGACY ULTRASONICS
    // =========================================================
    constexpr uint8_t kDistanceSensors[4][2] = {
        {35, 33},
        {36, 34},
        {255, 255},
        {255, 255}
    };
}

#endif