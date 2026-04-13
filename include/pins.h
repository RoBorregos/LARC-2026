#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

namespace Pins
{
    // =========================================================
    // CHASSIS MOTORS
    // =========================================================
    constexpr uint8_t kPwmPin[4] = {
        10, // PWM1 UPPER LEFT MOTOR
        2,  // PWM2 UPPER RIGHT MOTOR
        29, // PWM3 BOTTOM LEFT MOTOR
        5   // PWM4 BOTTOM RIGHT MOTOR
    };

    constexpr uint8_t kUpperMotors[4] = {
        12, // IN1.1 UPPER LEFT MOTOR    m1
        11, // IN1.2 UPPER LEFT MOTOR    m1
        38, // IN2.1 UPPER RIGHT MOTOR   m2
        37  // IN2.2 UPPER RIGHT MOTOR   m2
    };

    constexpr uint8_t kLowerMotors[4] = {
        32, // IN3.1 LOWER LEFT MOTOR    m3
        31, // IN3.2 LOWER LEFT MOTOR    m3
        26, // IN4.1 LOWER RIGHT MOTOR   m4
        9   // IN4.2 LOWER RIGHT MOTOR   m4
    };

    // =========================================================
    // ENCODERS
    // kEncoders = {B1, A1, A2, B2, A3, B3, A4, B4}
    // =========================================================
    constexpr uint8_t kEncoders[8] = {
        40, // ENC1B
        39, // ENC1A
        15, // ENC2A
        14, // ENC2B
        41, // ENC3A
        13, // ENC3B
        17, // ENC4A
        16  // ENC4B
    };

    // =========================================================
    // ELEVATOR
    // =========================================================
    constexpr uint8_t kElevatorINA1 = 8;   // IN1_MDC
    constexpr uint8_t kElevatorINA2 = 1;   // IN2_MDC
    constexpr uint8_t kElevatorPWM  = 22;  // PWM_DC

    // =========================================================
    // LIMIT SWITCH
    // =========================================================
    constexpr uint8_t kLimitSwitch = 28;

    // =========================================================
    // 74HC4067 MULTIPLEXERS
    // shared S0-S3, different SIG
    // =========================================================
    static constexpr uint8_t kMuxSig  = 24; // Sig mux QTR
    static constexpr uint8_t kMuxSig2 = 20; // Sig2 otro mux IRs

    static constexpr uint8_t kMuxS0 = 27; // S0
    static constexpr uint8_t kMuxS1 = 21; // S1
    static constexpr uint8_t kMuxS2 = 0;  // S2
    static constexpr uint8_t kMuxS3 = 25; // S3

    // =========================================================
    // QTR ARRAYS ON MUX1
    // =========================================================
    static constexpr uint8_t kQtrFrontFirstCh = 0; // C0..C7
    static constexpr uint8_t kQtrRearFirstCh  = 8; // C8..C15

    // =========================================================
    // IR SENSORS ON MUX2
    // =========================================================
     static constexpr uint8_t kIrChFL = 34;
    static constexpr uint8_t kIrChFR = 36;
    static constexpr uint8_t kIrChBL = 33;
    static constexpr uint8_t kIrChBR = 35;

    // =========================================================
    // TOF SENSORS ON I2C MUX
    // =========================================================
    static constexpr uint8_t kToFchFR = 0; // Front Right
    static constexpr uint8_t kToFchFL = 1; // Front Left
    static constexpr uint8_t kToFchBL = 2; // Back Left / placeholder
    static constexpr uint8_t kToFchBR = 3; // Back Right / placeholder

    // ======== Servos ========
    constexpr uint8_t kUpperIntakeServo = 30;
    constexpr uint8_t kLowerIntakeServo = 6;
    constexpr uint8_t kSeparatorServo   = 255;
    constexpr uint8_t kBenefitServo     = 255;
    constexpr uint8_t kHolderServo      = 255;


    // =========================================================
    // OPTIONAL / LEGACY ULTRASONICS
    // =========================================================
    constexpr uint8_t kDistanceSensors[4][2] = {
        {35, 33},     // FRONT LEFT  {TRIG, ECHO}
        {36, 34},     // FRONT RIGHT {TRIG, ECHO}
        {255, 255},   // BACK RIGHT not confirmed
        {255, 255}    // BACK LEFT  not confirmed
    };

    // =========================================================
    // OPTIONAL / LEGACY
    // =========================================================
    // constexpr uint8_t kBluetoothRx = 7;
    // constexpr uint8_t kBluetoothTx = 8;
}

#endif