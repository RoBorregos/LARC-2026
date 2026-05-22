#ifndef Pins_h
#define Pins_h

#include <Arduino.h>

// Pines de Balboa

namespace PinsBalboa
{
    // CHASSIS MOTORS
    constexpr uint8_t kPwmPin[4] = {
        8,  // PWM_M1
        9,  // PWM_M2
        10, // PWM_M3
        11  // PWM_M4
    };

    constexpr uint8_t kUpperMotors[4] = {
        33, // IN1_M1
        34, // IN2_M1
        35, // IN1_M2
        36  // IN2_M2
    };

    constexpr uint8_t kLowerMotors[4] = {
        37, // IN1_M3
        38, // IN2_M3
        39, // IN1_M4
        40  // IN2_M4
    };

    // ENCODERS / ENABLES
    constexpr uint8_t kEncoders[8] = {
        1,  // ENA_M1
        0,  // ENB_M1
        25, // ENA_M2
        24, // ENB_M2
        32, // ENA_M3
        31, // ENB_M3
        21, // ENA_M4
        20  // ENB_M4
    };

    // MOTOR 5 / ELEVATOR
    constexpr uint8_t kElevatorINA1 = 16;  // IN1_M5
    constexpr uint8_t kElevatorINA2 = 17;  // IN2_M5
    constexpr uint8_t kElevatorPWM  = 12;  // PWM_M5

    // LIMIT SWITCH
    constexpr uint8_t kLimitSwitch = 13; // Libre1 o cambiar si lo conectas a otro pin

    // 74HC4067 MULTIPLEXERS
    static constexpr uint8_t kMuxSig  = 26; // SIG_A0
    static constexpr uint8_t kMuxSig2 = 22; // SIG_A1

    static constexpr uint8_t kMuxS0 = 27; // S0_MUX
    static constexpr uint8_t kMuxS1 = 28; // S1_MUX
    static constexpr uint8_t kMuxS2 = 29; // S2_MUX
    static constexpr uint8_t kMuxS3 = 30; // S3_MUX

    // QTR ARRAYS ON MUX1
    static constexpr uint8_t kQtrFrontFirstCh = 0;
    static constexpr uint8_t kQtrRearFirstCh  = 8;

    // IR SENSORS ON MUX2
    // Estos son canales del mux, NO pines físicos
    static constexpr uint8_t kIrChFL = 0;
    static constexpr uint8_t kIrChFR = 1;
    static constexpr uint8_t kIrChBL = 2;
    static constexpr uint8_t kIrChBR = 3;

    // I2C
    static constexpr uint8_t kSCL = 19;
    static constexpr uint8_t kSDA = 18;

    // TOF SENSORS ON I2C MUX
    static constexpr uint8_t kToFchFR = 0;
    static constexpr uint8_t kToFchFL = 1;
    static constexpr uint8_t kToFchBL = 2;
    static constexpr uint8_t kToFchBR = 3;

    // SERVOS
    constexpr uint8_t kUpperIntakeServo = 2; // SERVO1
    constexpr uint8_t kLowerIntakeServo = 3; // SERVO2
    constexpr uint8_t kSeparatorServo   = 4; // SERVO3
    constexpr uint8_t kBenefitServo     = 5; // SERVO4
    constexpr uint8_t kHolderServo      = 6; // SERVO5
}

#endif