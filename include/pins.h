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
        12, // IN1.1 UPPER LEFT MOTOR    m1
        11, // IN1.2 UPPER LEFT MOTOR    m1
        38, // IN2.1 UPPER RIGHT MOTOR   m2
        37, // IN2.2 UPPER RIGHT MOTOR   m2
    };

    constexpr uint8_t kLowerMotors[4] = {
        32, // IN3.1 LOWER LEFT MOTOR   m3
        31, // IN3.2 LOWER LEFT MOTOR   m3
        26, // IN4.1 LOWER RIGHT MOTOR  m4
        9   // IN4.2 LOWER RIGHT MOTOR  m4
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

    // =======Line Sensor=======
    constexpr uint8_t kQTR1[8]{
        44, // ENC1B
        49, // ENC1A
        25, // ENC2A
        24, // ENC2B
        51, // ENC3A
        33, // ENC3B
        27, // ENC4A
        26  // ENC4B
    };

    constexpr uint8_t kQTR2[8]{
        50, // ENC1B
        49, // ENC1A
        55, // ENC2A
        64, // ENC2B
        31, // ENC3A
        73, // ENC3B
        97, // ENC4A
        26  // ENC4B
    };

    // ======== Distance Sensors========

    constexpr uint8_t kDistanceSensors[4][2] = {
        {36, 34}, // FRONT LEFT {TRIG, ECHO}
        {35, 33}, // FRONT RIGHT {TRIG, ECHO}
        {37, 32}, // BACK RIGHT {TRIG, ECHO}     >>>    nuevos
        {31, 34}, // BACK LEFt {TRIG, ECHO}
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

    // ========= Elevator =======

    constexpr uint8_t kElevatorINA1 = 1;
    constexpr uint8_t kElevatorINA2 = 8;
    constexpr uint8_t kElevatorPWM = 22;




    // ====== LED =====
    const uint8_t kLed1 = 6;
    const uint8_t kLed2 = 10;

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

 

    // ======== Intake ========
    const uint8_t kUpperIntakeServo = 3;
    const uint8_t kLowerIntakeServo = 4;
    const uint8_t kSeparatorServo   = 6;
    const uint8_t kBenefitServo     = 7;
    const uint8_t kHolderServo      = 23;

}

#endif