/**
 * @file pins.h
 * @date 12/01/2026
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Pin definitions for the robot.
 */

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

    // ======== Distance Sensors ========

    constexpr uint8_t kDistanceSensors[4][2] = {
        {36, 34}, // FRONT LEFT  {TRIG, ECHO}
        {35, 33}, // FRONT RIGHT {TRIG, ECHO}
    };

    // ======== 74HC4067 Multiplexer ========
    // NOTE: Valores TEMPORALES. Confirmar con el equipo antes de soldar.
    static constexpr uint8_t kMuxSig = 24; // SIG → pin analógico
    static constexpr uint8_t kMuxS0  = 27;
    static constexpr uint8_t kMuxS1  = 21;
    static constexpr uint8_t kMuxS2  = 0;
    static constexpr uint8_t kMuxS3  = 25;

    // ======== QTR – Canales en el mux ========
    // Primer canal del arreglo frontal (C0..C7)
    static constexpr uint8_t kQtrFrontFirstCh = 0;
    // Primer canal del arreglo trasero (C8..C15)
    static constexpr uint8_t kQtrRearFirstCh  = 8;

    // ======== IR Line – Canales en el mux ========
    // ← Modificar estos valores para reasignar los canales físicos del mux
    static constexpr uint8_t kIrChFL = 13;  // Canal mux - sensor Front-Left
    static constexpr uint8_t kIrChFR = 12;  // Canal mux - sensor Front-Right
    static constexpr uint8_t kIrChBL = 11;  // Canal mux - sensor Back-Left
    static constexpr uint8_t kIrChBR = 10;  // Canal mux - sensor Back-Right

    // ======== Line Sensors (digital, próxima competencia) ========
    static constexpr uint8_t kLineSensorFL = 28;
    static constexpr uint8_t kLineSensorFC = 0;
    static constexpr uint8_t kLineSensorFR = 27;
    static constexpr uint8_t kLineSensorBL = 21;
    static constexpr uint8_t kLineSensorBR = 20;

    // ======== Sorter ========
    constexpr uint8_t kSorterStepPin = 7;
    constexpr uint8_t kSorterDirPin  = 23;

    // ======== Elevator ========
    constexpr uint8_t kElevatorINA1 = 1;
    constexpr uint8_t kElevatorINA2 = 8;
    constexpr uint8_t kElevatorPWM  = 22;

    // ======== Intake ========
    const uint8_t kUpperIntakeServo = 3;
    const uint8_t kLowerIntakeServo = 4;
    const uint8_t kSeparatorServo   = 255;
    const uint8_t kRedBenefitServo  = 255;
    const uint8_t kBlueBenefitServo  = 255;

}

#endif