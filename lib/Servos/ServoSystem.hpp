/**
 * @file ServoSystem.hpp
 * @date 2026-02-27
 *
 * @brief Unified sysem for the SG90 servos
 *        Each servo with its own isbusy()
 *        Angles are defined in Constants::ServoAngles (constants.h).
 *        Pins are defined on pins.h
 */

#ifndef SERVO_SYSTEM_HPP
#define SERVO_SYSTEM_HPP

#include <Arduino.h>
#include <Servo.h>
#include "constants.h"
#include "pins.h"

class ServoSystem
{
public:
    ServoSystem();

    // Starts all the servos on the default pos
    void begin();

    // Servo state
    bool intakeUpperBusy() const;
    bool intakeLowerBusy() const;
    bool separatorBusy()   const;
    bool benefitBusy() const;
    bool holderBusy()     const;

    // Intake Superior
    void intakeUpperHome();
    void intakeUpperDeploy();

    // Intake Inferior
    void intakeLowerHome();
    void intakeLowerDeploy();

    // Separator
    void separatorCenter();
    void separatorLeft();
    void separatorRight();

    // Benefits
    void benefitRed();
    void benefitBlue();
    void benefitCenter();

    // Holder
    void holderHold();
    void holderRelease();

private:
    Servo _intakeUpper;
    Servo _intakeLower;
    Servo _separator;
    Servo _benefit;
    Servo _holder;

    // Current angle per servo to avoid overwriting
    uint8_t _curIntakeUpper;
    uint8_t _curIntakeLower;
    uint8_t _curSeparator;
    uint8_t _curBenefit;
    uint8_t _curholder;

    // Timestamp of the last movement per servo
    uint32_t _tIntakeUpper;
    uint32_t _tIntakeLower;
    uint32_t _tSeparator;
    uint32_t _tBenefit;
    uint32_t _tholder;

    void _move(Servo& srv, uint8_t angle, uint8_t& current, uint32_t& timestamp);
    bool _isMoving(uint32_t timestamp, uint32_t moveTimeMs) const;
};

#endif // SERVO_SYSTEM_HPP