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
    bool benefitRedBusy()  const;
    bool benefitBlueBusy() const;

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

    // Red benefit
    void benefitRedOpen();
    void benefitRedClose();

    // Blue benefit
    void benefitBlueOpen();
    void benefitBlueClose();

private:
    Servo _intakeUpper;
    Servo _intakeLower;
    Servo _separator;
    Servo _benefitRed;
    Servo _benefitBlue;

    // Current angle per servo to avoid overwriting
    uint8_t _curIntakeUpper;
    uint8_t _curIntakeLower;
    uint8_t _curSeparator;
    uint8_t _curBenefitRed;
    uint8_t _curBenefitBlue;

    // Timestamp of the last movement per servo
    uint32_t _tIntakeUpper;
    uint32_t _tIntakeLower;
    uint32_t _tSeparator;
    uint32_t _tBenefitRed;
    uint32_t _tBenefitBlue;

    void _move(Servo& srv, uint8_t angle, uint8_t& current, uint32_t& timestamp);
    bool _isMoving(uint32_t timestamp, uint32_t moveTimeMs) const;
};

#endif // SERVO_SYSTEM_HPP