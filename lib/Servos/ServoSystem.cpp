/**
 * @file ServoSystem.cpp
 * @date 2026-02-27
 *
 * @brief Implementation of the unified system for servos SG90
 */

#include "ServoSystem.hpp"


// Constructor
// Writing 255 as initial value, we make sure that the first movement always executes.

ServoSystem::ServoSystem()
    : _curIntakeUpper(255),
      _curIntakeLower(255),
      _curSeparator(255),
      _curBenefit(255),
      _curholder(255),
      _tIntakeUpper(0),
      _tIntakeLower(0),
      _tSeparator(0),
      _tBenefit(0),
      _tholder(0)
{}

// Initialization

void ServoSystem::begin()
{
        _intakeUpper.attach(Pins::kUpperIntakeServo);
        _intakeLower.attach(Pins::kLowerIntakeServo);
        _separator.attach(Pins::kSeparatorServo);
        _benefit.attach(Pins::kBenefitServo);
        _holder.attach(Pins::kHolderServo);

        //All servos move to their default position

    _move(_intakeUpper, Constants::ServoAngles::kIntakeUpperHome,   _curIntakeUpper, _tIntakeUpper);
    _move(_intakeLower, Constants::ServoAngles::kIntakeLowerHome,   _curIntakeLower, _tIntakeLower);
    _move(_separator,   Constants::ServoAngles::kSeparatorCenter,   _curSeparator,   _tSeparator);
    _move(_benefit, Constants::ServoAngles::kBenefitCenter, _curBenefit, _tBenefit);
    _move(_holder,      Constants::ServoAngles::kHolderHold,       _curholder,      _tholder);
}

// Intake Superior

void ServoSystem::intakeUpperHome()
{
    _move(_intakeUpper, Constants::ServoAngles::kIntakeUpperHome, _curIntakeUpper, _tIntakeUpper);
}
void ServoSystem::intakeUpperDeploy()

{
    _move(_intakeUpper, Constants::ServoAngles::kIntakeUpperDeploy, _curIntakeUpper, _tIntakeUpper);
}

// Intake Inferior

void ServoSystem::intakeLowerHome()
{
    _move(_intakeLower, Constants::ServoAngles::kIntakeLowerHome, _curIntakeLower, _tIntakeLower);
}
void ServoSystem::intakeLowerDeploy()
{
    _move(_intakeLower, Constants::ServoAngles::kIntakeLowerDeploy, _curIntakeLower, _tIntakeLower);
}

// Separator

void ServoSystem::separatorCenter()
{
    _move(_separator, Constants::ServoAngles::kSeparatorCenter, _curSeparator, _tSeparator);
}
void ServoSystem::separatorLeft()
{
    _move(_separator, Constants::ServoAngles::kSeparatorLeft, _curSeparator, _tSeparator);
}
void ServoSystem::separatorRight()
{
    _move(_separator, Constants::ServoAngles::kSeparatorRight, _curSeparator, _tSeparator);
}

// Benefits

void ServoSystem::benefitRed()
{
    _move(_benefit, Constants::ServoAngles::kBenefitRed, _curBenefit, _tBenefit);
}
void ServoSystem::benefitBlue()
{
    _move(_benefit, Constants::ServoAngles::kBenefitBlue, _curBenefit, _tBenefit);
}
void ServoSystem::benefitCenter()
{
    _move(_benefit, Constants::ServoAngles::kBenefitCenter, _curBenefit, _tBenefit);
}

//Holder

void ServoSystem::holderHold()
{
    _move(_holder, Constants::ServoAngles::kHolderHold, _curholder, _tholder);
}

void ServoSystem::holderRelease()
{
    _move(_holder, Constants::ServoAngles::kHolderRelease, _curholder, _tholder);
}
// Believed individual state per servo (true = moving, false = idle)

bool ServoSystem::intakeUpperBusy() const { return _isMoving(_tIntakeUpper, Constants::ServoAngles::kIntakeUpperMoveMs); }
bool ServoSystem::intakeLowerBusy() const { return _isMoving(_tIntakeLower, Constants::ServoAngles::kIntakeLowerMoveMs); }
bool ServoSystem::separatorBusy()   const { return _isMoving(_tSeparator,   Constants::ServoAngles::kSeparatorMoveMs);   }
bool ServoSystem::benefitBusy() const { return _isMoving(_tBenefit, Constants::ServoAngles::kBenefitMoveMs); }
bool ServoSystem::holderBusy()     const { return _isMoving(_tholder,      Constants::ServoAngles::kHolderMoveMs);     }

// Private methods

void ServoSystem::_move(Servo& srv, uint8_t angle, uint8_t& current, uint32_t& timestamp)
{
    if (angle == current) return;
    srv.write(angle);
    current   = angle;
    timestamp = millis();
}

bool ServoSystem::_isMoving(uint32_t timestamp, uint32_t moveTimeMs) const
{
    return (millis() - timestamp) < moveTimeMs;
}
