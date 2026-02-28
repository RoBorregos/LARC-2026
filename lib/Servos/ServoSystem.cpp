/**
 * @file ServoSystem.cpp
 * @date 2026-02-27
 *
 * @brief Implementación del sistema unificado de servos SG90.
 */

#include "ServoSystem.hpp"

// Constructor
// Poniendo 255 como valor inicial para los ángulos actuales, nos aseguramos de que el primer movimiento siempre se ejecute 

ServoSystem::ServoSystem()
    : _curIntakeUpper(255),
      _curIntakeLower(255),
      _curSeparator(255),
      _curBenefitRed(255),
      _curBenefitBlue(255),
      _tIntakeUpper(0),
      _tIntakeLower(0),
      _tSeparator(0),
      _tBenefitRed(0),
      _tBenefitBlue(0)
{}

// Inicialización
// En pines se tiene 255 los servos que no se estan usando.
// si no se asigna un pin válido, el servo no se attach y no se moverá (queda inactivo).

void ServoSystem::begin()
{
    if (Pins::kUpperIntakeServo != 255)
        _intakeUpper.attach(Pins::kUpperIntakeServo);

    if (Pins::kLowerIntakeServo != 255)
        _intakeLower.attach(Pins::kLowerIntakeServo);

    if (Pins::kSeparatorServo != 255)
        _separator.attach(Pins::kSeparatorServo);

    if (Pins::kRedBenefitServo != 255)
        _benefitRed.attach(Pins::kRedBenefitServo);

    if (Pins::kBlueBenefitServo != 255)
        _benefitBlue.attach(Pins::kBlueBenefitServo);

        //Se mueven todos los servos a su posición por default

    _move(_intakeUpper, Constants::ServoAngles::kIntakeUpperHome,   _curIntakeUpper, _tIntakeUpper);
    _move(_intakeLower, Constants::ServoAngles::kIntakeLowerHome,   _curIntakeLower, _tIntakeLower);
    _move(_separator,   Constants::ServoAngles::kSeparatorCenter,   _curSeparator,   _tSeparator);
    _move(_benefitRed,  Constants::ServoAngles::kBenefitRedClosed,  _curBenefitRed,  _tBenefitRed);
    _move(_benefitBlue, Constants::ServoAngles::kBenefitBlueClosed, _curBenefitBlue, _tBenefitBlue);
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

// Compuerta Separadora

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

// Beneficio Rojo

void ServoSystem::benefitRedOpen()
{
    _move(_benefitRed, Constants::ServoAngles::kBenefitRedOpen, _curBenefitRed, _tBenefitRed);
}
void ServoSystem::benefitRedClose()
{
    _move(_benefitRed, Constants::ServoAngles::kBenefitRedClosed, _curBenefitRed, _tBenefitRed);
}

// Beneficio Azul

void ServoSystem::benefitBlueOpen()
{
    _move(_benefitBlue, Constants::ServoAngles::kBenefitBlueOpen, _curBenefitBlue, _tBenefitBlue);
}
void ServoSystem::benefitBlueClose()
{
    _move(_benefitBlue, Constants::ServoAngles::kBenefitBlueClosed, _curBenefitBlue, _tBenefitBlue);
}

// Estado individual

bool ServoSystem::intakeUpperBusy() const { return _isMoving(_tIntakeUpper, Constants::ServoAngles::kIntakeUpperMoveMs); }
bool ServoSystem::intakeLowerBusy() const { return _isMoving(_tIntakeLower, Constants::ServoAngles::kIntakeLowerMoveMs); }
bool ServoSystem::separatorBusy()   const { return _isMoving(_tSeparator,   Constants::ServoAngles::kSeparatorMoveMs);   }
bool ServoSystem::benefitRedBusy()  const { return _isMoving(_tBenefitRed,  Constants::ServoAngles::kBenefitRedMoveMs);  }
bool ServoSystem::benefitBlueBusy() const { return _isMoving(_tBenefitBlue, Constants::ServoAngles::kBenefitBlueMoveMs); }

// Helpers privados

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

