/**
 * @file ServoSystem.hpp
 * @date 2026-02-27
 *
 * @brief Sistema unificado de servos SG90 no bloqueante.
 *        Cada servo tiene su propio isBusy() independiente.
 *        Los ángulos se definen en Constants::ServoAngles (constants.h).
 *        Los pines se definen en Pins (pins.h).
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

    // Inicializa todos los servos en su posición por defecto
    void begin();

    //   Estado individual por servo  
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

    // Separador
    void separatorCenter();
    void separatorLeft();
    void separatorRight();

    // Beneficiadero Rojo
    void benefitRedOpen();
    void benefitRedClose();

    // Beneficiadero Azul
    void benefitBlueOpen();
    void benefitBlueClose();

private:
    Servo _intakeUpper;
    Servo _intakeLower;
    Servo _separator;
    Servo _benefitRed;
    Servo _benefitBlue;

    // Ángulo actual por servo, evita re-escribir el mismo valor
    uint8_t _curIntakeUpper;
    uint8_t _curIntakeLower;
    uint8_t _curSeparator;
    uint8_t _curBenefitRed;
    uint8_t _curBenefitBlue;

    // Timestamp del último movimiento por servo
    uint32_t _tIntakeUpper;
    uint32_t _tIntakeLower;
    uint32_t _tSeparator;
    uint32_t _tBenefitRed;
    uint32_t _tBenefitBlue;

    void _move(Servo& srv, uint8_t angle, uint8_t& current, uint32_t& timestamp);
    bool _isMoving(uint32_t timestamp, uint32_t moveTimeMs) const;
};

#endif // SERVO_SYSTEM_HPP