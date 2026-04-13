/**
 * @file ServoSystem.hpp
 * @date 2026-02-27
 *
 * @brief Unified system for the SG90 servos.
 *        Angles and timing in Constants::ServoAngles (constants.h).
 *        Pins in pins.h.
 *
 *  begin()  — blocking ramp from mechanical start to home position.
 *  update() — call every loop. Handles auto-detach after idle.
 *
 *  Move methods write the servo immediately. Each servo has a hold
 *  timer: after the last move, the servo stays in position for
 *  kDetachMs before PWM is cut (servo goes loose).
 *
 *  Separator has debounce: separatorLeft/Right require kSepConfirm
 *  consecutive calls before moving; separatorCenter requires
 *  kSepClearMs of no detection before returning to center.
 *  Use separatorUpdate(warm, cool) instead of calling left/right/center
 *  directly for debounced behavior.
 *
 *  Intakes have hold timers: intakeUpperUpdate(detected) and
 *  intakeLowerUpdate(detected) deploy on detection and hold for
 *  kIntakeHoldMs after the last detection before returning home.
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

    void begin();
    void update();

    // Debounced high-level methods (use these from the state machine)
    void separatorUpdate(bool warm, bool cool);
    void intakeUpperUpdate(bool detected);
    void intakeLowerUpdate(bool detected);

    // Direct move methods (no debounce, for manual control)
    void intakeUpperHome();
    void intakeUpperDeploy();
    void intakeLowerHome();
    void intakeLowerDeploy();
    void separatorCenter();
    void separatorLeft();
    void separatorRight();
    void benefitCenter();
    void benefitLeft();
    void benefitRight();
    void holderPos1();
    void holderPos2();

    bool intakeUpperBusy() const;
    bool intakeLowerBusy() const;
    bool separatorBusy()   const;
    bool benefitBusy()     const;
    bool holderBusy()      const;

private:
    static constexpr uint8_t kNumServos = 5;

    enum Idx : uint8_t
    {
        INTAKE_UPPER = 0,
        INTAKE_LOWER,
        SEPARATOR,
        BENEFIT,
        HOLDER
    };

    Servo    _servo[kNumServos];
    uint8_t  _cur[kNumServos];
    uint32_t _tMove[kNumServos];
    uint8_t  _pin[kNumServos];

    // Separator debounce state
    uint8_t  _sepWarmCount;
    uint8_t  _sepCoolCount;
    int8_t   _sepLastDir;
    uint32_t _sepLastHitMs;

    // Intake hold state
    bool     _topActive;
    bool     _botActive;
    uint32_t _topLastHitMs;
    uint32_t _botLastHitMs;

    void _move(Idx idx, uint8_t angle);
    bool _isBusy(Idx idx, uint32_t moveMsConst) const;
    void _rampTo(Idx idx, uint8_t target);

    static uint8_t  _homeAngle(Idx idx);
    static uint8_t  _mechStart(Idx idx);
    static uint32_t _moveMs(Idx idx);
    static uint32_t _detachMs(Idx idx);
};

#endif