/**
 * @file ServoSystem.cpp
 * @date 2026-02-27
 *
 * @brief Implementation of the unified servo system.
 */

#include "ServoSystem.hpp"

using namespace Constants::ServoAngles;

ServoSystem::ServoSystem()
    : _sepWarmCount(0)
    , _sepCoolCount(0)
    , _sepLastDir(0)
    , _sepLastHitMs(0)
    , _topActive(false)
    , _botActive(false)
    , _topLastHitMs(0)
    , _botLastHitMs(0)
{
    const uint8_t pins[] = {
        Pins::kUpperIntakeServo,
        Pins::kLowerIntakeServo,
        Pins::kSeparatorServo,
        Pins::kBenefitServo,
        Pins::kHolderServo
    };

    for (uint8_t i = 0; i < kNumServos; i++)
    {
        _pin[i]   = pins[i];
        _cur[i]   = 255;
        _tMove[i] = 0;
    }
}

void ServoSystem::_rampTo(Idx idx, uint8_t target)
{
    int pos = (int)_cur[idx];
    int dir = (target > pos) ? 1 : -1;

    while (pos != (int)target)
    {
        pos += dir;
        _servo[idx].write((uint8_t)pos);
        delayMicroseconds((uint32_t)(1000000UL / (uint32_t)kStartupSpeed));
    }

    _cur[idx]   = target;
    _tMove[idx] = millis();
}

void ServoSystem::begin()
{
    for (uint8_t i = 0; i < kNumServos; i++)
    {
        if (_pin[i] == 255) continue;

        Idx idx  = (Idx)i;
        uint8_t mech = _mechStart(idx);

        _servo[i].attach(_pin[i]);
        _servo[i].write(mech);
        _cur[i] = mech;
    }

    delay(500);

    for (uint8_t i = 0; i < kNumServos; i++)
    {
        if (_pin[i] == 255) continue;

        Idx idx  = (Idx)i;
        uint8_t home = _homeAngle(idx);

        if (_cur[i] != home)
            _rampTo(idx, home);
    }
}

void ServoSystem::update()
{
    uint32_t now = millis();

    for (uint8_t i = 0; i < kNumServos; i++)
    {
        if (_pin[i] == 255) continue;
        if (!_servo[i].attached()) continue;

        uint32_t detach = _detachMs((Idx)i);
        if (now - _tMove[i] >= detach)
        {
            _servo[i].detach();
        }
    }
}

// ── Debounced high-level methods ────────────────────────────────────

void ServoSystem::separatorUpdate(bool warm, bool cool)
{
    if (warm)
    {
        _sepCoolCount = 0;
        _sepWarmCount++;
        if (_sepWarmCount >= kSepConfirm) separatorLeft();
        _sepLastDir   = -1;
        _sepLastHitMs = millis();
    }
    else if (cool)
    {
        _sepWarmCount = 0;
        _sepCoolCount++;
        if (_sepCoolCount >= kSepConfirm) separatorRight();
        _sepLastDir   = 1;
        _sepLastHitMs = millis();
    }
    else if (_sepLastDir != 0 && (millis() - _sepLastHitMs >= kSepClearMs))
    {
        separatorCenter();
        _sepWarmCount = 0;
        _sepCoolCount = 0;
        _sepLastDir   = 0;
    }
}

void ServoSystem::intakeUpperUpdate(bool detected)
{
    if (detected)
    {
        intakeUpperDeploy();
        _topActive    = true;
        _topLastHitMs = millis();
    }
    else if (_topActive && (millis() - _topLastHitMs >= kIntakeHoldMs))
    {
        intakeUpperHome();
        _topActive = false;
    }
}

void ServoSystem::intakeLowerUpdate(bool detected)
{
    if (detected)
    {
        intakeLowerDeploy();
        _botActive    = true;
        _botLastHitMs = millis();
    }
    else if (_botActive && (millis() - _botLastHitMs >= kIntakeHoldMs))
    {
        intakeLowerHome();
        _botActive = false;
    }
}

// ── Direct move methods ─────────────────────────────────────────────

void ServoSystem::intakeUpperHome()   { _move(INTAKE_UPPER, kIntakeUpperHome);   }
void ServoSystem::intakeUpperDeploy() { _move(INTAKE_UPPER, kIntakeUpperDeploy); }

void ServoSystem::intakeLowerHome()   { _move(INTAKE_LOWER, kIntakeLowerHome);   }
void ServoSystem::intakeLowerDeploy() { _move(INTAKE_LOWER, kIntakeLowerDeploy); }

void ServoSystem::separatorCenter() { _move(SEPARATOR, kSeparatorCenter); }
void ServoSystem::separatorLeft()   { _move(SEPARATOR, kSeparatorLeft);   }
void ServoSystem::separatorRight()  { _move(SEPARATOR, kSeparatorRight);  }

void ServoSystem::benefitCenter() { _move(BENEFIT, kBenefitCenter); }
void ServoSystem::benefitLeft()   { _move(BENEFIT, kBenefitRed);    }
void ServoSystem::benefitRight()  { _move(BENEFIT, kBenefitBlue);   }

void ServoSystem::holderPos1() { _move(HOLDER, kHolderHold);    }
void ServoSystem::holderPos2() { _move(HOLDER, kHolderRelease); }

// ── Busy checks ─────────────────────────────────────────────────────

bool ServoSystem::intakeUpperBusy() const { return _isBusy(INTAKE_UPPER, kIntakeUpperMoveMs); }
bool ServoSystem::intakeLowerBusy() const { return _isBusy(INTAKE_LOWER, kIntakeLowerMoveMs); }
bool ServoSystem::separatorBusy()   const { return _isBusy(SEPARATOR,    kSeparatorMoveMs);   }
bool ServoSystem::benefitBusy()     const { return _isBusy(BENEFIT,      kBenefitMoveMs);     }
bool ServoSystem::holderBusy()      const { return _isBusy(HOLDER,       kHolderMoveMs);      }

// ── Private ─────────────────────────────────────────────────────────

void ServoSystem::_move(Idx idx, uint8_t angle)
{
    if (_pin[idx] == 255) return;
    if (angle == _cur[idx]) return;

    if (!_servo[idx].attached())
        _servo[idx].attach(_pin[idx]);

    _servo[idx].write(angle);
    _cur[idx]   = angle;
    _tMove[idx] = millis();
}

bool ServoSystem::_isBusy(Idx idx, uint32_t moveMsConst) const
{
    return (millis() - _tMove[idx]) < moveMsConst;
}

uint8_t ServoSystem::_homeAngle(Idx idx)
{
    switch (idx)
    {
        case INTAKE_UPPER: return kIntakeUpperHome;
        case INTAKE_LOWER: return kIntakeLowerHome;
        case SEPARATOR:    return kSeparatorCenter;
        case BENEFIT:      return kBenefitCenter;
        case HOLDER:       return kHolderHold;
    }
    return 90;
}

uint8_t ServoSystem::_mechStart(Idx idx)
{
    switch (idx)
    {
        case INTAKE_UPPER: return kMechStartIntakeUpper;
        case INTAKE_LOWER: return kMechStartIntakeLower;
        case SEPARATOR:    return kMechStartSeparator;
        case BENEFIT:      return kMechStartBenefit;
        case HOLDER:       return kMechStartHolder;
    }
    return 90;
}

uint32_t ServoSystem::_moveMs(Idx idx)
{
    switch (idx)
    {
        case INTAKE_UPPER: return kIntakeUpperMoveMs;
        case INTAKE_LOWER: return kIntakeLowerMoveMs;
        case SEPARATOR:    return kSeparatorMoveMs;
        case BENEFIT:      return kBenefitMoveMs;
        case HOLDER:       return kHolderMoveMs;
    }
    return 50;
}

uint32_t ServoSystem::_detachMs(Idx idx)
{
    switch (idx)
    {
        case INTAKE_UPPER: return kIntakeUpperDetachMs;
        case INTAKE_LOWER: return kIntakeLowerDetachMs;
        case SEPARATOR:    return kSeparatorDetachMs;
        case BENEFIT:      return kBenefitDetachMs;
        case HOLDER:       return kHolderDetachMs;
    }
    return 300;
}