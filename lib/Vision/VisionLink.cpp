/**
   @file VisionLink.cpp
   @date 2026-08-19
  
   @brief Implementation of the Orin link: serial glue plus every policy
          decision the protocol implies. See PROTOCOL.md sections 7 and 8
          for the confirmation and watchdog rules encoded here.
 */

#include "VisionLink.hpp"

using VisionProto::Phase;
using VisionProto::SeparatorCode;

namespace
{
// The confirmation depth, read once from the one place it is configured.
constexpr uint8_t kConfirmFrames = Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES;

ServoSystem::SeparatorPos toServoPos(SeparatorCode code)
{
    switch (code)
    {
        case SeparatorCode::LEFT:  return ServoSystem::SeparatorPos::LEFT;
        case SeparatorCode::RIGHT: return ServoSystem::SeparatorPos::RIGHT;
        default:                   return ServoSystem::SeparatorPos::NEUTRAL;
    }
}
} // namespace

// Lifecycle

VisionLink::VisionLink(Stream &port, ServoSystem &servos, uint32_t timeoutMs)
    : _serial(port)
    , _servos(servos)
    , _link(timeoutMs)
    , _appliedPhase(Phase::IDLE)
    , _status(0)
    , _separatorInvalid(false)
    , _criticalError(false)
    , _separatorError(false)
    , _benefitsError(false)
    , _beansSent(false)
    , _benefitsSent(false)
    , _stopSent(false)
{
    _confirm.reset((uint8_t)Phase::IDLE, 0);
}

void VisionLink::begin()
{
    _status           = 0;
    _separatorInvalid = false;
    clearErrors();
    resetGuards();
    _link.resetStats();
    _goSafe(); // includes closing and re arming both benefit doors
}

void VisionLink::update()
{
    const uint32_t now = millis();

    // Consume everything that arrived since the last loop
    while (_serial.available())
    {
        const VisionProto::LinkEvent ev =
            _link.feedByte((uint8_t)_serial.read(), now);

        // Rejected and duplicate frames fall through here on purpose: they
        // give no confirmation credit and do not feed the watchdog.
        if (!ev.accepted)
            continue;

        _status           = ev.cmd.status;
        _separatorInvalid = ev.cmd.separatorInvalid;
        _latchFaults(ev.cmd.status);

        // Safety first, and without waiting for confirmation.
        _applySafetyImmediate(ev.cmd);

        // Everything else has to earn REQUIRED_CONFIRMATION_FRAMES.
        if (_confirm.sample((uint8_t)ev.cmd.phase, ev.cmd.rawPayload, kConfirmFrames))
            _applyCommand(ev.cmd);
    }

    // Watchdog: the Orin went quiet
    if (_link.checkTimeout(now))
    {
        _goSafe();
        _status = 0;
        // A dead link is a critical fault: the state machine must see that
        // nobody is steering the sorter any more.
        _criticalError = true;
        // Let the state machine re-request its phase once the Orin is back.
        resetGuards();
    }

    // Benefit-door auto close timers
    _servos.update(now);
}

// Phase requests

void VisionLink::startBeans()
{
    if (!_beansSent)
    {
        _serial.write(VisionProto::kCmdStartBeans);
        _beansSent = true;
    }
}

void VisionLink::startBenefits()
{
    if (!_benefitsSent)
    {
        _serial.write(VisionProto::kCmdStartBenefits);
        _benefitsSent = true;
    }
}

void VisionLink::stop()
{
    if (!_stopSent)
    {
        _serial.write(VisionProto::kCmdStop);
        _stopSent = true;
    }
}

void VisionLink::requestStatus()
{
    _serial.write(VisionProto::kCmdStatus);
}

void VisionLink::resetGuards()
{
    _beansSent    = false;
    _benefitsSent = false;
    _stopSent     = false;
}

// Status

bool VisionLink::isOrinReady() const
{
    return (_status & VisionProto::kStatusOrinReady) != 0;
}

bool VisionLink::isBeansRunning() const
{
    return (_status & VisionProto::kStatusBeansRunning) != 0;
}

bool VisionLink::isBenefitsRunning() const
{
    return (_status & VisionProto::kStatusBenefitsRunning) != 0;
}

void VisionLink::clearErrors()
{
    _criticalError  = false;
    _separatorError = false;
    _benefitsError  = false;
}

void VisionLink::_latchFaults(uint8_t status)
{
    if (status & VisionProto::kStatusCriticalMask)  _criticalError  = true;
    if (status & VisionProto::kStatusSeparatorFault) _separatorError = true;
    if (status & VisionProto::kStatusBenefitsFault)  _benefitsError  = true;
}

// Applying commands

void VisionLink::_applySafetyImmediate(const VisionProto::Command &cmd)
{
    // HALT and IDLE mean "stop now". Neither waits for a second opinion.
    if (cmd.phase == Phase::HALT || cmd.phase == Phase::IDLE)
    {
        if (_appliedPhase != cmd.phase)
        {
            _servos.safeState();
            _appliedPhase = cmd.phase;
            // The payload of these phases is always 0 (validated upstream).
            _confirm.reset((uint8_t)cmd.phase, 0);
        }
        return;
    }

    if (_appliedPhase == Phase::BENEFITS && cmd.phase != Phase::BENEFITS)
        _servos.closeBenefits();

    if (cmd.phase == Phase::BENEFITS)
    {
        if (!cmd.benefit1Open) _servos.setBenefit(0, false);
        if (!cmd.benefit2Open) _servos.setBenefit(1, false);
    }
}

void VisionLink::_applyCommand(const VisionProto::Command &cmd)
{
    _appliedPhase = cmd.phase;

    switch (cmd.phase)
    {
        case Phase::BEANS:
            _servos.closeBenefits();
            _servos.setIntakeUpper(cmd.intakeUpper);
            _servos.setIntakeLower(cmd.intakeLower);
            _servos.setSeparator(toServoPos(cmd.separator));
            break;

        case Phase::BENEFITS:
            _servos.setIntakeUpper(false);
            _servos.setIntakeLower(false);
            _servos.setSeparator(ServoSystem::SeparatorPos::NEUTRAL);
            _servos.setBenefit(0, cmd.benefit1Open);
            _servos.setBenefit(1, cmd.benefit2Open);
            break;

        case Phase::IDLE:
        case Phase::HALT:
            _servos.safeState();
            break;
    }
}

void VisionLink::_goSafe()
{
    _servos.safeState();
    _appliedPhase = Phase::IDLE;
    _confirm.reset((uint8_t)Phase::IDLE, 0);
}
