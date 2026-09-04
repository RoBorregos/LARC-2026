/**
   @file VisionLink.hpp
   @date 2026-08-19
  
   @brief The whole Orin link, behind five verbs.
  
    This is the ONLY class the state machine talks to about vision. It owns
    every piece of the protocol so nothing else has to know one exists:
  
    reading the serial port
    framing, CRC-8, version and phase validation, reserved-bit checks
    sequence numbers and duplicate rejection
    the single confirmation filter (CommandConfirm)
    the link watchdog and the safe state it forces
    phase gating: which actuators a stage may address at all
    driving ServoSystem
    the four uplink request bytes

    Typical use:
    ServoSystem servos;
    VisionLink  vision(Serial, servos);   // USB link to the Orin
  
    void setup() { servos.begin(); vision.begin(); }
    void loop()  { vision.update(); } // drives the servos for you
  
    update() must be called every loop. just consumes
    whatever bytes have arrived, applies whatever became confirmed, runs
    the watchdog, and services the benefit door timers.
 */

#ifndef VISION_LINK_HPP
#define VISION_LINK_HPP

#include <Arduino.h>

#include "constants.h"
#include "CommandConfirm.hpp"
#include "VisionProtocol.hpp"
#include "ServoSystem.hpp"

class VisionLink
{
public:
    static_assert(Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES >= 1 &&
                  Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES <= 3,
                  "Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES must be 1, 2 or 3");

    VisionLink(Stream &port, ServoSystem &servos,
               uint32_t timeoutMs = Constants::VisionConfig::kLinkTimeoutMs);

    // put the actuators in the
    // safe state with both benefit doors closed.
    void begin();

    /// Call every loop. Consumes serial, applies confirmed commands, runs
    /// the watchdog, services the benefit timers. Never blocks.
    void update();

    // Phase requests (one byte out, guarded against repeats)
    void startBeans();
    void startBenefits();
    void stop();
    void requestStatus();

    // Re arm the request guards so the next start*/stop() transmits again.
    void resetGuards();

    // Status 

    // The phase the Teensy is currently acting on (not the requested one).
    VisionProto::Phase phase() const { return _appliedPhase; }
    bool inBeansPhase()        const { return _appliedPhase == VisionProto::Phase::BEANS; }
    bool inBenefitsPhase()     const { return _appliedPhase == VisionProto::Phase::BENEFITS; }

    // False once the watchdog has fired, true again on the next valid frame.
    bool isLinkUp() const { return _link.linkUp(); }

    // Live flags from the STATUS byte of the last accepted frame.
    bool isOrinReady()         const;
    bool isBeansRunning()    const;
    bool isBenefitsRunning() const;

    // They stay set until clearErrors().
    bool hasCriticalError()  const { return _criticalError; }
    bool hasSeparatorError() const { return _separatorError; }
    bool hasBenefitsError()  const { return _benefitsError; }
    bool hasError()          const { return _criticalError || _separatorError || _benefitsError; }
    uint8_t lastStatus()     const { return _status; }
    void clearErrors();

    // Debug
    const VisionProto::LinkStats &stats() const { return _link.stats(); }

    uint8_t confirmRemaining() const
    {
        return _confirm.remaining(Constants::VisionConfig::REQUIRED_CONFIRMATION_FRAMES);
    }
    bool lastSeparatorInvalid() const { return _separatorInvalid; }

private:
    Stream                   &_serial;
    ServoSystem              &_servos;
    VisionProto::LinkManager  _link;
    CommandConfirm            _confirm;

    VisionProto::Phase _appliedPhase;
    uint8_t            _status;
    bool               _separatorInvalid;

    bool _criticalError;
    bool _separatorError;
    bool _benefitsError;

    bool _beansSent;
    bool _benefitsSent;
    bool _stopSent;

    void _applySafetyImmediate(const VisionProto::Command &cmd);

    void _applyCommand(const VisionProto::Command &cmd);

    // Everything safe, filters reset, phase back to IDLE.
    void _goSafe();

    void _latchFaults(uint8_t status);
};

#endif
