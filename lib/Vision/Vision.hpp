#pragma once
#include <Arduino.h>

/**
 * @file Vision.hpp
 * @brief Interface to the Raspberry Pi vision dispatcher.
 *
 * Error policy:
 * raspi_visao crashes (any reason including camera) = CRITICAL = stop robot
 * separator crashes (any reason including camera) = NOT critical = keep beans running
 * benefits crashes (any reason including camera) = NOT critical = robot keeps moving (will manage when to stop later)
 *
 * The dispatcher sends script-specific error bytes:
 *   0xE0 = raspi_visao died    = critical
 *   0xE1 = separator died      = not critical
 *   0xE2 = both died           = critical (because raspi_visao is dead)
 *   0xE4 = benefits died       = not critical
 *   Anything else              = treated as critical (unknown = be safe)
 * 
 * Usage example (in the state machine):
 * 
case LARC_STATE::BEANS:
{
    if (vision.hasCriticalError()) {
        vision.stop();
        setMainState(LARC_STATE::STOP);
        break;
    }
}

 */
class Vision
{
public:
    explicit Vision(Stream &port);
    void begin();

    // ── Update (call every loop) ─────────────────────────────
    void update();

    // ── Commands (Teensy to Pi) ───────────────────────────────
    void startBeans();
    void startBenefits();
    void stop();
    void requestStatus();

    // ── Bean vision data ─────────────────────────────────────
    bool beanBottom()      const { return _beanLeft != 0; }
    bool beanTop()         const { return _beanRight != 0; }
    uint8_t beanLeftRaw()  const { return _beanLeft; }
    uint8_t beanRightRaw() const { return _beanRight; }

    // ── Separator vision data ────────────────────────────────
    bool warmBall() const { return _warmHit != 0; }
    bool coolBall() const { return _coolHit != 0; }

    // ── Box vision data (benefits phase) ─────────────────────
    uint8_t boxType()  const { return _boxType; }
    bool boxDetected() const { return _boxType != 0; }
    bool isRedBox()    const { return _boxType == 1; }
    bool isBlueBox()   const { return _boxType == 2; }

    // ── Dispatcher status ────────────────────────────────────
    bool isPiReady()         const { return _piReady; }
    bool isBeansRunning()    const { return _beansRunning; }
    bool isBenefitsRunning() const { return _benefitsRunning; }

    // ── Error handling ───────────────────────────────────────
    // CRITICAL = raspi_visao is dead → bean detection gone → STOP

    bool hasCriticalError() const { return _criticalError; }

    // NON-CRITICAL = separator died → beans still works, just no sorting

    bool hasSeparatorError() const { return _separatorError; }

    // NON-CRITICAL = benefits died → robot keeps moving

    bool hasBenefitsError() const { return _benefitsError; }

    // Any error at all

    bool hasError() const { return _criticalError || _separatorError || _benefitsError; }

    uint8_t lastError() const { return _lastError; }
    void clearErrors();

    // ── Reset ────────────────────────────────────────────────

    void resetGuards();

private:
    Stream &_serial;

    bool _beansSent;
    bool _benefitsSent;
    bool _stopSent;

    uint8_t _beanLeft;
    uint8_t _beanRight;
    uint8_t _warmHit;
    uint8_t _coolHit;
    uint8_t _boxType;

    bool _piReady;
    bool _beansRunning;
    bool _benefitsRunning;

    bool    _criticalError;
    bool    _separatorError;
    bool    _benefitsError;
    uint8_t _lastError;

    // Protocol bytes
    static constexpr uint8_t CMD_START_BEANS    = 0xA0;
    static constexpr uint8_t CMD_STOP           = 0xA1;
    static constexpr uint8_t CMD_STATUS         = 0xA2;
    static constexpr uint8_t CMD_START_BENEFITS = 0xA3;

    static constexpr uint8_t ACK_READY    = 0xB0;
    static constexpr uint8_t ACK_STARTING = 0xB1;
    static constexpr uint8_t ACK_RUNNING  = 0xB2;
    static constexpr uint8_t ACK_STOPPED  = 0xB3;
    static constexpr uint8_t ACK_BENEFITS = 0xB4;

    static constexpr uint8_t HEADER_BEANS     = 0xFF;
    static constexpr uint8_t HEADER_SEPARATOR = 0xFD;
    static constexpr uint8_t HEADER_BOX       = 0xFE;

    static constexpr uint8_t ERR_RASPI_VISAO = 0xE0;
    static constexpr uint8_t ERR_SEPARATOR   = 0xE1;
    static constexpr uint8_t ERR_BOTH_BEANS  = 0xE2;
    static constexpr uint8_t ERR_BENEFITS    = 0xE4;
    static constexpr uint8_t ERROR_MIN       = 0xE0;
    static constexpr uint8_t ERROR_MAX       = 0xEF;
};