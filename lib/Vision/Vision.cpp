#include "Vision.hpp"

// ══════════════════════════════════════════════════════════════════════
//  Constructor
// ══════════════════════════════════════════════════════════════════════
Vision::Vision(Stream &port)
    : _serial(port)
    , _beansSent(false)
    , _benefitsSent(false)
    , _stopSent(false)
    , _beanLeft(0)
    , _beanRight(0)
    , _warmHit(0)
    , _coolHit(0)
    , _boxType(0)
    , _piReady(false)
    , _beansRunning(false)
    , _benefitsRunning(false)
    , _criticalError(false)
    , _separatorError(false)
    , _benefitsError(false)
    , _lastError(0)
{}

// ══════════════════════════════════════════════════════════════════════
//  begin()
// ══════════════════════════════════════════════════════════════════════
void Vision::begin()
    // Reset everything to a clean state.
    // NOTE: We do NOT call _serial.begin() here, the caller handles that
    // since baud rate and port config are projectclevel decisions.
{
    resetGuards();
    _beanLeft  = 0;  
    _beanRight = 0;
    _warmHit   = 0;  
    _coolHit   = 0;
    _boxType   = 0;
    _piReady         = false;
    _beansRunning    = false;
    _benefitsRunning = false;
    clearErrors();
}

// ══════════════════════════════════════════════════════════════════════
//  update() — call every loop()
// ══════════════════════════════════════════════════════════════════════
void Vision::update()
{
    while (_serial.available()) {
        uint8_t b = _serial.read();

        // ── Dispatcher ACKs ──
        if (b == ACK_READY)    { _piReady = true;          continue; }
        if (b == ACK_STARTING) {                            continue; }
        if (b == ACK_RUNNING)  { _beansRunning = true;     continue; }
        if (b == ACK_BENEFITS) { _benefitsRunning = true;  continue; }
        if (b == ACK_STOPPED)  { _beansRunning = false;
                                 _benefitsRunning = false;  continue; }

        // ── Bean vision: [0xFF, left, right] ──
        if (b == HEADER_BEANS) {
            if (_serial.available() >= 2) {
                _beanLeft  = _serial.read();
                _beanRight = _serial.read();
            }
            continue;
        }

        // ── Separator vision: [0xFD, warm, cool] ──
        if (b == HEADER_SEPARATOR) {
            if (_serial.available() >= 2) {
                _warmHit = _serial.read();
                _coolHit = _serial.read();
            }
            continue;
        }

        // ── Box vision: [0xFE, boxType] ──
        if (b == HEADER_BOX) {
            if (_serial.available() >= 1) {
                _boxType = _serial.read();
            }
            continue;
        }

        // ── Errors: [0xE0-0xEF, errorCode] ──
        if (b >= ERROR_MIN && b <= ERROR_MAX) {
            _lastError = b;
            if (_serial.available()) _serial.read();  // consume error code byte

            //  ERROR CLASSIFICATION:
            //
            //  0xE1 (separator died)  → NOT critical. Beans keeps running.
            //                           Separator servo just stays centered.
            //
            //  0xE4 (benefits died)   → NOT critical. Robot keeps moving.
            //                           Benefit servo just stays centered.
            //
            //  0xE0 (raspi_visao died) → CRITICAL. No bean detection.
            //  0xE2 (both died)        → CRITICAL. raspi_visao is gone.
            //  Anything else           → CRITICAL. Unknown = be safe.

            if (b == ERR_SEPARATOR) {
                _separatorError = true;
                // Do NOT touch _beansRunning — raspi_visao is still alive
            }
            else if (b == ERR_BENEFITS) {
                _benefitsError   = true;
                _benefitsRunning = false;
            }
            else {
                // raspi_visao died, or both died, or unknown → CRITICAL
                _criticalError   = true;
                _beansRunning    = false;
                _benefitsRunning = false;
            }
            continue;
        }

        // Unknown byte — ignore
    }
}

// ══════════════════════════════════════════════════════════════════════
//  Commands (Teensy to Pi) — each sends only once until resetGuards()
// ══════════════════════════════════════════════════════════════════════
void Vision::startBeans()
{
    if (!_beansSent) {
        _serial.write(CMD_START_BEANS);
        _beansSent = true;
    }
}

void Vision::startBenefits()
{
    if (!_benefitsSent) {
        _serial.write(CMD_START_BENEFITS);
        _benefitsSent = true;
    }
}

void Vision::stop()
{
    if (!_stopSent) {
        _serial.write(CMD_STOP);
        _stopSent = true;
    }
}

void Vision::requestStatus()
{
    _serial.write(CMD_STATUS);
}

// ══════════════════════════════════════════════════════════════════════
//  Error handling
// ══════════════════════════════════════════════════════════════════════
void Vision::clearErrors()
{
    _criticalError  = false;
    _separatorError = false;
    _benefitsError  = false;
    _lastError      = 0;
}

void Vision::resetGuards()
{
    _beansSent    = false;
    _benefitsSent = false;
    _stopSent     = false;
}