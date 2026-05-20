#pragma once
#include <Arduino.h>

/**
 * @file Vision.hpp
 * @brief Interface to the Raspberry Pi vision dispatcher (v4 — bitfield protocol).
 *
 * Vision data is a single byte bitfield (0x00–0x0F), no header:
 *   bit 0 = beanTop        bit 1 = beanBottom
 *   bit 2 = warmBall       bit 3 = coolBall
 *
 * Box data (benefits phase) uses header: 0xFE + 1 byte type
 *
 * Error policy:
 *   0xE0 = raspi_visao died  = critical
 *   0xE1 = separator died    = not critical
 *   0xE2 = both died         = critical (because raspi_visao is dead)
 *   0xE4 = benefits died     = not critical
 *   Anything else 0xE0-0xEF  = treated as critical (unknown = be safe)
 */
class Vision
{
public:
    explicit Vision(Stream &port);
    void begin();

    void update();

    void startBeans();
    void startBenefits();
    void stop();
    void requestStatus();

    bool beanTop()    const { return _bitfield & 0x01; }
    bool beanBottom() const { return _bitfield & 0x02; }
    bool warmBall()   const { return _bitfield & 0x04; }
    bool coolBall()   const { return _bitfield & 0x08; }
    uint8_t bitfield() const { return _bitfield; }

    uint8_t boxType()  const { return _boxType; }
    bool boxDetected() const { return _boxType != 0; }
    bool isRedBox()    const { return _boxType == 1; }
    bool isBlueBox()   const { return _boxType == 2; }

    bool isPiReady()         const { return _piReady; }
    bool isBeansRunning()    const { return _beansRunning; }
    bool isBenefitsRunning() const { return _benefitsRunning; }

    bool hasCriticalError()  const { return _criticalError; }
    bool hasSeparatorError() const { return _separatorError; }
    bool hasBenefitsError()  const { return _benefitsError; }
    bool hasError()          const { return _criticalError || _separatorError || _benefitsError; }
    uint8_t lastError()      const { return _lastError; }

    void clearErrors();
    void resetGuards();

private:
    Stream &_serial;

    bool _beansSent;
    bool _benefitsSent;
    bool _stopSent;

    uint8_t _bitfield;
    uint8_t _boxType;

    bool _piReady;
    bool _beansRunning;
    bool _benefitsRunning;

    bool    _criticalError;
    bool    _separatorError;
    bool    _benefitsError;
    uint8_t _lastError;

    // Parser state for multi-byte messages (errors, box data)
    uint8_t _parseState;
    uint8_t _parseHeader;

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

    static constexpr uint8_t HEADER_BOX   = 0xFE;

    static constexpr uint8_t ERR_RASPI_VISAO = 0xE0;
    static constexpr uint8_t ERR_SEPARATOR   = 0xE1;
    static constexpr uint8_t ERR_BOTH_BEANS  = 0xE2;
    static constexpr uint8_t ERR_BENEFITS    = 0xE4;
    static constexpr uint8_t ERROR_MIN       = 0xE0;
    static constexpr uint8_t ERROR_MAX       = 0xEF;
};