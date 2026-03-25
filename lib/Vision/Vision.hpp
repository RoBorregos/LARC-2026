#pragma once
#include <Arduino.h>

/**
 * @file Vision.hpp
 * @brief Interface to the Raspberry Pi vision dispatcher.
 *
 * Handles the serial protocol between Teensy and Pi dispatcher:
 *   - Sending commands (start beans, start benefits, stop, status)
 *   - Receiving ACKs, vision data, and errors
 *   - Simple getters for vision results
 *
 * Usage:
 *   Vision vision;             // uses Serial by default
 *   vision.begin();           // wait for vision.isPiReady() before starting
 *   vision.startBeans();       // launch bean detection scripts
 *   vision.update();            // call every loop()
 *   if (vision.beanLeft()) { }  // check detections
 *   vision.stop();            // kill all scripts on Pi
 */
class Vision
{
public:
    // ── Constructor ──────────────────────────────────────────
    // Pass any serial port (USB Serial, Serial1, etc.)
    explicit Vision(Stream &port);

    /// Call once in setup(). Does NOT open the serial port, we handle Serial.begin() ourselves.
    void begin();

    // ── Update (call every loop) ─────────────────────────────
    /// Reads and parses all available bytes from the Pi.
    void update();

    // ── Commands (Teensy to Pi) ───────────────────────────────
    /// Launch raspi_visao.py and separator_visao.py. Safe to call multiple times, only sends command once.
    void startBeans();

    /// Launch benefits.py (auto stops beans on Pi side). Safe to call multiple times likewise
    void startBenefits();

    /// Kill all running scripts on the Pi. Safe to call multiple times.
    void stop();

    /// Request status from dispatcher.
    void requestStatus();

    // ── Bean vision data ─────────────────────────────────────
    /// Returns true if a bean is detected on the left (bottom camera side).
    bool beanBottom()  const { return _beanLeft != 0; }

    /// Returns true if a bean is detected on the right (top camera side).
    bool beanTop() const { return _beanRight != 0; }

    /// Raw left value (0 or 1).
    uint8_t beanLeftRaw()  const { return _beanLeft; }

    /// Raw right value (0 or 1).
    uint8_t beanRightRaw() const { return _beanRight; }

    // ── Separator vision data ────────────────────────────────
    /// Returns true if a warm colored ball was detected.
    bool warmBall() const { return _warmHit != 0; }

    /// Returns true if a cool colored ball was detected.
    bool coolBall() const { return _coolHit != 0; }

    // ── Box vision data (benefits phase) ─────────────────────
    /// Box type: 0 = none, 1 = red, 2 = blue.
    uint8_t boxType() const { return _boxType; }

    bool boxDetected() const { return _boxType != 0; }
    bool isRedBox()    const { return _boxType == 1; }
    bool isBlueBox()   const { return _boxType == 2; }

    // ── Dispatcher status ────────────────────────────────────
    bool isPiReady()         const { return _piReady; }
    bool isBeansRunning()    const { return _beansRunning; }
    bool isBenefitsRunning() const { return _benefitsRunning; }
    bool hasError()          const { return _errorFlag; }
    uint8_t lastError()      const { return _lastError; }

    /// Clear the error flag after handling it.
    void clearError() { _errorFlag = false; _lastError = 0; }

    // ── Reset ────────────────────────────────────────────────
    /// Reset all command guards so you can re-send commands 
    //(e.g. after a state machine restart or if we want to stop/start 
    //vision multiple times throughout the state machine).
    void resetGuards();

private:
    Stream &_serial;

    // Command-sent guards (prevent repeated sends)
    bool _beansSent;
    bool _benefitsSent;
    bool _stopSent;

    // Vision data
    uint8_t _beanLeft;
    uint8_t _beanRight;
    uint8_t _warmHit;
    uint8_t _coolHit;
    uint8_t _boxType;

    // Dispatcher status
    bool _piReady;
    bool _beansRunning;
    bool _benefitsRunning;

    // Error tracking
    bool    _errorFlag;
    uint8_t _lastError;

    // ── Protocol bytes ───────────────────────────────────────
    static constexpr uint8_t CMD_START_BEANS    = 0xA0;
    static constexpr uint8_t CMD_STOP           = 0xA1;
    static constexpr uint8_t CMD_STATUS         = 0xA2;
    static constexpr uint8_t CMD_START_BENEFITS = 0xA3;

    static constexpr uint8_t ACK_READY    = 0xB0;
    static constexpr uint8_t ACK_STARTING = 0xB1;
    static constexpr uint8_t ACK_RUNNING  = 0xB2;
    static constexpr uint8_t ACK_STOPPED  = 0xB3;
    static constexpr uint8_t ACK_BENEFITS = 0xB4;

    static constexpr uint8_t HEADER_BEANS     = 0xFF;  // [0xFF, left, right]
    static constexpr uint8_t HEADER_SEPARATOR = 0xFD;  // [0xFD, warm, cool]
    static constexpr uint8_t HEADER_BOX       = 0xFE;  // [0xFE, boxType]

    static constexpr uint8_t ERROR_MIN = 0xE0;
    static constexpr uint8_t ERROR_MAX = 0xEF;
};