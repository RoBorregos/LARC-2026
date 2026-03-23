#pragma once

/*
 * @file IR.hpp
 * @date 2026-02-10
 *
 * @brief Line IR sensors with per-sensor inversion, hysteresis, and debounce.
 *        Direct analog read from Teensy pins.
 */

#include <Arduino.h>
#include "constants.h"

class IRLine
{
public:
    // Sensor Index
    enum Sensor : uint8_t
    {
        FL = 0, // Front-Left
        FR = 1, // Front-Right
        BL = 2, // Back-Left
        BR = 3, // Back-Right
        N  = 4
    };

    //Constructor
    IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin,
        uint8_t invertedMask = 0b0000);

    // Configs pins and does first reading
    bool begin();

    // Reads raw values of all sensors
    void update();

    // Returns the debounced + hysteresis state (after inversion)
    // true = Line detected
    // false = No line
    bool getState(Sensor s) const;

    // Fill array with all debounced states
    void getArray(bool out[N]) const;

    // Debug: prints raw ADC, threshold, hysteresis, and stable states
    void debugPrint() const;

private:
    bool     initialized;
    uint8_t  pins[N];
    uint8_t  invertedMask;

    uint16_t rawVal[N];    // ADC raw value (0..1023)
    bool     lineState[N];   // State after simple threshold
    bool     hysteresisState[N];  // State after hysteresis

    uint16_t threshold[N];
    uint16_t threshHigh[N];  // threshold + kHysteresis
    uint16_t threshLow[N];    // threshold - kHysteresis
 
    // Consensus debounce
    bool     stableState[N];   // Committed state
    bool     pendingState[N];  // Candidate state
    uint8_t  debounceCount[N];
};