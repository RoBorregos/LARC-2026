/**
 * @file IRLine.hpp
 * @date 2026-02-22
 *
 * @brief Sensores IR de línea analógicos a través de un 74HC4067.
 *        Cada sensor tiene su propio canal en el mux y su propio umbral.
 *        Si la lectura supera el umbral = línea detectada (true).
 */

#ifndef IRLINE_HPP
#define IRLINE_HPP

#include <Arduino.h>
#include "mux.h"

class IR_mux

{
public:
    // Índices de sensores
    enum Sensor : uint8_t
    {
        FL = 0, // Front-Left
        FR = 1, // Front-Right
        BL = 2, // Back-Left
        BR = 3, // Back-Right
        N  = 4
    };

    IR_mux(Mux74HC4067& mux_, const uint8_t channels_[N],
           uint8_t invertedMask = 0b0000);


    // Configs mux and does first reading
    bool begin();

    // Reads raw values of all sensors
    void update();

    //Rrturns the logic reading of sensors (after palying inversion)
    //true = Line
    //false = No line
    bool getState(Sensor s) const;
    
    void getArray(bool out[N]) const;

    //Hysteresis filtered state
    void debugPrint() const;

private:
    Mux74HC4067& mux;

    //Channels assigned to each sensor on the mux (defined in constructor)
    uint8_t  channels[N];

    // Analog thresholds for each sensor (defined in constructor or constants.h)
    uint16_t threshold[N];

    uint8_t  invertedMask;

    bool     initialized;
    bool     lineState[N]; // State after applying simple threshold
    bool     hysteresisState[N]; // State after applying hysteresis

    uint16_t threshHigh[N];
    uint16_t threshLow[N];

    uint16_t rawVal[N]; // ADC raw value (0..1023)
    uint16_t stableState[N];    // center + kHysteresis
    uint16_t pendingState[N];
    uint8_t     debounceCount[N];
};

#endif // IRLINE_HPP