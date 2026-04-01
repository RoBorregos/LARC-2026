/**
 * @file IRLine.cpp
 * @date 2026-02-22
 *
 * @brief Implementation of the IR sensors using analog read with thresholds and per-sensor inversion.
 *        Through a 74HC4067.
 */

#include "IR_mux.hpp"
#include "constants.h"
#include "pins.h"

// Constructor

IR_mux::IR_mux(Mux74HC4067& mux_, const uint8_t channels_[N],
               uint8_t invertedMask_)
    : mux(mux_), invertedMask(invertedMask_), initialized(false)
{
    threshold[FL] = Constants::IRCalibration::kThreshFL;
    threshold[FR] = Constants::IRCalibration::kThreshFR;
    threshold[BL] = Constants::IRCalibration::kThreshBL;
    threshold[BR] = Constants::IRCalibration::kThreshBR;

    for (uint8_t i = 0; i < N; i++)
    {
        channels[i]        = channels_[i];
        rawVal[i]          = 0;
        lineState[i]       = false;
        hysteresisState[i] = false;

        threshHigh[i] = threshold[i] + Constants::IRCalibration::kHysteresis;
        threshLow[i]  = threshold[i] - Constants::IRCalibration::kHysteresis;

        stableState[i]   = false;
        pendingState[i]  = false;
        debounceCount[i] = 0;
    }
}

//Initialization

bool IR_mux::begin()
{
    mux.begin();

    initialized = true;
    update(); // First reading
    return true;
}

// Reading of all sensors

void IR_mux::update()
{
    if (!initialized) return;

    for (uint8_t i = 0; i < N; i++)
    {
        rawVal[i] = mux.read(channels[i]);

        // --- Simple threshold ---
        const bool detected   = (rawVal[i] >= threshold[i]);
        const bool isInverted = (invertedMask >> i) & 0x01;
        lineState[i] = isInverted ? !detected : detected;

        // --- Hysteresis ---
        bool hDetected;
        if      (rawVal[i] >= threshHigh[i]) hDetected = true;
        else if (rawVal[i] <  threshLow[i])  hDetected = false;
        else                                  hDetected = hysteresisState[i];

        hysteresisState[i] = isInverted ? !hDetected : hDetected;

        // --- Debouce ---
        if (hysteresisState[i] == stableState[i])
        {
            // Reading agrees with committed state — reset any pending transition
            pendingState[i]  = stableState[i];
            debounceCount[i] = 0;
        }
        else if (hysteresisState[i] == pendingState[i])
        {
            // Reading agrees with the candidate — advance counter
            debounceCount[i]++;
            if (debounceCount[i] >= Constants::IRCalibration::kDebounceCount)
            {
                // Enough consecutive agreeing reads — commit the change
                stableState[i]   = pendingState[i];
                debounceCount[i] = 0;
            }
        }
        else
        {
            // New candidate value seen — start tracking it from 1
            pendingState[i]  = hysteresisState[i];
            debounceCount[i] = 1;
        }
    }
}

// Getters

bool IR_mux::getState(Sensor s) const
{
    return stableState[(uint8_t)s];
}

void IR_mux::getArray(bool out[N]) const
{
    for (uint8_t i = 0; i < N; i++)
        out[i] = stableState[i];  // was lineState, now uses stable output
}

void IR_mux::debugPrint() const
{
    static const char* const labels[N] = { "FL", "FR", "BL", "BR" };

    // Row 1 — raw ADC
    Serial.print(F("RAW        → "));
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(labels[i]); Serial.print(':');
        Serial.print(rawVal[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    // Row 2 — simple threshold
    Serial.print(F("THRESHOLD  → "));
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(labels[i]); Serial.print(':');
        Serial.print(lineState[i] ? '1' : '0');
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    // Row 3 — hysteresis
    Serial.print(F("HYSTERESIS → "));
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(labels[i]); Serial.print(':');
        Serial.print(hysteresisState[i] ? '1' : '0');
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();
}