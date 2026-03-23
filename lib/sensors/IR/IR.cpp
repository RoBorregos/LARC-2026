/*
 * @file IR.cpp
 * @date 2026-02-10
 *
 * @brief Implementation of line IR sensors with per-sensor inversion,
 *        hysteresis, and consensus debounce. Direct analog read.
 */

#include "IR.hpp"

// ── Helpers ──────────────────────────────────────────────

static void initArrays(uint8_t n,
                        uint16_t* rawVal, bool* lineState, bool* hysteresisState,
                        bool* stableState, bool* pendingState, uint8_t* debounceCount)
{
    for (uint8_t i = 0; i < n; i++)
    {
        rawVal[i]          = 0;
        lineState[i]       = false;
        hysteresisState[i] = false;
        stableState[i]     = false;
        pendingState[i]    = false;
        debounceCount[i]   = 0;
    }
}

static void initThresholds(uint16_t* threshold, uint16_t* threshHigh, uint16_t* threshLow)
{
    threshold[IRLine::FL] = Constants::IRCalibration::kThreshFL;
    threshold[IRLine::FR] = Constants::IRCalibration::kThreshFR;
    threshold[IRLine::BL] = Constants::IRCalibration::kThreshBL;
    threshold[IRLine::BR] = Constants::IRCalibration::kThreshBR;

    for (uint8_t i = 0; i < IRLine::N; i++)
    {
        threshHigh[i] = threshold[i] + Constants::IRCalibration::kHysteresis;
        threshLow[i]  = threshold[i] - Constants::IRCalibration::kHysteresis;
    }
}

// ── Constructors ─────────────────────────────────────────

IRLine::IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin,
               uint8_t invertedMask)
    : initialized(false), invertedMask(invertedMask)
{
    pins[FL] = flPin;
    pins[FR] = frPin;
    pins[BL] = blPin;
    pins[BR] = brPin;

    initThresholds(threshold, threshHigh, threshLow);
    initArrays(N, rawVal, lineState, hysteresisState,
               stableState, pendingState, debounceCount);
}

// ── Initialization ───────────────────────────────────────

bool IRLine::begin()
{
    for (uint8_t i = 0; i < N; i++)
        pinMode(pins[i], INPUT);

    initialized = true;
    update();
    return true;
}

// Update (analog read + threshold + hysteresis + debounce)

void IRLine::update()
{
    if (!initialized) return;

    for (uint8_t i = 0; i < N; i++)
    {
        rawVal[i] = analogRead(pins[i]);

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

        // --- Consensus debounce ---
        if (hysteresisState[i] == stableState[i])
        {
            pendingState[i]  = stableState[i];
            debounceCount[i] = 0;
        }
        else if (hysteresisState[i] == pendingState[i])
        {
            debounceCount[i]++;
            if (debounceCount[i] >= Constants::IRCalibration::kDebounceCount)
            {
                stableState[i]   = pendingState[i];
                debounceCount[i] = 0;
            }
        }
        else
        {
            pendingState[i]  = hysteresisState[i];
            debounceCount[i] = 1;
        }
    }
}

// ── Getters ──────────────────────────────────────────────

bool IRLine::getState(Sensor s) const
{
    return stableState[(uint8_t)s];
}

void IRLine::getArray(bool out[N]) const
{
    for (uint8_t i = 0; i < N; i++)
        out[i] = stableState[i];
}

// ── Debug ────────────────────────────────────────────────

void IRLine::debugPrint() const
{
    static const char* const labels[N] = { "FL", "FR", "BL", "BR" };

    Serial.print(F("RAW        → "));
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(labels[i]); Serial.print(':');
        Serial.print(rawVal[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("THRESHOLD  → "));
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(labels[i]); Serial.print(':');
        Serial.print(lineState[i] ? '1' : '0');
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("HYSTERESIS → "));
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(labels[i]); Serial.print(':');
        Serial.print(hysteresisState[i] ? '1' : '0');
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("STABLE     → "));
    for (uint8_t i = 0; i < N; i++) {
        Serial.print(labels[i]); Serial.print(':');
        Serial.print(stableState[i] ? '1' : '0');
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();
}