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
    : mux(mux_),
      invertedMask(invertedMask_),
      initialized(false)
{
    for (uint8_t i = 0; i < N; i++)
    {
        channels[i]  = channels_[i];
        rawVal[i]    = 0;
        lineState[i] = false;
    }

    // Loads thrtesholds from constants.h
    // To change the thresholds, modify Constants::IRCalibration in constants.h
    threshold[FL] = Constants::IRCalibration::kThreshFL;
    threshold[FR] = Constants::IRCalibration::kThreshFR;
    threshold[BL] = Constants::IRCalibration::kThreshBL;
    threshold[BR] = Constants::IRCalibration::kThreshBR;
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
    if (!initialized)
        return;

    for (uint8_t i = 0; i < N; i++)
    {
        
        // Reads the corresponding channel for this sensor on the mux
        rawVal[i] = mux.read(channels[i]);

        // If the raw value exceeds the threshold → line detected (before inversion)
        const bool detected = (rawVal[i] >= threshold[i]);

        //Applym logic inversion if the corresponding bit is active
        const bool isInverted = (invertedMask >> i) & 0x01;
        lineState[i] = isInverted ? (!detected) : detected;
    }
}

// Getters

bool IR_mux::getState(Sensor s) const
{
    return lineState[(uint8_t)s];
}

uint16_t IR_mux::getRaw(Sensor s) const
{
    return rawVal[(uint8_t)s];
}

void IR_mux::debugPrint() const
{

    static const char* const labels[N] = { "FL", "FR", "BL", "BR" };

    Serial.print(F("IR RAW  → "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(labels[i]);
        Serial.print(':');
        Serial.print(rawVal[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("IR LINE → "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(labels[i]);
        Serial.print(':');
        Serial.print(lineState[i] ? '1' : '0');
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();
}
