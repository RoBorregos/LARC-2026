/**
 * @file IR.cpp
 * @date 2026-02-10
 *
 * @brief Implementación de sensores IR de línea
 */

#include "IR.hpp"

// Constructor
IRLine::IRLine(bool lineIsBlack_)
    : initialized(false),
      lineIsBlack(lineIsBlack_)
{
    for (uint8_t i = 0; i < N; i++)
    {
        rawState[i] = false;
        lineState[i] = false;
    }
}

bool IRLine::begin()
{
    // Configurar pinMode para cada sensor
    for (uint8_t i = 0; i < N; i++)
    {
        const uint8_t pin = pinFor(static_cast<Index>(i));
        pinMode(pin, INPUT);
    }

    initialized = true;
    update(); // primera lectura
    return true;
}

void IRLine::update()
{
    if (!initialized)
        return;

    for (uint8_t i = 0; i < N; i++)
    {
        const uint8_t pin = pinFor(static_cast<Index>(i));
        const bool v = (digitalRead(pin) == HIGH); // true=HIGH, false=LOW

        rawState[i] = v;

        // Interpretación típica (cambiar si la lectura es invertida):
        //Negro suele dar LOW(0)
        // Blanco suele dar HIGH(1)
        lineState[i] = lineIsBlack ? (!v) : (v);
    }
}

bool IRLine::raw(Index i) const
{
    return rawState[(uint8_t)i];
}

bool IRLine::onLine(Index i) const
{
    return lineState[(uint8_t)i];
}

uint8_t IRLine::pinFor(Index i)
{
    switch (i)
    {
    case FL:
        return Pins::kLineSensorFL;
    case FR:
        return Pins::kLineSensorFR;
    case BL:
        return Pins::kLineSensorBL;
    case BR:
        return Pins::kLineSensorBR;
    default:
        return Pins::kLineSensorFL; // fallback (no debería pasar)
    }
}