/**
 * @file IR.cpp
 * @date 2026-02-10
 *
 * @brief Implementación de sensores IR de línea
 */

#include "IR.hpp"

// Constructor
IRLine::IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin, bool normal)
    : initialized(false),
      normal(normal)
{
    pins[FL] = flPin;
    pins[FR] = frPin;
    pins[BL] = blPin;
    pins[BR] = brPin;

    for (uint8_t i = 0; i < N; i++)
    {
        rawState[i] = false;
        lineState[i] = false;
    }
}

// Constructor (arreglo de pines)
IRLine::IRLine(const uint8_t pins_[N], bool normal)
    : initialized(false),
      normal(normal)
{
    for (uint8_t i = 0; i < N; i++)
    {
        pins[i] = pins_[i];
        rawState[i] = false;
        lineState[i] = false;
    }
}

bool IRLine::begin()
{
    // Configurar pinMode para cada sensor
    for (uint8_t i = 0; i < N; i++)
    {
        pinMode(pins[i], INPUT);
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
        const bool v = (digitalRead(pins[i]) == HIGH); // true=HIGH, false=LOW

        rawState[i] = v;

        // Interpretación típica (cambiar si la lectura es invertida):
        //Negro suele dar LOW(0)
        // Blanco suele dar HIGH(1)
        lineState[i] = normal ? (!v) : (v);
    }
}

bool IRLine::getRaw(Sensor s) const
{
    return rawState[(uint8_t)s];
}

bool IRLine::getState(Sensor s) const
{
    return lineState[(uint8_t)s];
}

uint8_t IRLine::getPin(Sensor s) const
{
    return pins[(uint8_t)s];
}