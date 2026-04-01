#include "IR_digital.hpp"

IR_digital::IR_digital(const uint8_t pins_[N], uint8_t invertedMask_)
    : invertedMask(invertedMask_), initialized(false)
{
    for (uint8_t i = 0; i < N; i++)
    {
        pins[i] = pins_[i];
        lineState[i] = false;
    }
}

bool IR_digital::begin()
{
    for (uint8_t i = 0; i < N; i++)
    {
        pinMode(pins[i], INPUT);
    }

    initialized = true;
    update();
    return true;
}

void IR_digital::update()
{
    if (!initialized)
        return;

    for (uint8_t i = 0; i < N; i++)
    {
        bool detected = digitalRead(pins[i]);

        const bool isInverted = (invertedMask >> i) & 0x01;
        lineState[i] = isInverted ? (!detected) : detected;
    }
}

bool IR_digital::getState(Sensor s) const
{
    return lineState[(uint8_t)s];
}

void IR_digital::getArray(bool out[N]) const
{
    for (uint8_t i = 0; i < N; i++)
    {
        out[i] = lineState[i];
    }
}

void IR_digital::debugPrint() const
{
    static const char* const labels[N] = {"FL", "FR", "BL", "BR"};

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