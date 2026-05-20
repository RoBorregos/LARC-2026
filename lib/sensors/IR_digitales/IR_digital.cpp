#include "IR_digital.hpp"

IR_mux::IR_mux(const uint8_t pins_[N], uint8_t invertedMask_)
    : invertedMask(invertedMask_), initialized(false)
{
    for (uint8_t i = 0; i < N; i++)
    {
        pins[i] = pins_[i];
        rawVal[i] = 0;
        lineState[i] = false;
    }
}

bool IR_mux::begin()
{
    for (uint8_t i = 0; i < N; i++)
    {
        pinMode(pins[i], INPUT);
        // Si se comportan mejor así, prueba:
        // pinMode(pins[i], INPUT_PULLUP);
    }

    initialized = true;
    update();
    return true;
}

void IR_mux::update()
{
    if (!initialized) return;

    for (uint8_t i = 0; i < N; i++)
    {
        rawVal[i] = digitalRead(pins[i]);

        const bool detected = (rawVal[i] != 0);
        const bool isInverted = (invertedMask >> i) & 0x01;

        lineState[i] = isInverted ? !detected : detected;
    }
}

bool IR_mux::getState(Sensor s) const
{
    return lineState[(uint8_t)s];
}

void IR_mux::getArray(bool out[N]) const
{
    for (uint8_t i = 0; i < N; i++)
        out[i] = lineState[i];
}

void IR_mux::debugPrint() const
{
    static const char* const labels[N] = {"FL", "FR", "BL", "BR"};

    Serial.print(F("RAW/DIGITAL → "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(labels[i]);
        Serial.print(':');
        Serial.print(rawVal[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("STATE       → "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(labels[i]);
        Serial.print(':');
        Serial.print(lineState[i] ? '1' : '0');
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();
}