#include "qtr.hpp"
#include "constants.h"

// 74HC4067
// Wiring (per schematic):
//   SIG - A0
//   S0 - pin 26
//   S1 - pin 27
//   S2 - pin 28
//   S3 - pin 29
//   EN - GND (siempre enabled)

static constexpr bool LINE_IS_BLACK = false;

QTR::QTR(uint8_t firstChannel, Mux74HC4067& mux_)
    : firstCh(firstChannel), initialized(false), mux(mux_), position(0)
{
    for (uint8_t i = 0; i < N; i++)
    {
        raw[i]    = 0;
        calMin[i] = 0;
        calMax[i] = 1;
        norm[i]   = 0;
    }
}

bool QTR::begin()
{
    mux.begin();

    // Seleccionar el primer canal de este array
    mux.select(firstCh);

    initialized = true;
    return true;
}

void QTR::ensureCalValid()
{
    for (uint8_t i = 0; i < N; i++)
    {
        if (calMax[i] <= calMin[i])
            calMax[i] = calMin[i] + 1;
    }
}

void QTR::setCalibration(const uint16_t* minVals, const uint16_t* maxVals)
{
    for (uint8_t i = 0; i < N; i++)
    {
        calMin[i] = minVals[i];
        calMax[i] = maxVals[i];
    }
    ensureCalValid();
}

void QTR::useDefaultCalibration(uint8_t profile)
{
    using namespace Constants::QTRCalibration;

    switch (profile)
    {
    case 1: // REAR
        setCalibration(Rear.min, Rear.max);
        break;
    case 0: // FRONT
    default:
        setCalibration(Front.min, Front.max);
        break;
    }
}

void QTR::update()
{
    if (!initialized)
        return;

    // 1) Leer raw (ADC)
    for (uint8_t i = 0; i < N; i++)
        raw[i] = mux.read(firstCh + i);

    // 2) Normalizar a 0..1000 usando calMin/calMax
    for (uint8_t i = 0; i < N; i++)
    {
        const long x = (long)(raw[i] - calMin[i]) * 1000L;
        const long d = (long)(calMax[i] - calMin[i]);
        long v = (d > 0) ? (x / d) : 0;

        if (v < 0)   v = 0;
        if (v > 1000) v = 1000;
        norm[i] = (uint16_t)v;
    }

    // 3) Calcular posición tipo Pololu (0 a 7000) con el promedio ponderado
    uint32_t sum      = 0;
    uint32_t weighted = 0;

    for (uint8_t i = 0; i < N; i++)
    {
        const uint16_t v = norm[i];
        sum      += v;
        weighted += (uint32_t)v * (uint32_t)(i * 1000);
    }

    if (sum == 0)
    {
        // No se detectó línea (o muy poca señal). se conserva la última posición.
        return;
    }

    position = (int)(weighted / sum); // 0 a 7000
}

int QTR::getPosition() const
{
    return position;
}

bool QTR::onLine(uint16_t threshold) const
{
    uint16_t maxv = 0;
    for (uint8_t i = 0; i < N; i++)
        if (norm[i] > maxv)
            maxv = norm[i];

    return maxv > threshold;
}

void QTR::debugPrint() const
{
    Serial.print(F("RAW : "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(raw[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("NORM: "));
    for (uint8_t i = 0; i < N; i++)
    {
        Serial.print(norm[i]);
        if (i < N - 1) Serial.print('\t');
    }
    Serial.println();

    Serial.print(F("POS : "));
    Serial.println(position);
}