
#include "qtr.hpp"

// 74HC4067
// Wiring (per schematic):
//   SIG - A0
//   S0 - pin 26
//   S1 - pin 27
//   S2 - pin 28
//   S3 - pin 29
//   EN - GND (siempre enabled)
static constexpr uint8_t MUX_SIG = A0;
static constexpr uint8_t MUX_S0  = 26;
static constexpr uint8_t MUX_S1  = 27;
static constexpr uint8_t MUX_S2  = 28;
static constexpr uint8_t MUX_S3  = 29;

static constexpr bool LINE_IS_BLACK = true;

// Perfiles de calibración (PLACEHOLDERS)
// Profile 0: FRONT
static const uint16_t FRONT_MIN[QTR::N] = {120, 130, 115, 140, 150, 135, 128, 122};
static const uint16_t FRONT_MAX[QTR::N] = {3100, 3200, 3050, 3300, 3350, 3250, 3150, 3000};

// Profile 1: REAR
static const uint16_t REAR_MIN[QTR::N]  = {120, 130, 115, 140, 150, 135, 128, 122};
static const uint16_t REAR_MAX[QTR::N]  = {3100, 3200, 3050, 3300, 3350, 3250, 3150, 3000};

// helpers del mux
static inline void muxSelect(uint8_t ch)
{
    digitalWrite(MUX_S0, (ch >> 0) & 1);
    digitalWrite(MUX_S1, (ch >> 1) & 1);
    digitalWrite(MUX_S2, (ch >> 2) & 1);
    digitalWrite(MUX_S3, (ch >> 3) & 1);
}

static bool s_muxPinsInitialized = false;
static inline void initMuxPinsOnce()
{
    if (s_muxPinsInitialized)
        return;

    pinMode(MUX_SIG, INPUT);
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);
    muxSelect(0);

    s_muxPinsInitialized = true;
}

QTR::QTR(uint8_t firstChannel)
    : firstCh(firstChannel), initialized(false), position(0)
{
    for (uint8_t i = 0; i < N; i++)
    {
        raw[i] = 0;
        calMin[i] = 0;
        calMax[i] = 1;
        norm[i] = 0;
    }
}

bool QTR::begin()
{
    initMuxPinsOnce();

    // Select this array's first channel (nice for sanity)
    muxSelect(firstCh);

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

void QTR::setCalibration(const uint16_t (&minVals)[N], const uint16_t (&maxVals)[N])
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
    // Nota: Si por alguna razón llaman profile "equivocado",
    // sigue funcionando; solo cambia qué rangos usa para normalizar.
    switch (profile)
    {
    case 1: // REAR
        setCalibration(REAR_MIN, REAR_MAX);
        break;
    case 0: // FRONT
    default:
        setCalibration(FRONT_MIN, FRONT_MAX);
        break;
    }
}

void QTR::update()
{
    if (!initialized)
        return;

    // 1) Leer raw (ADC)
    for (uint8_t i = 0; i < N; i++)
    {
        muxSelect(firstCh + i);
        delayMicroseconds(5);
        raw[i] = analogRead(MUX_SIG);
    }

    // 2) Normalizar a 0..1000 usando calMin/calMax
    for (uint8_t i = 0; i < N; i++)
    {
        const long x = (long)(raw[i] - calMin[i]) * 1000L;
        const long d = (long)(calMax[i] - calMin[i]);
        long v = (d > 0) ? (x / d) : 0;

        if (v < 0)
            v = 0;
        if (v > 1000)
            v = 1000;
        norm[i] = (uint16_t)v;
    }

    // 3) Calcular posición tipo Pololu (0 a 7000) con el promedio ponderado
    uint32_t sum = 0;
    uint32_t weighted = 0;

    for (uint8_t i = 0; i < N; i++)
    {
        const uint16_t v = norm[i];
        sum += v;
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