/**
 * @file bno.cpp
 * @date 2026-01-28
 *
 * @brief Implementación del driver BNO055 (I2C)
*/

#include "bno.hpp"
#include <math.h> // fmodf

BNO::BNO(uint8_t address)
    : bno(55, address, &Wire), initialized(false)
{
}

bool BNO::begin()
{
    // Intento simple de inicio (sin while infinito)
    if (!bno.begin())
    {
        initialized = false;
        return false;
    }

    delay(1000);
    //si tiene cristal externo entonces se usa
    // si el hardware no lo encuentra, no pasa nada
    bno.setExtCrystalUse(true); 

    initialized = true;

    // Primer update para llenar el cache
    update();
    return true;
}

void BNO::update()
{
    if (!initialized)
        return;

    // Angulos de Euler (nomás el yaw)
    bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER);
}

float BNO::wrapAngle(float deg) const
{
    // Normaliza a [0, 360)
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f)
        deg += 360.0f;

    // Luego a [-180, 180]
    if (deg > 180.0f)
        deg -= 360.0f;

    return deg;
}

float BNO::getYawWrapped() const
{
    // Devuelve yaw en rango [-180, 180]
    return -wrapAngle(orientEvent.orientation.x);
}

float BNO::getYaw360() const
{
    // Devuelve yaw en rango [0, 360)
    float y = orientEvent.orientation.x;
    y = fmodf(y, 360.0f);
    if (y < 0.0f) y += 360.0f;
    return y;
}

void BNO::getLinearAcceleration(float out[N_ACCEL])
{
    if (!initialized)
    {
        out[0] = out[1] = out[2] = 0.0f;
        return;
    }

    sensors_event_t e;
    bno.getEvent(&e, Adafruit_BNO055::VECTOR_LINEARACCEL);

    out[0] = e.acceleration.x;
    out[1] = e.acceleration.y;
    out[2] = e.acceleration.z;
}