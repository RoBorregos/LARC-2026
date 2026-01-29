/**
 * @file rgb.hpp
 * @date 2026-01-28
 *
 * @brief TCS34725 sensor RGB
*/

#include "rgb.hpp"

// Constructor:

RGB::RGB(decltype(TCS34725_INTEGRATIONTIME_50MS) it,
         decltype(TCS34725_GAIN_4X) gain)
    : tcs(it, gain),
      initialized(false),
      clear(0),
      red(0),
      green(0),
      blue(0)
{
}

// begin():
//se ntenta detectar/inicializar el sensor por I2C.
// Si no lo encuentra, regresa false.
bool RGB::begin()
{
    if (!tcs.begin())
    {
        initialized = false;
        return false;
    }

    initialized = true;
    return true;
}

// update():
// Lee los 4 canales crudos del sensor y los guarda en el "cache" interno.
void RGB::update()
{
    if (!initialized)
        return;

    //devuelve en el orden: R, G, B, C
    tcs.getRawData(&red, &green, &blue, &clear);
}

void RGB::getRaw(uint16_t out[N_CHANNELS]) const
{
    out[0] = clear;
    out[1] = red;
    out[2] = green;
    out[3] = blue;
}