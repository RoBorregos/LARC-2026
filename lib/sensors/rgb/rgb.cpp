/**
 * @file rgb.hpp
 * @date 2026-01-28
 *
 * @brief TCS34725 RGB Sensor


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


// Reads the 4 raw channels from the sensor and stores them in the internal "cache".
void RGB::update()
{
    if (!initialized)
        return;

    //Returns in the order: R, G, B, C
    tcs.getRawData(&red, &green, &blue, &clear);
}

void RGB::getRaw(uint16_t out[N_CHANNELS]) const
{
    out[0] = clear;
    out[1] = red;
    out[2] = green;
    out[3] = blue;
}*/