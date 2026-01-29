/**
 * @file rgb.hpp
 * @date 2026-01-28
 *
 * @brief TCS34725 RGB sensor (I2C)
 */

#ifndef RGB_HPP
#define RGB_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

class RGB
{
public:
    static constexpr uint8_t N_CHANNELS = 4;

    // Constructor: usa las mismas constantes que los ejemplos de Adafruit
    RGB(decltype(TCS34725_INTEGRATIONTIME_50MS) it = TCS34725_INTEGRATIONTIME_50MS,
        decltype(TCS34725_GAIN_4X) gain = TCS34725_GAIN_4X);

    bool begin();
    void update();

    // out[0]=Clear, out[1]=Red, out[2]=Green, out[3]=Blue
    void getRaw(uint16_t out[N_CHANNELS]) const;

    bool isInitialized() const { return initialized; }

private:
    Adafruit_TCS34725 tcs;
    bool initialized;
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

#endif // RGB_HPP