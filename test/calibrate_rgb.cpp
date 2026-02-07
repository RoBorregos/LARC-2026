/**
 * @file calibrate_rgb.cpp
 * @date 2026-02-06
 *
 * @brief Lectura cruda del sensor TCS34725 para calibración de semillas
 */

#include <Arduino.h>
#include "rgb.hpp"

RGB rgb;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("RGB calibration mode");

    if (!rgb.begin())
    {
        Serial.println("ERROR: RGB sensor not detected");
        while (1)
        {
            delay(500);
        }
    }

    Serial.println("RGB sensor initialized");
    Serial.println("Format: Clear, Red, Green, Blue");
}

void loop()
{
    rgb.update();

    uint16_t raw[RGB::N_CHANNELS] = {0, 0, 0, 0};
    rgb.getRaw(raw);

    // raw[0]=Clear, raw[1]=Red, raw[2]=Green, raw[3]=Blue
    Serial.print(raw[0]); Serial.print(", ");
    Serial.print(raw[1]); Serial.print(", ");
    Serial.print(raw[2]); Serial.print(", ");
    Serial.println(raw[3]);

    delay(200);
}