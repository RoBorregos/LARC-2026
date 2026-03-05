/**
 * @file main.cpp
 * @brief Ejemplo de uso de IRLine y Ultrasonic con Pins.h
 */

#include <Arduino.h>
#include "pins.h"
#include "ultrasonic.hpp"

// Ultrasónico frontal izquierdo:  {TRIG=36, ECHO=34}
Ultrasonic usFL(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);

// Ultrasónico frontal derecho:    {TRIG=35, ECHO=33}
Ultrasonic usFR(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);


void setup()
{
    Serial.begin(115200);
    usFL.begin();
    usFR.begin();
}

void loop()
{
    usFL.update();
    usFR.update();

    // Ultrasónico izquierdo
    Serial.print(" | US_FL → ");
    if (usFL.isValid())
    {
        Serial.print(usFL.getdistance(), 1);
        Serial.print(" cm");
    }
    else
    {
        Serial.print("sin lectura");
    }

    // Ultrasónico derecho
    Serial.print(" | US_FR → ");
    if (usFR.isValid())
    {
        Serial.print(usFR.getdistance(), 1);
        Serial.println(" cm");
    }
    else
    {
        Serial.println("sin lectura");
    }
}