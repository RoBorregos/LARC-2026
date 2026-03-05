/**
 * @file main.cpp
 * @brief Ejemplo de uso de IRLine y Ultrasonic con Pins.h
 */

#include <Arduino.h>
#include "pins.h"
#include "IR.hpp"
#include "ultrasonic.hpp"

// ── Objetos globales ─────────────────────────────────────────────────────────

// IR: FL, FR, BL, BR desde Pins.h
// invertedMask = 0b1111 → todos invertidos (ajusta si alguno está al revés)
IRLine ir(
    Pins::kLineSensorFL,
    Pins::kLineSensorFR,
    Pins::kLineSensorBL,
    Pins::kLineSensorBR,
    0b1111
);

// Ultrasónico frontal izquierdo:  {TRIG=36, ECHO=34}
Ultrasonic usFL(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);

// Ultrasónico frontal derecho:    {TRIG=35, ECHO=33}
Ultrasonic usFR(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);


void setup()
{
    Serial.begin(115200);

    ir.begin();
    usFL.begin();
    usFR.begin();
}

void loop()
{
    ir.update();
    usFL.update();
    usFR.update();

    // IR
    bool fl = ir.getState(IRLine::FL);
    bool fr = ir.getState(IRLine::FR);
    bool bl = ir.getState(IRLine::BL);
    bool br = ir.getState(IRLine::BR);

    Serial.print("IR → FL:");
    Serial.print(fl);
    Serial.print(" FR:");
    Serial.print(fr);
    Serial.print(" BL:");
    Serial.print(bl);
    Serial.print(" BR:");
    Serial.print(br);

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