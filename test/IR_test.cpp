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


void setup()
{
    Serial.begin(115200);

    ir.begin();
}

void loop()
{
    ir.update();

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
}