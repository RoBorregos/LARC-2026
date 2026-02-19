#include <Arduino.h>
#include "pins.h"
#include "ultrasonic.hpp"

// config
constexpr float kObstacleThresholdCm = 20.0f; //distancia (20cm) Cambiar luego desde constants
constexpr uint16_t kPrintIntervalMs = 80; //intervalo de lectura (Placeholder, mover a constants después)

// sensores
Ultrasonic usFrontLeft(
    Pins::kDistanceSensors[0][0],
    Pins::kDistanceSensors[0][1]
);

Ultrasonic usFrontRight(
    Pins::kDistanceSensors[1][0],
    Pins::kDistanceSensors[1][1]
);

unsigned long lastPrint = 0;

void setup()
{
    Serial.begin(115200);
    delay(50);

    usFrontLeft.begin();
    usFrontRight.begin();

    Serial.println("=== ULTRASONIC LIBRARY TEST ===");
}

void loop()
{

    usFrontLeft.update();
    usFrontRight.update();

    unsigned long now = millis();

    if (now - lastPrint >= kPrintIntervalMs)
    {
        lastPrint = now;

        bool leftValid  = usFrontLeft.isValid();
        bool rightValid = usFrontRight.isValid();

        float leftDist  = usFrontLeft.getdistance();
        float rightDist = usFrontRight.getdistance();

        bool leftObstacle  = leftValid  && (leftDist  <= kObstacleThresholdCm);
        bool rightObstacle = rightValid && (rightDist <= kObstacleThresholdCm);

        Serial.println();

        // Mostrar distancia de cada sensor
        Serial.print("FL: ");
        if (leftValid)
        {
            Serial.print(leftDist);
            Serial.print(" cm");
        }
        else
        {
            Serial.print("INVALID");
        }
        Serial.println();

        Serial.print("FR: ");
        if (rightValid)
        {
            Serial.print(rightDist);
            Serial.print(" cm");
        }
        else
        {
            Serial.print("INVALID");
        }
        Serial.println();

        // Lógica de obstáculo
        if (leftObstacle || rightObstacle)
        {
            Serial.println("Se detectó obstáculo, mover robot");
        }
        else
        {
            Serial.println("Camino libre");
        }
    }
}