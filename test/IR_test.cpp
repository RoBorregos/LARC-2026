#include <Arduino.h>
#include "pins.h"
#include "IR.hpp"

IRLine ir(
    Pins::kLineSensorFL,
    Pins::kLineSensorFR,
    Pins::kLineSensorBL,
    Pins::kLineSensorBR,
    true
);

void setup()
{
    Serial.begin(115200);

    ir.begin();

    // Distance sensors (TRIG as OUTPUT, ECHO as INPUT)
    for (int i = 0; i < 2; i++)
    {
        pinMode(Pins::kDistanceSensors[i][0], OUTPUT);
        pinMode(Pins::kDistanceSensors[i][1], INPUT);
    }

    Serial.println("Pins test initialized");
}

long readUltrasonic(uint8_t trigPin, uint8_t echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000);
    long distance = duration * 0.034 / 2;
    return distance;
}

void loop()
{
    ir.update();

    bool fl = ir.getRaw(IRLine::FL);
    bool fr = ir.getRaw(IRLine::FR);
    bool bl = ir.getState(IRLine::BL);
    bool br = ir.getState(IRLine::BR);

    Serial.print("Line Sensors -> ");
    Serial.print("FL: "); Serial.print(fl);
    Serial.print(" FR: "); Serial.print(fr);
    Serial.print(" BL: "); Serial.print(bl);
    Serial.print(" BR: "); Serial.println(br);

    for (int i = 0; i < 2; i++)
    {
        long dist = readUltrasonic(
            Pins::kDistanceSensors[i][0],
            Pins::kDistanceSensors[i][1]
        );

        Serial.print("Distance Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(dist);
        Serial.println(" cm");
    }

    Serial.println("------------------------");
    delay(80);
}
