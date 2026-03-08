#include <Arduino.h>
#include "Elevator.hpp"

Elevator elevator;

void setup()
{
    Serial.begin(115200);
    elevator.begin();
}

void loop()
{
    // UP
    elevator.ElevatorPosition(1);
    delay(3000);

    // Stop
    elevator.ElevatorPosition(0);
    delay(2000);

    // DOWN
    elevator.ElevatorPosition(2);
    delay(3000);

    // Stop
    elevator.ElevatorPosition(0);
    delay(2000);
}