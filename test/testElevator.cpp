#include <Arduino.h>
#include "lib/Elevator/Elevator.hpp"
Elevator elevator;

void setup()
{
    Serial.begin(115200);
    elevator.begin();
}

void loop()
{
    // UP
    elevator.setState(1);   
    elevator.update();
    delay(3000);

    // STOP
    elevator.setState(0);   // STOP
    elevator.update();
    delay(2000);

    // DOWN
    elevator.setState(2);   
    elevator.update();
    delay(3000);

    // STOP
    elevator.setState(0);   
    elevator.update();
    delay(2000);
}