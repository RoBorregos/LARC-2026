#include <Arduino.h>
#include "robot/StateMachine/StateMachine.hpp"
#include "robot/instances/instances.hpp"

LARCStateMachine stateMachine;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    ir.begin();
    qtrFront.begin();
    servos.begin();
    elevator.begin();

    stateMachine.begin();
}

void loop()
{
    stateMachine.update();

}