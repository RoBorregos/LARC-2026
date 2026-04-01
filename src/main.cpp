#include <Arduino.h>
#include "robot/StateMachine/StateMachine.h"
#include "robot/instances/instances.h"

LARCStateMachine stateMachine;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    LARC.begin();
    ir.begin();
    us1.begin();
    us2.begin();
    qtrFront.begin();
    servos.begin();
    elevator.begin();

    stateMachine.begin();
}

void loop()
{
    stateMachine.update();
}