#include <Arduino.h>
#include "Vision.hpp"
#include "ServoSystem.hpp"

Vision vision(Serial);
ServoSystem servos;
bool started = false;

void setup()
{
    Serial.begin(115200);
    delay(500);

    vision.begin();
    servos.begin();
}

void loop()
{
    servos.update();
    vision.update();

    if (!started && vision.isPiReady())
    {
        vision.clearErrors();
        vision.startBeans();
        started = true;
    }

    servos.separatorUpdate(vision.warmBall(), vision.coolBall());
    servos.intakeUpperUpdate(vision.beanTop());
    servos.intakeLowerUpdate(vision.beanBottom());
}