#include <Arduino.h>
#include <Servo.h>
#include "pins.h"

Servo s;
Servo s2;


// pin 6 separator
// pin 30 separator

void setup() {
    Serial.begin(115200);
    s.attach(6);
    s.write(90);
    delay(500);
    s.detach();
    Serial.println("done pin 6 servo 3");

    s2.attach(30);
    s2.write(90);
    delay(500);
    s2.detach();
    Serial.println("done pin 30 servo 4");
}

void loop() {}

