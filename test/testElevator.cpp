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
    elevator.update();
    delay(3000);

    // STOP
    elevator.ElevatorPosition(0); 
    elevator.update();
    delay(2000);

    // DOWN
    elevator.ElevatorPosition(2);   
    elevator.update();
    delay(3000);

    // STOP
    elevator.ElevatorPosition(0);   
    elevator.update();
    delay(2000);
}

/*
const uint8_t kUpperIntakeServo = 3;
const uint8_t kLowerIntakeServo = 4;

Servo upperServo;
Servo lowerServo;

void testOneServo(Servo &servo, const char *name)
{
    Serial.print("Testing: ");
    Serial.println(name);

    servo.write(0); // abierto 0
    delay(1000);
    servo.write(90); //cerrado 90
    delay(1000);

}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    upperServo.attach(kUpperIntakeServo);
    lowerServo.attach(kLowerIntakeServo);

    Serial.println("Starting intake servo test...");
}

void loop()
{
    //testOneServo(upperServo, "Upper Intake Servo");  // es el de abajo
    upperServo.write(20);
    lowerServo.write(20);
    delay(1000);

    //testOneServo(lowerServo, "Lower Intake Servo");
    upperServo.write(120); //90
    lowerServo.write(120);
    delay(1000);
}


*/