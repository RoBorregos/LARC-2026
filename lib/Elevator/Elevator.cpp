/** 
 * Author: Ximena Patricia Garcia Magdaleno
 * 
 * 03/03/2025 
 * 
 * Elevator.cpp file from Elevator class
 * 
**/
#include "Elevator.hpp"

Elevator::Elevator() : state(), pin1(Pins::kElevatorINA1), pin2(Pins::kElevatorINA2), pwm(Pins::kElevatorPWM)
{

}

void Elevator::begin()
{
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pwm, OUTPUT);
}

void Elevator::update()
{
    switch (state)
    {
        case STOP:
            analogWrite(pwm, 0); //elevator stop
            break;
        case UP:
            moveElevator(UP);
        break;
        case DOWN:
            moveElevator(DOWN);
        break;
    }
}


void Elevator::ElevatorPosition(int state)
{
    state = static_cast<ElevatorState>(state);
}
    

void Elevator::moveElevator(int direction)
{
    if (direction == UP){
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
        analogWrite(pwm, 180);
        delay(7000);
    }
    else if(direction == DOWN){
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
        analogWrite(pwm, 180);   // velocidad de 0 a 255
        delay(7000);
    }


}
/*
#include <Arduino.h>
#include "pins.h"

const uint8_t p1  = Pins::kElevatorINA1;
const uint8_t p2  = Pins::kElevatorINA2;
const uint8_t pwm = Pins::kElevatorPWM;

void setup()
{
    pinMode(p1, OUTPUT);
    pinMode(p2, OUTPUT);
    pinMode(pwm, OUTPUT);
}

void loop()
{
    // Stop
    analogWrite(pwm, 0);
    delay(2000);

    // Bajar
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
    analogWrite(pwm, 180);   // velocidad de 0 a 255
    delay(7000);

    // Stop
    analogWrite(pwm, 0);
    delay(2000);

    // Subir
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
    analogWrite(pwm, 180);
    delay(7000);

    // Stop
    analogWrite(pwm, 0);
    delay(2000);
    
}
*/
