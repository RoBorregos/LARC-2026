/** 
 * Author: Ximena Patricia Garcia Magdaleno
 * 
 * 03/03/2025 
 * 
 * Elevator.cpp file from Elevator class
 * 
**/
#pragma once
#include "constants.h"
#include "pins.h"

class Elevator 
{
    public:
        Elevator();
        void begin();
        void update();
        void ElevatorPosition(int state_);

    private:
    int state;
    int pin1;
    int pin2;
    int pwm;
    
    void moveElevator(int direction);

};