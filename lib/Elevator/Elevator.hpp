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
        void ElevatorPosition(int state);

    private:
    int state;
    int pin1;
    int pin2;
    int pwm;

    enum ElevatorState
    {
        STOP =  0,
        UP = 1,
        DOWN = 2,
    };
    
    void moveElevator(int direction);

};