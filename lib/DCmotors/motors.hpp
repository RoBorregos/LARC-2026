/*
 * @file motors.hpp
 *
 * @author Ximena Patricia Garcia Magdaleno
 * 
 * @brief Header file for the Omnidirectional Motor class, which is the class for the Omnidirectional motor.
 *
 * @version 0.1
 * 
 * @date 2026-01-14
 */

#ifdef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>
#include <Encoder.h>

class DCmotor
{
    public:

        //create "Direction" class?
        enum class Direction { 
            FORWARD,
            BACKWARD,
            LEFT,
            RIGHT
        };
      

        DCmotor(int in1, int int2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2, float d);

        DCmotor(int in1, int int2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2); //without diameter
        
        ~DCmotor();    

        
        void begin();
        void move(int speed, Direction direction);
        void stop();
        int getEncoderCount();
        double getPRotation(); // Position Rotation
        float getPMeters(); // Position Meters




    private:
    // change to class/header (constants/motor)
        int in1_pin;
        int in2_pin;
        int pwm_pin;
        int encoder_pin1;
        int encoder_pin2;
        bool inverted;
        float diameter;

        Direction current;
        
        Encoder* encoder_;
    



};

#endif 