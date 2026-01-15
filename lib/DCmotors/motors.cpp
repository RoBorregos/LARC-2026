/*
 * @file motors.cpp
 * @author Ximena Patricia Garcia Magdaleno
 * @brief Omnidirectional Motor class
 * @version 0.1
 * @date 2026-01-12
 */

#include "motors.hpp"
#include <math.h>

DCMotor::DCMotor(int in1, int in2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2, float d)
{
    in1_pin = in1;
    in2_pin = in2;
    pwm_pin = pwm;

    encoder_pin1_ = encoder_pin1; // change to encoder_pin1_
    encoder_pin2_ = encoder_pin2;

    inverted = invert;
    diameter = d;
}


DCMotor::DCMotor(int in1, int in2, int pwm, bool invert, 
            int encoder_pin1, int encoder_pin2)
{
    in1_pin = in1;
    in2_pin = in2;
    pwm_pin = pwm;
    encoder_pin1_ = encoder_pin1;
    encoder_pin2_ = encoder_pin2;

    inverted = invert;
}