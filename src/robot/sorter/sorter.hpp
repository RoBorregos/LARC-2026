/*
 * @file bno.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Source file for the sorter class.
 * @version 0.1
 * @date 2026-03-27
 */

#ifndef SORTER_HPP
#define SORTER_HPP

#include <Arduino.h>
#include "Servo.h"
#include "pins.h"
#include "constants.h"
#include "robot/instances/instances.h"

class SORTER
{
public:
    SORTER();
    void separate(bool warmBean);     // warm or cold colored bean
    void box(SORTER box_, bool left); // Left or right move of the box

private:
    Servo separateS; //"RGB" (separates) servo
    Servo boxS;      // Move box servo
    Servo rightFree; // Box Right Servo
    Servo leftFree;  // Box Left servo
    bool boxLeft;
    uint32_t now;

    void moveLeft();
    void moveRight();

    // MILLIS
    void startStateTime();
    //const uint32_t now = millis();

};

#endif