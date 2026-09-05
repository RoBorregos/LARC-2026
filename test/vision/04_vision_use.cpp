/**
   @file 06_vision_use.cpp
   @date 2026-09-04
  
   @brief Every call the state machine ever needs to talk to vision.
          Reference sketch
 */

#include <Arduino.h>
#include "robot/instances/instances.hpp"

// 1. Lifecycle — two calls in setup, one in loop
void setup()
{
    Serial.begin(Constants::VisionConfig::kSerialBaud);

    servos.begin();
    vision.begin();
}

// 2. Enter the beans stage
//    One byte goes out and the call guards itself, so calling it every
//    loop costs nothing. call it on inBeansPhase() in state machine

void enterBeans()
{
    vision.startBeans();
}

// 3. Leave the beans stage
// stop() asks for IDLE. state
// the servos park on the first valid frame. resetGuards() re-arms
// start*() / stop() so the next request transmits again
// setState()
// in LARCStateMachine already calls it on every transition.
void leaveBeans()
{
    vision.stop();
    vision.resetGuards();
}

// 4. Release the beans (benefits stage)
// The doors are pulsed by the protocol, not held: the Teensy opens one
// for kBenefitOpenMs and refuses to open it again until it sees that
// door commanded shut. You do not time it and you do not touch the servo.
void enterBenefits()
{
    vision.startBenefits();
}

// 5. Read what vision is doing
void printVisionState()
{
    Serial.printf("link=%s orin=%s beans=%d benefits=%d up=%d lo=%d sep=%d\n",
                  vision.isLinkUp() ? "UP" : "DOWN",   // false once the watchdog fired
                  vision.isPiReady() ? "READY" : "-",  // Orin says it is up
                  vision.inBeansPhase(),               // acting on beans right now
                  vision.inBenefitsPhase(),            // acting on benefits right now
                  servos.intakeUpperDeployed(),        // last commanded, read-only
                  servos.intakeLowerDeployed(),
                  (int)servos.separatorPos());         // LEFT = mature, RIGHT = immature
}

// 6. Faults — latched until clearErrors()
// hasCriticalError() also latches when the link drops: nobody is
// steering the sorter any more, so stop driving as if it were.
void handleFaults()
{
    if (vision.hasCriticalError())
    {
        vision.stop();
        vision.clearErrors();
    }
}

// 7. Putting it together
void loop()
{
    // The one call that must run every loop. 
    //it reads the port, validates and confirms frames, drives the servos, runs the
    // watchdog and closes the benefit doors on time.
    vision.update();

    handleFaults();

    // Replace these with your own state conditions.
    const bool atBeansCorner    = false;
    const bool atBeansEnd       = false;
    const bool atBenefitsCorner = false;

    if (atBeansCorner)    enterBeans();
    if (atBeansEnd)       leaveBeans();
    if (atBenefitsCorner) enterBenefits();
}