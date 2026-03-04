#pragma once

/**
 * @file ultrasonic.hpp
 * @brief Ultrasonic sensor HC-SR04 non-blocking state machine
 */

#include <Arduino.h>
#include <constants.h>

class Ultrasonic
{
public:
    Ultrasonic(uint8_t trigPin, uint8_t echoPin);

    void  begin();
    void  update();

    float getdistance() const;
    bool  isValid()     const;

private:
    // Explicit stater per cycle
    enum class State : uint8_t
    {
        Idle,  // waiting for period between pings
        Trig_High, // trigger HIGH, waiting 10 µs
        Wait_Echo_Down, // trigger low, waiting for echo high
        Wait_Echo_Up // echo HIGH, waiting for echo low to measure
    };

    uint8_t trig;
    uint8_t echo;

    State    state;
    uint32_t lastPingMs;  // moment of last ping (ms)
    uint32_t trigStartUs; // moment the trig went high(µs)
    uint32_t echoRiseUs;  // moment the echo went high (µs)

    float distance;
    bool  valid;

    //Tiempos  
    static constexpr uint32_t pingperiodms  = Constants::UltrasonicConstants::kPingPeriodMs; // ms between pings 
    static constexpr uint32_t trighighus    = Constants::UltrasonicConstants::kTrigHighUs; // µs in which trig is HIGH
    static constexpr uint32_t echotimeoutus = Constants::UltrasonicConstants::kEchoTimeoutUs; // µs (~4 m máx)
};