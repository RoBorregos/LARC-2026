/**
 * @file ultrasonic.cpp
 * @brief Ultrasonic sensor library without delays.
 */

#include "ultrasonic.hpp"

Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t echoPin)
    : trig(trigPin), echo(echoPin),
      state(State::Idle),
      lastPingMs(0), trigStartUs(0), echoRiseUs(0),
      distance(0.0f), valid(false)
{}

void Ultrasonic::begin()
{
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trig, LOW);

    state      = State::Idle;
    lastPingMs = millis();
}

void Ultrasonic::update()
{
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();

    switch (state)
    {
        //Idle: wait for the period between pings
        case State::Idle:
            if (nowMs - lastPingMs >= pingperiodms)
            {
                digitalWrite(trig, HIGH);
                trigStartUs = nowUs;
                state       = State::Trig_High;
            }
            break;

        //Trig High: Maintain trig HIGH for 10 µs
        case State::Trig_High:
            if (nowUs - trigStartUs >= trighighus)
            {
                digitalWrite(trig, LOW);
                state = State::Wait_Echo_Up;
            }
            break;

        //Wait echo high: wait for the echo to go HIGH
        // Global timeout: if kEchoTimeoutUs passes without rising = reset
        case State::Wait_Echo_Up:
            if (digitalRead(echo) == HIGH)
            {
                echoRiseUs = nowUs;
                state      = State::Wait_Echo_Down;
            }
            else if (nowUs - trigStartUs >= echotimeoutus)
            {
                // No echo: object out of range or failure
                valid      = false;
                lastPingMs = nowMs; // reset period from now
                state      = State::Idle;
            }
            break;

        //Wait echo down: wait for the echo to go LOW and calculate distance
        case State::Wait_Echo_Down:
            if (digitalRead(echo) == LOW)
            {
                distance   = (nowUs - echoRiseUs) / 58.0f;
                valid      = true;
                lastPingMs = nowMs;
                state      = State::Idle;
            }
            else if (nowUs - echoRiseUs >= echotimeoutus)
            {
                // Echo stayed HIGH for too long = object is too far away
                valid      = false;
                lastPingMs = nowMs;
                state      = State::Idle;
            }
            break;
    }
}

float Ultrasonic::getdistance() const { return distance; }
bool  Ultrasonic::isValid()     const { return valid; }