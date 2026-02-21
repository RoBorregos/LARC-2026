/**
 * @file ultrasonic.cpp
 * @brief Sensor ultrasónico HC-SR04 sin delay
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
        //Idle: esperar el período entre pings
        case State::Idle:
            if (nowMs - lastPingMs >= pingperiodms)
            {
                digitalWrite(trig, HIGH);
                trigStartUs = nowUs;
                state       = State::Trig_High;
            }
            break;

        //Trig High: mantener trigger en HIGH 10 µs
        case State::Trig_High:
            if (nowUs - trigStartUs >= trighighus)
            {
                digitalWrite(trig, LOW);
                state = State::Wait_Echo_Up;
            }
            break;

        //Wait echo high: esperar subida del echo
        // Timeout global: si pasan kEchoTimeoutUs sin subida = reset
        case State::Wait_Echo_Up:
            if (digitalRead(echo) == HIGH)
            {
                echoRiseUs = nowUs;
                state      = State::Wait_Echo_Down;
            }
            else if (nowUs - trigStartUs >= echotimeoutus)
            {
                // Sin echo: objeto fuera de rango o fallo
                valid      = false;
                lastPingMs = nowMs; // reinicia período desde ahora
                state      = State::Idle;
            }
            break;

        //Wait echo down: esperar bajada del echo calcular distancia
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
                // Echo se quedó en HIGH demasiado tiempo → objeto muy lejos
                valid      = false;
                lastPingMs = nowMs;
                state      = State::Idle;
            }
            break;
    }
}

float Ultrasonic::getdistance() const { return distance; }
bool  Ultrasonic::isValid()     const { return valid; }