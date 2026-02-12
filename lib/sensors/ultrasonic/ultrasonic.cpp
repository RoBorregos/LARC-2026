#include "ultrasonic.hpp"

Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t echoPin)
    : trig(trigPin), echo(echoPin),
      distance(0.0f), valid(false)
{
}

bool Ultrasonic::begin()
{
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    digitalWrite(trig, LOW);
    delay(50);

    return true;
}

void Ultrasonic::update()
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // Leer duración del pulso ECHO
    unsigned long duration = pulseIn(echo, HIGH, 30000); // timeout 30ms (~5m)

    if (duration == 0)
    {
        // esto solo es en caso de timeout (fuera de rango o problemas de conexión)
        valid = false;
        return;
    }

    distance = duration / 58.0f;
    valid = true;
}

float Ultrasonic::distanceCm() const
{
    return distance;
}

bool Ultrasonic::isValid() const
{
    return valid;
}