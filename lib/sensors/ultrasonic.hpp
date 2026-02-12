#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <Arduino.h>

class Ultrasonic
{
public:
    explicit Ultrasonic(uint8_t trigPin, uint8_t echoPin);

    bool begin();

    void update();

    float distanceCm() const;
    bool isValid() const;

private:
    uint8_t trig;
    uint8_t echo;

    float distance;
    bool valid;
};

#endif