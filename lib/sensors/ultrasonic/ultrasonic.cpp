#include "ultrasonic.hpp"

Ultrasonic::Ultrasonic(uint8_t trigPin, uint8_t echoPin)
    : trig(trigPin),
      echo(echoPin),
      state(State::Idle),
      lastPingMs(0),
      tStateUs(0),
      echoRiseUs(0),
      distance(0.0f),
      valid(false)
{
}

bool Ultrasonic::begin()
{
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    digitalWrite(trig, LOW);

    // Usa placeholder (mover a constantes después)
    delay(kBeginSettleMs);

    lastPingMs = millis();
    state = State::Idle;
    valid = false;

    return true;
}

void Ultrasonic::update()
{
    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();

    switch (state)
    {
        case State::Idle:
        {
            // esperar a que sea tiempo de que que haga otro ping
            if (nowMs - lastPingMs >= kPingPeriodMs)
            {
                lastPingMs = nowMs;
                digitalWrite(trig, LOW);
                tStateUs = nowUs;
                state = State::TrigLow;
            }
            break;
        }

        case State::TrigLow:
        {
            if (nowUs - tStateUs >= kTrigLowUs)
            {
                digitalWrite(trig, HIGH);
                tStateUs = nowUs;
                state = State::TrigHigh;
            }
            break;
        }

        case State::TrigHigh:
        {
            if (nowUs - tStateUs >= kTrigHighUs)
            {
                digitalWrite(trig, LOW);
                tStateUs = nowUs;
                state = State::WaitEchoRise;
            }
            break;
        }

        case State::WaitEchoRise:
        {
            // Espera a que ECHO este HIGH o timeout
            if (digitalRead(echo) == HIGH)
            {
                echoRiseUs = micros();
                state = State::WaitEchoFall;
            }
            else if (nowUs - tStateUs >= kEchoTimeoutUs)
            {
                // Timeout: no se detectó el echo
                valid = false;
                state = State::Idle;
            }
            break;
        }

        case State::WaitEchoFall:
        {
            if (digitalRead(echo) == LOW)
            {
                const uint32_t duration = micros() - echoRiseUs;

                // Convertir a cm (speed of sound factor)
                distance = duration / 58.0f;
                valid = true;

                state = State::Idle;
            }
            else if (micros() - echoRiseUs >= kEchoTimeoutUs)
            {
                // esperando que ECHO vuelva a LOW
                valid = false;
                state = State::Idle;
            }
            break;
        }
    }
}

float Ultrasonic::distanceCm() const
{
    return distance;
}

bool Ultrasonic::isValid() const
{
    return valid;
}