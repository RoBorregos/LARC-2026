#pragma once

/**
 * @file ultrasonic.hpp
 * @brief Sensor ultrasónico HC-SR04 non-blocking con máquina de estados robusta
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
    // Estados explícitos del ciclo
    enum class State : uint8_t
    {
        Idle,  // esperando el período entre pings
        Trig_High, // trigger en HIGH, esperando 10 µs
        Wait_Echo_Down, // trigger bajado, esperando subida del echo
        Wait_Echo_Up // echo en HIGH, esperando bajada para medir
    };

    uint8_t trig;
    uint8_t echo;

    State    state;
    uint32_t lastPingMs;  // momento del último ping (ms)
    uint32_t trigStartUs; // momento en que subió el trigger (µs)
    uint32_t echoRiseUs;  // momento en que subió el echo (µs)

    float distance;
    bool  valid;

    //Tiempos  
    static constexpr uint32_t pingperiodms  = Constants::UltrasonicConstants::kPingPeriodMs; // ms entre pings 
    static constexpr uint32_t trighighus    = Constants::UltrasonicConstants::kTrigHighUs; // µs que el trigger está en HIGH
    static constexpr uint32_t echotimeoutus = Constants::UltrasonicConstants::kEchoTimeoutUs; // µs (~4 m máx)
};