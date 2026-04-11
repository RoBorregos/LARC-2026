#pragma once
#include <Arduino.h>
#include "pins.h"

class Mux74HC4067
{
public:
    // Constructor con SIG configurable
    Mux74HC4067(uint8_t sig = Pins::kMuxSig)
        : _sig(sig),
          _s0(Pins::kMuxS0),
          _s1(Pins::kMuxS1),
          _s2(Pins::kMuxS2),
          _s3(Pins::kMuxS3)
    {
    }

    void begin()
    {
        pinMode(_sig, INPUT);
        pinMode(_s0, OUTPUT);
        pinMode(_s1, OUTPUT);
        pinMode(_s2, OUTPUT);
        pinMode(_s3, OUTPUT);
        select(0);
    }

    inline void select(uint8_t ch) const
    {
        digitalWrite(_s0, (ch >> 0) & 1);
        digitalWrite(_s1, (ch >> 1) & 1);
        digitalWrite(_s2, (ch >> 2) & 1);
        digitalWrite(_s3, (ch >> 3) & 1);
    }

    inline uint16_t read(uint8_t ch) const
    {
        select(ch);
        delayMicroseconds(5);
        return analogRead(_sig);
    }

private:
    uint8_t _sig;
    uint8_t _s0;
    uint8_t _s1;
    uint8_t _s2;
    uint8_t _s3;
};