#pragma once
#include <Arduino.h>

class Mux74HC4067
{
public:
    Mux74HC4067(uint8_t sig, uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3)
        : _sig(sig), _s0(s0), _s1(s1), _s2(s2), _s3(s3) {}

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

    inline void setSigMode(uint8_t mode) { pinMode(_sig, mode); }
    inline uint8_t sig() const { return _sig; }

private:
    uint8_t _sig, _s0, _s1, _s2, _s3;
};