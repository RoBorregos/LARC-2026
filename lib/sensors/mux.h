// mux.h
#pragma once
#include <Arduino.h>
#include "pins.h"

class Mux74HC4067
{
public:
    Mux74HC4067()
        : _sig(Pins::kMuxSig),
          _s0(Pins::kMuxS0),
          _s1(Pins::kMuxS1),
          _s2(Pins::kMuxS2),
          _s3(Pins::kMuxS3) {}

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

    void debugPrint() const
    {
        for (uint8_t ch = 0; ch < 16; ch++) {
            uint16_t v = read(ch);  // just read(), not mux.read()
            Serial.print(ch);
            Serial.print(':');
            Serial.print(v);
            if (ch < 15) Serial.print('\t');
        }
        Serial.println();
    }
private:
    uint8_t _sig, _s0, _s1, _s2, _s3;
};