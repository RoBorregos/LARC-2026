#pragma once
#include <Arduino.h>

class IR_mux
{
public:
    enum Sensor : uint8_t
    {
        FL = 0,
        FR,
        BL,
        BR,
        N
    };

    IR_mux(const uint8_t pins_[N], uint8_t invertedMask_ = 0);

    bool begin();
    void update();

    bool getState(Sensor s) const;
    void getArray(bool out[N]) const;
    void debugPrint() const;

private:
    uint8_t pins[N];
    uint8_t invertedMask;
    bool initialized;

    uint16_t rawVal[N];
    bool lineState[N];
};