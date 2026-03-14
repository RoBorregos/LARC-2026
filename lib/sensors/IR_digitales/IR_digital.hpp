#ifndef IR_DIGITAL_HPP
#define IR_DIGITAL_HPP

#include <Arduino.h>

class IR_digital
{
public:
    static constexpr uint8_t N = 4;

    enum Sensor : uint8_t
    {
        FL = 0,
        FR,
        BL,
        BR
    };

    IR_digital(const uint8_t pins_[N], uint8_t invertedMask_ = 0);

    bool begin();
    void update();

    bool getState(Sensor s) const;
    void getArray(bool out[N]) const;
    void debugPrint() const;

private:
    uint8_t pins[N];
    bool lineState[N];
    uint8_t invertedMask;
    bool initialized;
};

#endif