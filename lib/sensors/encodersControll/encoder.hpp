#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>

class ENCODER
{
public:

    ENCODER();

    bool begin();
    void update();

private:

};

#endif