#ifndef MOVING_AVERAGE_FILTER_HPP
#define MOVING_AVERAGE_FILTER_HPP

#include <Arduino.h>

class MovingAverageFilter
{
private:
    float* buffer;
    uint8_t windowSize;
    uint8_t index;
    uint8_t count;
    float sum;

public:
    explicit MovingAverageFilter(uint8_t size);
    ~MovingAverageFilter();

    void reset();
    float update(float newValue);
    float getAverage() const;
    bool isReady() const;
};

#endif