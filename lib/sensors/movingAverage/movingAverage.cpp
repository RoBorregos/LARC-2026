#include "movingAverage.hpp"

MovingAverageFilter::MovingAverageFilter(uint8_t size)
    : windowSize(size), index(0), count(0), sum(0.0f)
{
    buffer = new float[windowSize];
    for (uint8_t i = 0; i < windowSize; i++)
    {
        buffer[i] = 0.0f;
    }
}

MovingAverageFilter::~MovingAverageFilter()
{
    delete[] buffer;
}

void MovingAverageFilter::reset()
{
    index = 0;
    count = 0;
    sum = 0.0f;

    for (uint8_t i = 0; i < windowSize; i++)
    {
        buffer[i] = 0.0f;
    }
}

float MovingAverageFilter::update(float newValue)
{
    if (count < windowSize)
    {
        buffer[index] = newValue;
        sum += newValue;
        count++;
    }
    else
    {
        sum -= buffer[index];
        buffer[index] = newValue;
        sum += newValue;
    }

    index++;
    if (index >= windowSize)
    {
        index = 0;
    }

    return getAverage();
}

float MovingAverageFilter::getAverage() const
{
    if (count == 0)
        return 0.0f;

    return sum / count;
}

bool MovingAverageFilter::isReady() const
{
    return (count >= windowSize);
}