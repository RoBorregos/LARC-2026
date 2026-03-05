/**
 * @file tof.cpp
 * @date 2026-01-28
 *
 * @brief Implementation of VL53L1X Sensor
 */

#include "tof.hpp"

ToF::ToF()
    : initialized(false),
      continuous(false),
      distanceMm(INVALID_MM)
{
}

bool ToF::begin()
{
    // Wire.begin(); is normally called once in setup() outside of this class.
    sensor.setTimeout(50); // ms,

    if (!sensor.init())
    {
        initialized = false;
        return false;
    }

    // Distance mode: Short/Medium/Long
    sensor.setDistanceMode(VL53L1X::Short);

    // Timing budget by defect. Pololu recommends 50ms for balance between speed and accuracy.
    sensor.setMeasurementTimingBudget(50000); // 50,000 us = 50ms

    // Bu defect we start on continuous so update(): is cheap
    sensor.startContinuous(50); // ms
    continuous = true;

    initialized = true;

    // First reading
    update();
    return true;
}

void ToF::update()
{
    if (!initialized)
        return;

    // On continuous mode it returns the last reading, and triggers a new one. So we can call it as fast as we want.
    uint16_t d = sensor.read();

    if (sensor.timeoutOccurred())
    {
        distanceMm = INVALID_MM;
        return;
    }

    distanceMm = d;
}

void ToF::setTimingBudgetMs(uint16_t ms)
{
    if (!initialized)
        return;

    sensor.setMeasurementTimingBudget((uint32_t)ms * 1000UL);
}

void ToF::setInterMeasurementMs(uint16_t ms)
{
    if (!initialized)
        return;
    //Only applies if you reconfigure continuous mode with startContinuous(period)
    if (continuous)
        sensor.startContinuous(ms);
}

void ToF::startContinuous(uint16_t periodMs)
{
    if (!initialized)
        return;

    sensor.startContinuous(periodMs);
    continuous = true;
}

void ToF::stopContinuous()
{
    if (!initialized)
        return;

    sensor.stopContinuous();
    continuous = false;
}