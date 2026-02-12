/**
 * @file tof.cpp
 * @date 2026-01-28
 *
 * @brief Implementación VL53L1X (Pololu)
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
    // Wire.begin() normalmente se llama una vez en setup() fuera de aquí.
    sensor.setTimeout(50); // ms,

    if (!sensor.init())
    {
        initialized = false;
        return false;
    }

    // Modo de distancia: Short, Medium, Long
    sensor.setDistanceMode(VL53L1X::Short);

    // Timing budget por defecto para el begin.
    sensor.setMeasurementTimingBudget(50000); // 50,000 us = 50ms

    // Por defecto arrancamos en continuo para que update() sea barato
    sensor.startContinuous(50); // ms
    continuous = true;

    initialized = true;

    // Primera lectura
    update();
    return true;
}

void ToF::update()
{
    if (!initialized)
        return;

    // En modo continuo, read() entrega la última medición lista
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

    // Pololu usa microsegundos
    sensor.setMeasurementTimingBudget((uint32_t)ms * 1000UL);
}

void ToF::setInterMeasurementMs(uint16_t ms)
{
    if (!initialized)
        return;

    // Esto solo aplica si reconfiguras modo continuo con startContinuous(period)
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