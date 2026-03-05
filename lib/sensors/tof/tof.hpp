/**
 * @file tof.hpp
 * @date 2026-01-28
 *
 * @brief VL53L1X TOF distance sensor (I2C)
*/

#ifndef TOF_HPP
#define TOF_HPP

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

class ToF
{
public:
    // Invalid distance if timeout occurs (sensor out of range, etc)
    static constexpr uint16_t INVALID_MM = 0xFFFF;

    ToF();

    bool begin();
    void update();

    // Last reading in mm
    uint16_t getDistanceMm() const { return distanceMm; }

    bool isInitialized() const { return initialized; }

    // Optional: changes the timing budget (time spent on each measurement)
    
    void setTimingBudgetMs(uint16_t ms);

    // Optional: changes the time between measurements
    // (if you´re on continuous mode)
    // *** IMPORTANT: Cant be smaller than the timing budget ***
    void setInterMeasurementMs(uint16_t ms);

    // Optional: Starts/stops continuous mode
    void startContinuous(uint16_t periodMs = 50);
    void stopContinuous();

private:
    VL53L1X sensor;
    bool initialized;
    bool continuous;

    uint16_t distanceMm;
};

#endif // TOF_HPP