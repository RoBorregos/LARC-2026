/*
 * @file tof.cpp
 * @date 2026-01-28
 *
 * @brief VL53L0X / VL53L1X TOF distance sensor (I2C)
 *        Millis-based update with last-known good reading.
*/

#include "tof.hpp"

ToF::ToF()
    : type_(ToFType::L1X),
      initialized(false), continuous(false),
      useMux(false),
      distanceMm(INVALID_MM),
      lastUpdateMs_(0),
      updateIntervalMs_(Constants::ToFConfig::kContinuousPeriodMs),
      maxRangeMm_(600),
      muxChannel_(0), mux_(nullptr)
{}

ToF::ToF(uint8_t muxChannel, TCA9548A& mux, ToFType type)
    : type_(type),
      initialized(false), continuous(false),
      useMux(true),
      distanceMm(INVALID_MM),
      lastUpdateMs_(0),
      updateIntervalMs_(Constants::ToFConfig::kContinuousPeriodMs),
      maxRangeMm_(600),
      muxChannel_(muxChannel), mux_(&mux)
{}

bool ToF::begin()
{
    selectIfMux();
    
    // Non blocking settle: just record time; first update() will
    // respect the interval guard anyway.
    uint32_t settleStart = millis();
    while (millis() - settleStart < Constants::ToFConfig::kMuxSettleDelayMs);

    bool ok = false;

    if (type_ == ToFType::L0X) {
        sensorL0X.setTimeout(Constants::ToFConfig::kTimeoutMs);
        ok = sensorL0X.init();
        if (ok) {
            sensorL0X.setMeasurementTimingBudget(Constants::ToFConfig::kTimingBudgetUs);
            sensorL0X.startContinuous(Constants::ToFConfig::kContinuousPeriodMs);
        }
    } else {
        sensorL1X.setTimeout(Constants::ToFConfig::kTimeoutMs);
        ok = sensorL1X.init();
        if (ok) {
            sensorL1X.setDistanceMode(VL53L1X::Short);
            sensorL1X.setMeasurementTimingBudget(Constants::ToFConfig::kTimingBudgetUs);
            sensorL1X.startContinuous(Constants::ToFConfig::kContinuousPeriodMs);
        }
    }

    if (!ok) {
        initialized = false;
        return false;
    }

    continuous    = true;
    initialized   = true;
    lastUpdateMs_ = millis();

    return true;
}

void ToF::update()
{
    if (!initialized) return;

    // Rate limit: don't poll faster than the sensor produces data
    uint32_t now = millis();
    if (now - lastUpdateMs_ < updateIntervalMs_) return;
    lastUpdateMs_ = now;

    selectIfMux();

    if (type_ == ToFType::L0X) {
        // Single read call. avoids the double read bug
        uint16_t raw = sensorL0X.readRangeContinuousMillimeters();

        if (sensorL0X.timeoutOccurred()) {
            // Keep last known good value, don't overwrite with INVALID
            return;
        }
        if (raw == 65535 || raw == 0) {
            // Sensor returned garbage = keep last value
            return;
        }
        if (raw > maxRangeMm_) {
            distanceMm = maxRangeMm_; // valor alto = sin obstáculo
            return;
        }
        distanceMm = raw;

    } else {
        if (!sensorL1X.dataReady()) {
            // No new measurement yet = keep last value
            return;
        }
        uint16_t raw = sensorL1X.read(false);  // non blocking, clears interrupt internally

        if (sensorL1X.timeoutOccurred()) {
            return;
        }
        if (raw == 0) return; // mantener último valor si es 0
        if (raw > maxRangeMm_) {
            distanceMm = maxRangeMm_; // valor alto = sin obstáculo
            return;
        }
        distanceMm = raw;
    }
}

void ToF::setMaxRange(uint16_t mm)
{
    maxRangeMm_ = mm;
}

void ToF::setUpdateInterval(uint16_t ms)
{
    updateIntervalMs_ = ms;
}

void ToF::setTimingBudgetMs(uint16_t ms)
{
    if (!initialized) return;
    selectIfMux();

    uint32_t budgetUs = (uint32_t)ms * 1000UL;

    if (type_ == ToFType::L0X)
        sensorL0X.setMeasurementTimingBudget(budgetUs);
    else
        sensorL1X.setMeasurementTimingBudget(budgetUs);
}

void ToF::setInterMeasurementMs(uint16_t ms)
{
    if (!initialized) return;
    selectIfMux();
    if (!continuous) return;

    if (type_ == ToFType::L0X)
        sensorL0X.startContinuous(ms);
    else
        sensorL1X.startContinuous(ms);

    // Keep the software rate limit in sync
    updateIntervalMs_ = ms;
}

void ToF::startContinuous(uint16_t periodMs)
{
    if (!initialized) return;
    selectIfMux();

    if (type_ == ToFType::L0X)
        sensorL0X.startContinuous(periodMs);
    else
        sensorL1X.startContinuous(periodMs);

    continuous       = true;
    updateIntervalMs_ = periodMs;
}

void ToF::stopContinuous()
{
    if (!initialized) return;
    selectIfMux();

    if (type_ == ToFType::L0X)
        sensorL0X.stopContinuous();
    else
        sensorL1X.stopContinuous();

    continuous = false;
}