/**
 * @file qtr.hpp
 * @date 2026-01-28
 *
 * @brief QTR-8A (8-channel reflectance array) read through a shared 74HC4067 analog mux.
 *
 * Case of use, 2 separate arrays connected to the same mux:
 *  - QTR Frontal front connected form C0 to C7 (firstChannel = 0)
 *  - QTR Back connected from C8 to C15 (firstChannel = 8)
 * State machine can use them independently
 */

#ifndef QTR_HPP
#define QTR_HPP

#include <Arduino.h>
#include <stdint.h>
#include "mux.h"

class QTR
{
public:

    static constexpr uint8_t N = 8;

    // firstChannel as it states is where the QTR starts in the mux.
    // QTR qtrFront(0, mux); (C0..C7)
    // QTR qtrRear(8, mux);  (C8..C15)
    explicit QTR(uint8_t firstChannel, Mux74HC4067& mux);

    // Initializes the pin arrangement on the mux
    // Called once on setup()
    bool begin();

    // Loads preset calibration for the 8 sensors,
    void setCalibration(const uint16_t* minVals, const uint16_t* maxVals);

    // Calls setcalibration and loads the profile for each qtr from constants.h
    void useDefaultCalibration(uint8_t profile = 0);

    // Reads the sensor and updates the cache
    void update();

    // Pos on the array from 0 (extreme left) to 7000 (extreme right)
    int getPosition() const;

    // True if any sensor is over the threshold.
    // From 0 to 1000 after normalization
    bool onLine(uint16_t threshold = 200) const;

    // Function that performs calibration routine for 10 seconds
    void calibrate(uint32_t durationMs = 10000);

    //Prints the calibration values for each sensor
    // Useful for copying to constants.h.
    void printCalibration(const char* label) const;

    // Returns the position reading the qtr sensors as binary values
    // Where each bit represents whether the corresponding sensor is over the threshold or not
    // Uses already normalized values
    int getBinaryPosition() const;

    // Debug
    const uint16_t* getRaw()  const { return raw;  }

    const uint16_t* getNorm() const { return norm; }

    void debugPrint() const;

private:

    uint8_t         firstCh;
    bool            initialized;

    Mux74HC4067&    mux;

    // Raw values form the ADC
    uint16_t raw[N];
    // Minimum values per sensor
    uint16_t calMin[N];
    // Maximum values per sensor
    uint16_t calMax[N];
    // Normalized values
    uint16_t norm[N];

    // Final result for control.
    int position;

    void ensureCalValid();
};

#endif // QTR_HPP