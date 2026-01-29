/**
 * @file tof.hpp
 * @date 2026-01-28
 *
 * @brief VL53L1X TOF sensor de distancia (I2C)
*/

#ifndef TOF_HPP
#define TOF_HPP

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

class ToF
{
public:
    // Distancia inválida si no hay lectura
    static constexpr uint16_t INVALID_MM = 0xFFFF;

    ToF();

    bool begin();
    void update();

    // Última distancia leída en mm
    uint16_t getDistanceMm() const { return distanceMm; }

    bool isInitialized() const { return initialized; }

    // Opcional: cambia el timing budget (precisión/velocidad)
    // Valores típicos: 20, 33, 50, 100 ms 
    //(depende de como funcione en pista)
    void setTimingBudgetMs(uint16_t ms);

    // Opcional: cambia el periodo entre mediciones
    // (si estás en modo continuo)
    // *** OJO, no puede ser menor que el timing budget ***
    void setInterMeasurementMs(uint16_t ms);

    // Opcional: arranca/parar modo continuo
    void startContinuous(uint16_t periodMs = 50);
    void stopContinuous();

private:
    VL53L1X sensor;
    bool initialized;
    bool continuous;

    uint16_t distanceMm;
};

#endif // TOF_HPP