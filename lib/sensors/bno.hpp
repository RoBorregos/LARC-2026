/**
 * @file bno.hpp
 * @date 2026-01-28
 *
 * @brief BNO055 sensor de orientación (I2C)
 */

#ifndef BNO_HPP
#define BNO_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class BNO
{
public:
    // Constructor: por defecto usa ADR conectado a GND (address 0x28).
    // Si se llega a cambiar ADR a VCC, la address debe ser 0x29.
    explicit BNO(uint8_t address = 0x28);

    // Inicializa el sensor; retorna true si lo detecta.
    bool begin();

    // Lee y actualiza el cache de orientación.
    void update();

    // Yaw absoluto (rango [-180, 180])
    float getYawWrapped() const;
    // Yaw absoluto continuo (rango [0, 360))
    float getYaw360() const;

    // Aceleración lineal (sin gravedad) en m/s^2
    static constexpr uint8_t N_ACCEL = 3;
    void getLinearAcceleration(float out[N_ACCEL]);

    bool isInitialized() const { return initialized; }

private:
    Adafruit_BNO055 bno;
    sensors_event_t orientEvent;
    bool initialized;

    // Helper: wrap a [-180, 180]
    float wrapAngle(float deg) const;
};

#endif // BNO_HPP