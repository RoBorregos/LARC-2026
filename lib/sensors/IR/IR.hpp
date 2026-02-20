#pragma once

/**
 * @file IR.hpp
 * @date 2026-02-10
 *
 * @brief Declaración de sensores IR de línea con inversión por sensor
 */

#include <Arduino.h>

class IRLine
{
public:
    // Índices de sensores
    enum Sensor : uint8_t
    {
        FL = 0, // Front-Left
        FR = 1, // Front-Right
        BL = 2, // Back-Left
        BR = 3, // Back-Right
        N  = 4
    };

    /**
     * @brief Constructor con pines individuales.
     *
     * @param flPin Pin Front-Left
     * @param frPin Pin Front-Right
     * @param blPin Pin Back-Left
     * @param brPin Pin Back-Right
     * @param invertedMask  Bitmask de sensores invertidos.
     * Bit 0:FL, Bit 1: FR, Bit 2: BL, Bit 3: BR.
     * Ejemplo: 0b0011 invierte FL y FR.
     * Por defecto 0b1111 (todos invertidos).
     */
    IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin,
           uint8_t invertedMask = 0b1111);

    /**
     * @brief Constructor con arreglo de pines.
     *
     * @param pins_ Arreglo de 4 pines en orden FL, FR, BL, BR
     * @param invertedMask Bitmask de sensores invertidos (igual que el otro constructor)
     */
    IRLine(const uint8_t pins_[N], uint8_t invertedMask = 0b1111);

    /** @brief Inicializa los pines y hace la primera lectura. */
    bool begin();

    /** @brief Lee todos los sensores. Llamar periódicamente. */
    void update();

    /**
     * @brief Devuelve el estado lógico del sensor (ya aplicada la inversión).
     *true = línea
     *false = sin línea
     */
    bool getState(Sensor s) const;

private:
    bool    initialized;
    uint8_t pins[N];
    bool    rawState[N];
    bool    lineState[N];
    uint8_t invertedMask; // bit i = 1 = sensor i está invertido
};