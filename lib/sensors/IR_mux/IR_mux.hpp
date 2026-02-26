/**
 * @file IRLine.hpp
 * @date 2026-02-22
 *
 * @brief Sensores IR de línea analógicos a través de un 74HC4067.
 *        Cada sensor tiene su propio canal en el mux y su propio umbral.
 *        Si la lectura supera el umbral = línea detectada (true).
 */

#ifndef IRLINE_HPP
#define IRLINE_HPP

#include <Arduino.h>
#include "mux.h"

class IR_mux

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
     * @brief Constructor.
     *
     * @param mux_ Referencia al mux compartido (ya inicializado con begin())
     * @param channels_ Canales del mux para cada sensor en orden FL, FR, BL, BR.
     * @param invertedMask  Bitmask de sensores invertidos.
     * Bit 0=FL, Bit 1=FR, Bit 2=BL, Bit 3=BR.
     * Ejemplo: 0b0011 invierte FL y FR.
     * Por defecto 0b0000 (ninguno invertido).
     */
    IR_mux(Mux74HC4067& mux_, const uint8_t channels_[N],
           uint8_t invertedMask = 0b0000);


    /** @brief Inicializa el mux y hace la primera lectura. */
    bool begin();

    /** @brief Lee todos los sensores. Llamar periódicamente en el loop. */
    void update();

    /**
     * @brief Devuelve el estado lógico del sensor (ya aplicada la inversión).
     * true  = línea detectada
     * false = sin línea
     */
    bool getState(Sensor s) const;

    /** @brief Devuelve el valor analógico crudo (0..1023) del sensor. */
    uint16_t getRaw(Sensor s) const;

    void debugPrint() const;
    

private:
    Mux74HC4067& mux;

    // Canales del mux asignados a cada sensor (se definen en el constructor)
    uint8_t  channels[N];

    //Umbrales analógicos por sensor (0..1023)
    uint16_t threshold[N];

    // bit i = 1 sensor i está invertido lógicamente
    uint8_t  invertedMask;

    bool     initialized;
    uint16_t rawVal[N]; // lecturas crudas del ADC
    bool     lineState[N]; // estado lógico final (con inversión aplicada)
};

#endif // IRLINE_HPP