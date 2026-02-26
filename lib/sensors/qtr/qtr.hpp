/**
 * @file qtr.hpp
 * @date 2026-01-28
 *
 * @brief QTR-8A (8-channel reflectance array) read through a shared 74HC4067 analog mux.
 *
 * Caso de uso: DOS arrays separados de QTR en el mismo mux.
 *  - QTR Frontal conectado de C0 a C7 (firstChannel = 0)
 *  - QTR Trasero conectado de C8 a C15 (firstChannel = 8)
 * La maquina de estados los puede usar independientemente
 */

#ifndef QTR_HPP
#define QTR_HPP

#include <Arduino.h>
#include <stdint.h>
#include "mux.h"

class QTR
{
public:
    //Característica compartida de los QTR (tienen 8 sensores)
    static constexpr uint8_t N = 8;

    // firstChannel es el canal donde empieza el mux, por ej.
    // QTR qtrFront(0, mux); (C0..C7)
    // QTR qtrRear(8, mux);  (C8..C15)
    explicit QTR(uint8_t firstChannel, Mux74HC4067& mux);

    // Inicializa los pines del mux y deja el QTR listo pa jalar.
    // Se llama UNA sola vez en setup()
    // Siempre antes de cualquier update() o lectura
    // void setup() {
    //  qtrFront.begin();
    //  qtrRear.begin();
    // }
    // Valor de retorno:
    // true = inicialización correcta
    // false = error (hardware no disponible, mala configuración, etc...)
    bool begin();

    // carga calibración predeterminada (fija) para los QTR.
    void setCalibration(const uint16_t* minVals, const uint16_t* maxVals);

    // llama a setCalibration y se carga el perfil de cada qtr
    // hardcodeado dentro del qtr.cpp
    void useDefaultCalibration(uint8_t profile = 0);

    // se leen los sensores y se actualizan el cache
    void update();

    // posición en el array de 0 a 7000
    int getPosition() const;

    // verdadero si cualquier sensor esta arriba del treshhold
    // de 0..1000 después de la normalización
    bool onLine(uint16_t threshold = 200) const;

    // Debug
    const uint16_t* getRaw()  const { return raw;  }
    const uint16_t* getNorm() const { return norm; }

    void debugPrint() const;

private:
    // donde empieza el qtr en el mux
    uint8_t         firstCh;
    bool            initialized;

    Mux74HC4067&    mux;

    // valores crudos del ADC
    uint16_t raw[N];
    // mínimos por sensor
    uint16_t calMin[N];
    // máximos por sensor
    uint16_t calMax[N];
    // valores normalizados
    uint16_t norm[N];

    // resultado final para control
    int position;

    void ensureCalValid();
};

#endif // QTR_HPP