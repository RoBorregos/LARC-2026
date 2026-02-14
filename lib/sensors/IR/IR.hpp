/**
 * @file IR.hpp
 * @date 2026-02-10
 *
 * @brief Sensores IR de línea para detección de salida de pista
 *
 * Nota: la salida digital puede venir invertida según el módulo/cableado.
 * En muchos sensores: NEGRO = LOW(0) y BLANCO = HIGH(1).
 * Ajusta "normal" según el comportamiento en pista.
 */

#ifndef IR_HPP
#define IR_HPP

#include <Arduino.h>

class IRLine
{
public:
    static constexpr uint8_t N = 4;

    enum Sensor : uint8_t
    {
        FL = 0, // Front Left
        FR = 1, // Front Right
        BL = 2, // Back Left
        BR = 3  // Back Right
    };

    // Constructor: pines en orden FL, FR, BL, BR
    IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin, bool normal = true);

    // Alternativa: arreglo {FL, FR, BL, BR}
    explicit IRLine(const uint8_t pins_[N], bool normal = true);

    // Configura pines como INPUT
    bool begin();

    // Lee sensores y actualiza estados internos
    void update();

    // Estado interpretado: true si detecta "línea" según normal
    bool getState(Sensor s) const;

    // Estado crudo del pin: HIGH/LOW (true=HIGH, false=LOW)
    bool getRaw(Sensor s) const;

    // Acceso al pin físico (debug)
    uint8_t getPin(Sensor s) const;

    // --- Compatibilidad (opcional): puedes borrar esto si ya migraste ---
    using Index = Sensor;
    bool onLine(Index i) const { return getState(i); }
    bool raw(Index i) const { return getRaw(i); }

private:
    bool initialized;
    bool normal;
    uint8_t pins[N];

    bool rawState[N];
    bool lineState[N];
};

#endif // IR_HPP

/*
Lo llamas así:

  bool fl = ir.getState(IRLine::FL); (Front Left)
  bool fr = ir.getState(IRLine::FR); (Front Right)
  bool bl = ir.getState(IRLine::BL); (Back Left)
  bool br = ir.getState(IRLine::BR); (Back Right)

*/