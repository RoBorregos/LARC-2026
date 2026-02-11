/**
 * @file IR.hpp
 * @date 2026-02-10
 *
 * @brief Sensores IR de línea para detección de salida de pista
 *
 * Nota: la salida digital puede venir invertida según el módulo/cableado.
 * En muchos sensores: NEGRO = LOW(0) y BLANCO = HIGH(1).
 * Modificar "normal" según el comportamiento en pista.
 */

#ifndef IR_HPP
#define IR_HPP

#include <Arduino.h>

class IRLine
{
public:
    static constexpr uint8_t N = 4;

    enum Index : uint8_t
    {
        FL = 0, // Front Left
        FR = 1, // Front Right
        BL = 2, // Back Left
        BR = 3  // Back Right
    };

    // Constructor: recibe los pines (en orden FL, FR, BL, BR)
    IRLine(uint8_t flPin, uint8_t frPin, uint8_t blPin, uint8_t brPin, bool normal = true);

    // Alternativa: recibe un arreglo de 4 pines {FL, FR, BL, BR}
    explicit IRLine(const uint8_t pins_[N], bool normal = true);

    // Configura los pines como INPUT
    bool begin();

    // Lee los sensores y actualiza estados internos
    void update();

    // Valor crudo del pin (HIGH/LOW)
    bool raw(Index i) const;

    // Valor interpretado: true si detecta "línea" según normal
    bool onLine(Index i) const;

private:
    bool initialized;
    bool normal;
    uint8_t pins[N];

    bool rawState[N];
    bool lineState[N];
};
/*Ejemplo para llamarlo en code:
IRLine ir(Pins::kLineSensorFL, Pins::kLineSensorFR, Pins::kLineSensorBL, Pins::kLineSensorBR, true);   // línea negra

void setup() {
  ir.begin();
}

void loop() {
  ir.update();

  bool fl = ir.onLine(IRLine::FL);
  bool fr = ir.onLine(IRLine::FR);
  bool bl = ir.onLine(IRLine::BL);
  bool br = ir.onLine(IRLine::BR);

  // se decide después que hacer con cada esquina
}
*/
#endif // IR_HPP