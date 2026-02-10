/**
 * @file IR.hpp
 * @date 2026-02-10
 *
 * @brief Sensores IR de línea para detección de salida de pista
 *
 * Nota: la salida digital puede venir invertida según el módulo/cableado.
 * En muchos sensores: NEGRO = LOW(0) y BLANCO = HIGH(1).
 * Modificar lineIsBlack según el comportamiento en pista.
 */

#ifndef IR_HPP
#define IR_HPP

#include <Arduino.h>
#include "pins.h"

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

    // lineIsBlack=true: considera "línea" cuando el sensor ve negro.
    explicit IRLine(bool lineIsBlack = true);

    // Configura los pines como INPUT
    bool begin();

    // Lee los sensores y actualiza estados internos
    void update();

    // Valor crudo del pin (HIGH/LOW)
    bool raw(Index i) const;

    // Valor interpretado: true si detecta "línea" según lineIsBlack
    bool onLine(Index i) const;

private:
    bool initialized;
    bool lineIsBlack;

    bool rawState[N];
    bool lineState[N];

    static uint8_t pinFor(Index i);
};
/*Ejemplo para llamarlo en code:
IRLine ir(true);   // línea negra

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