/**
 * @file seeds.hpp
 * @date 2026-02-05
 *
 * @brief Clasificación de semillas usando rangos de lecturas crudas del TCS34725 (Clear, R, G, B)
 */

#ifndef SEEDS_HPP
#define SEEDS_HPP

#include <Arduino.h>
#include "rgb.hpp"

// Tipos de semilla (sin VERDE)
enum class SeedType : uint8_t {
    MADURO_ROJO = 0,
    MADURO_AMARILLO,
    MADURO_NARANJA,
    SOBREMADURO_AZUL,
    SOBREMADURO_NEGRO,

    UNKNOWN,// No cayó en ningún rango
    NOT_INITIALIZED // El sensor RGB no está inicializado
};

// Un rango de comparación para (Clear, Red, Green, Blue)
struct SeedRange {
    uint16_t cMin, cMax;
    uint16_t rMin, rMax;
    uint16_t gMin, gMax;
    uint16_t bMin, bMax;

    // Devuelve true si (c,r,g,b) cae dentro del rango.
    //si useClear=false, ignora cMin/cMax (clear).
    bool contains(uint16_t c,
                  uint16_t r,
                  uint16_t g,
                  uint16_t b,
                  bool useClear) const;
};

class SeedClassifier {
public:
    //Solo las clases reales (las primeras 5 del enum)
    static constexpr uint8_t N_TYPES = 5;

    SeedClassifier();

    // Lee el sensor y clasifica según los rangos definidos en Templates()
    SeedType classify(RGB& rgb, bool useClear = false);

    //Convierte SeedType a texto para debug (Serial)
    static const char* toString(SeedType type);

private:
    //rangos por clase (orden: rojo, amarillo, naranja, azul, negro)
    SeedRange ranges[N_TYPES];

    // Carga rangos hardcodeados (aquí se debe editar a mano cuando se tengan los rangos de pista)
    void Templates();
};

#endif // SEEDS_HPP