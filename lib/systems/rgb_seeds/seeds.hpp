#pragma once
#include <Arduino.h>
#include "rgb.hpp"

enum class SeedType : uint8_t {
    MADURO = 0, // Rojo, Amarillo, Naranja combinados
    SOBREMADURO = 1,// Azul, Negro combinados
    NOT_INITIALIZED,
    UNKNOWN
};

/**
 * Rango RGB para clasificación
 */
struct SeedRange {
    uint16_t cMin, cMax; // Clear (brillo)
    uint16_t rMin, rMax;// Red
    uint16_t gMin, gMax;// Green
    uint16_t bMin, bMax;// Blue

    bool contains(uint16_t c, uint16_t r, uint16_t g, uint16_t b, bool useClear = false) const;
};

class SeedClassifier {
public:
    static constexpr uint8_t N_TYPES = 2;

    SeedClassifier();

    /**
     * Clasificar semilla según lectura RGB
     */
    SeedType classify(RGB& rgb, bool useClear = false);

    /**
     * Obtener texto descriptivo del tipo
     */
    static const char* toString(SeedType type);

private:
    SeedRange ranges[N_TYPES];
    void Templates();
};