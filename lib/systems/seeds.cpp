/**
 * @file seeds.cpp
 * @date 2026-02-06
 *
 * @brief Implementación de la clasificación de semillas usando rangos RGB (TCS34725)
 */

#include "seeds.hpp"

bool SeedRange::contains(uint16_t c, uint16_t r, uint16_t g, uint16_t b, bool useClear) const 
{
    if (useClear) {
        if (!(c >= cMin && c <= cMax))
            return false;
    }

    return (r >= rMin && r <= rMax) &&
           (g >= gMin && g <= gMax) &&
           (b >= bMin && b <= bMax);
}

SeedClassifier::SeedClassifier()
{
    Templates();
}

SeedType SeedClassifier::classify(RGB& rgb, bool useClear)
{
    if (!rgb.isInitialized())
        return SeedType::NOT_INITIALIZED;

    rgb.update();

    uint16_t raw[RGB::N_CHANNELS] = {0, 0, 0, 0};
    rgb.getRaw(raw);

    const uint16_t c = raw[0];
    const uint16_t r = raw[1];
    const uint16_t g = raw[2];
    const uint16_t b = raw[3];

    // Probar MADURO primero, luego SOBREMADURO
    for (uint8_t i = 0; i < N_TYPES; i++) {
        if (ranges[i].contains(c, r, g, b, useClear)) {
            return static_cast<SeedType>(i);
        }
    }

    return SeedType::UNKNOWN;
}

const char* SeedClassifier::toString(SeedType type)
{
    switch (type) {
        case SeedType::MADURO:
            return "Maduro";
        case SeedType::SOBREMADURO:
            return "Sobremaduro";
        case SeedType::NOT_INITIALIZED:
            return "Sensor no inicializado!";
        default:
            return "Desconocido";
    }
}

// PLACEHOLDERS!!

void SeedClassifier::Templates()
{
    //MADURO
    // Combina: Rojo, Amarillo, Naranja
    
    ranges[(int)SeedType::MADURO] = SeedRange{
        301, 65535,// C 
        80, 65535,// R
        0, 2400,// G 
        0, 1800 // B
    };

    // ===== SOBREMADURO =====
    // Combina: Azul, Negro
    ranges[(int)SeedType::SOBREMADURO] = SeedRange{
        0, 300, // C
        0, 80, // R
        0, 80, // G
        0, 80 // B:
    };
}