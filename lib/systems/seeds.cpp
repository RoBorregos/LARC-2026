/**
 * @file seeds.cpp
 * @date 2026-02-06
 *
 * @brief Implementación de la clasificación de semillas usando rangos RGB (TCS34725)
 */

#include "seeds.hpp"

bool SeedRange::contains(uint16_t c, uint16_t r, uint16_t g, uint16_t b, bool useClear) const 
{
    if (useClear)
    {
        // Si el brillo no está dentro del rango, reject
        if (!(c >= cMin && c <= cMax))
            return false;
    }

    //coincide si R, G y B caen dentro de sus rangos
    return (r >= rMin && r <= rMax) &&
           (g >= gMin && g <= gMax) &&
           (b >= bMin && b <= bMax);
}

// clasificador de semillas
SeedClassifier::SeedClassifier()
{
    Templates();
}

SeedType SeedClassifier::classify(RGB& rgb, bool useClear)
{
    if (!rgb.isInitialized())
        return SeedType::NOT_INITIALIZED;

    // Actualiza lectura del sensor
    rgb.update();

    // raw[0]=Clear, raw[1]=Red, raw[2]=Green, raw[3]=Blue
    uint16_t raw[RGB::N_CHANNELS] = {0, 0, 0, 0};
    rgb.getRaw(raw);

    const uint16_t c = raw[0];
    const uint16_t r = raw[1];
    const uint16_t g = raw[2];
    const uint16_t b = raw[3];

    // Prueba cada rango en orden: Rojo, Amarillo, Naranja, Azul, Negro
    for (uint8_t i = 0; i < N_TYPES; i++)
    {
        if (ranges[i].contains(c, r, g, b, useClear))
        {
            return static_cast<SeedType>(i);
        }
    }

    return SeedType::UNKNOWN;
}

/**
 * Texto para debug (Serial) de cada tipo de semilla. 
 * Ej.
 * SeedType t = classifier.classify(rgb);
 * Serial.println(SeedClassifier::toString(t));
 */
const char* SeedClassifier::toString(SeedType type)
{
    switch (type)
    {
    case SeedType::MADURO_ROJO:
        return "Maduro (Rojo)";
    case SeedType::MADURO_AMARILLO:
        return "Maduro (Amarillo)";
    case SeedType::MADURO_NARANJA:
        return "Maduro (Naranja)";
    case SeedType::SOBREMADURO_AZUL:
        return "Sobremaduro (Azul)";
    case SeedType::SOBREMADURO_NEGRO:
        return "Sobremaduro (Negro)";
    case SeedType::NOT_INITIALIZED:
        return "Sensor no inicializado!";
    default:
        return "Desconocido";
    }
}

void SeedClassifier::Templates()
{
    // Cambiar estos valores manualmente cuando se tengan los rangos medidos en pista.
    //si useClear=false, cMin/cMax se ignoran.

    // Maduro (Rojo)
    ranges[(int)SeedType::MADURO_ROJO] = SeedRange{
        0, 65535,      // C
        1200, 65535,   // R
        0, 1800,       // G
        0, 1800        // B
    };

    // Maduro (Amarillo)
    ranges[(int)SeedType::MADURO_AMARILLO] = SeedRange{
        0, 65535,
        900, 65535,
        900, 65535,
        0, 1600
    };

    // Maduro (Naranja)
    ranges[(int)SeedType::MADURO_NARANJA] = SeedRange{
        0, 65535,
        1100, 65535,
        400, 2400,
        0, 1600
    };

    // Sobremaduro (Azul)
    ranges[(int)SeedType::SOBREMADURO_AZUL] = SeedRange{
        0, 65535,
        0, 2200,
        0, 2200,
        1000, 65535
    };

    // Sobremaduro (Negro)
    ranges[(int)SeedType::SOBREMADURO_NEGRO] = SeedRange{
        0, 65535,
        0, 600,
        0, 600,
        0, 600
    };
}