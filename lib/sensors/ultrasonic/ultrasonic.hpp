/**
 * @brief Driver para sensores ultrasónicos tipo HC-SR04.
 ** @date 2026-02-11
 *
 *   casos de uso:
 *  - begin(): configura pines y deja el sensor listo.
 *  - update(): se llama seguido en loop(); NO debería bloquear el robot.
 *  - distanceCm(): devuelve la última distancia válida en cm.
 * - isValid(): indica si la última medición fue válida (eco recibido a tiempo).
 * 
 */

#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

#include <Arduino.h>


class Ultrasonic
{
public:
    /**
     * @brief Constantes de tiempo (PLACEHOLDERS)
     *
     * Mover estos valores al archivo de constantes globales
     * para hacerlos configurables por constructor.
     */
    static constexpr uint16_t kBeginSettleMs = 50; // (placeholder) tiempo tras begin()
    static constexpr uint16_t kPingPeriodMs = 60;// (placeholder) periodo mínimo entre mediciones para evitar crosstalk
    static constexpr uint16_t kEchoTimeoutUs = 7950; // (placeholder) timeout del eco (1.5m). Si se pasa = lectura inválida
    static constexpr uint8_t  kTrigLowUs = 2; // (placeholder) TRIG en LOW antes del pulso
    static constexpr uint8_t  kTrigHighUs = 10; // (placeholder) pulso TRIG HIGH (datasheet ~10us)

    explicit Ultrasonic(uint8_t trigPin, uint8_t echoPin);

    /**
     * Inicializa el sensor (solo una vez en setup).
     * - Configura trig como OUTPUT y echo como INPUT.
     * - Deja TRIG en LOW.
     * - Espera kBeginSettleMs para que el sensor se estabilice.
     */
    bool begin();

    /**
     * - Dispara un ping cada kPingPeriodMs
     * - Mide el pulso ECHO usando una pequeña máquina de estados
     * isValid() indica si la última medición fue válida.
     */
    void update();

    /** @return Distancia en cm de la última medición válida. */
    float getdistance() const;

    /**
     * @return true si la última medición fue válida.
     * Se pone false típicamente cuando:
     * - No llegó eco antes de kEchoTimeoutUs (fuera de rango / mala orientación)
     * - Problema de cableado o sensor desconectado
     */
    bool isValid() const;

private:
    enum class State : uint8_t
    {
        Idle,
        TrigLow,
        TrigHigh,
        WaitEchoRise,
        WaitEchoFall
    };

    uint8_t trig;
    uint8_t echo;

    // Máquina de estados
    State state;

    // Tiempos
    uint32_t lastPingMs;// para espaciar mediciones (millis)
    uint32_t tStateUs;// marca de tiempo para estados (micros)
    uint32_t echoRiseUs; // inicio de pulso ECHO (micros)

    float distance;
    bool valid;
};

#endif