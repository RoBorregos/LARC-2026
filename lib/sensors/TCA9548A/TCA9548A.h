#pragma once
#include <Arduino.h>
#include <Wire.h>

class TCA9548A {
public:
    /**
     * Constructor
     * @param address: Dirección I2C del TCA (default 0x70)
     * @param wire: Referencia al bus I2C (default Wire)
     */
    TCA9548A(uint8_t address = 0x70, TwoWire& wire = Wire);
    
    /**
     * Inicializar el multiplexor
     * @return true si el TCA responde
     */
    bool begin();
    
    /**
     * Seleccionar un canal (0-7)
     */
    void selectChannel(uint8_t channel);
    
    /**
     * Desactivar todos los canales
     */
    void disableAll();
    
    /**
     * Verificar si un dispositivo está presente en el canal actual
     * @param deviceAddress: Dirección I2C del dispositivo
     * @return true si el dispositivo responde
     */
    bool checkDevice(uint8_t deviceAddress);
    
    /**
     * Escanear un canal específico
     * @param channel: Canal a escanear (0-7)
     * @param callback: Función a llamar por cada dispositivo encontrado
     */
    void scanChannel(uint8_t channel, void (*callback)(uint8_t address));

    void scanAll();

    void scanDirect();

private:
    uint8_t address_;
    TwoWire& wire_;
    uint8_t currentChannel_;
};