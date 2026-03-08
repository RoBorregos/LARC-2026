//Mini Maquinade Estados
#include <Arduino.h>
#include "pins.h"
#include "constants.h"
#include <math.h>

#include "IR_mux/IR_mux.hpp"
#include "mux.h"
#include "ultrasonic/ultrasonic.hpp"
#include "subsystem/Drive/Drive.hpp"

Drive LARC;
Mux74HC4067 mux;

static constexpr float velocity = Constants::PID::kcurrentVelocity; 


const uint8_t irChannels[IR_mux::N] = { 13, 12, 11, 10 };

// invertedMask = 0b0010 -> invierte el sensor índice 1 (normalmente FR)
IR_mux ir(mux, irChannels, 0b000);

enum LARC_State
{
    START, //Despliegue de elevador y omision de lectura de 
    POOL, //Avanzar {Ultrasonicos (prioridad) + IRs}
    LOOKFORLINE, // Forward until finds

};

LARC_State currentState = START;