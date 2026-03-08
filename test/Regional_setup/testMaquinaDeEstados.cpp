//Minimaquina <<Rutina completa>>
#include <Arduino.h>
#include <math.h>
#include "pins.h"
#include "constants.h"
//Sensors
#include "IR_mux/IR_mux.hpp"
#include "mux.h"
#include "qtr.hpp"
#include "ultrasonic/ultrasonic.hpp"
//Drive
#include "subsystem/Drive/Drive.hpp"
#include "PIDController.hpp"

//========================== OBJECTS ===========================
Drive LARC;
Mux74HC4067 mux;
//--------------------ends objects

//========================== VARIABLES ===========================

static constexpr float velocity = Constants::PID::kcurrentVelocity; 
static constexpr float velocity = 0.45f;
static constexpr float kObstacleDistanceCm = 15.0f;
static constexpr uint32_t kClearDelayMs = 300;
//--------------------ends variables


Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);
const uint8_t irChannels[IR_mux::N] = { 13, 12, 11, 10 };

//========================== SENSORS ===========================

// IRs
IR_mux ir(mux, irChannels, 0b0000); 
const uint8_t irChannels[IR_mux::N] = {13, 12, 11, 10};
/*
    *invertedMask = 0b0000 -> invierte el sensor índice 1 (normalmente FR)
    * Bit 0:FL, Bit 1: FR, Bit 2: BL, Bit 3: BR.
    * Ejemplo: 0b0011 invierte FL y FR.
*/

    // Ultrasonics
Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);
//--------------------ends sensors

enum LARC_State
{
    START, //Despliegue de elevador y omision de lectura de 
    POOL, //Avanzar {Ultrasonicos (prioridad) + IRs}
    LOOKFORLINE, // Forward until finds

};

LARC_State currentState = LARC_State::START;
static uint32_t clearStartMs = 0;
static bool obstacleHandled = false;

void setState(LARC_State newState)
{
    currentState = newState;
    clearStartMs = 0;
}

void setup()
{
Serial.begin(115200);
    delay(1000);

    // Drive class begin
    LARC.begin();
    LARC.holdYaw(true);
    LARC.setTargetYaw(LARC.getYaw());

    //Sensors begin
    ir.begin();
    us1.begin();
    us2.begin();

    //State machine begin
    currentState = LARC_State::START;

    Serial.println(F("TEST State Machine << complete rutine >> STARTS... now   "));
}

void loop()
{
    //update
    LARC.update();
    ir.update();
    us1.update();
    us2.update();
    ir.debugPrint();

    // IRs bools
    bool FL = ir.getState(IR_mux::FL);
    bool FR = ir.getState(IR_mux::FR);
    bool BL = ir.getState(IR_mux::BL);
    bool BR = ir.getState(IR_mux::BR);

    //Ultrasonics
    float d1 = us1.getdistance();
    float d2 = us2.getdistance();

    bool obstacle =
    (us1.isValid() && d1 < kObstacleDistanceCm) ||
    (us2.isValid() && d2 < kObstacleDistanceCm);

    //millis
    const uint32_t now = millis();

    switch (currentState)
    {

    }


}
