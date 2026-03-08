//MiniMaquina <<Rutina completa>>
#include <Arduino.h>
#include <math.h>
#include "pins.h"
#include "constants.h"
//Sensors
#include "IR_mux/IR_mux.hpp"
#include "mux.h"
#include "qtr.hpp"
#include "ultrasonic/ultrasonic.hpp"
//Elevator
#include "Elevator.hpp"
//Drive
#include "subsystem/Drive/Drive.hpp"
#include "PIDController.hpp"

//========================== OBJECTS ===========================
Drive LARC;
Mux74HC4067 mux;
Elevator elevator;
//--------------------ends objects

//========================== VARIABLES ===========================

static constexpr float velocity = 0.45f; //velocidad a 0.30f
static constexpr float kObstacleDistanceCm = 15.0f;

//State Machine
    // PoolSubstate     
    static bool obstacleHandled = false;

//--------------------ends variables


//========================== SENSORS ===========================

    // IRs
const uint8_t irChannels[IR_mux::N] = {13, 12, 11, 10};
IR_mux ir(mux, irChannels, 0b0000); 
        /*
            *invertedMask = 0b0000 -> invierte el sensor índice 1 (normalmente FR)
            * Bit 0:FL, Bit 1: FR, Bit 2: BL, Bit 3: BR.
            * Ejemplo: 0b0011 invierte FL y FR.
        */

    // Ultrasonics
Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);
//--------------------ends sensors


//========================== STATE MACHINEs ===========================
enum LARC_STATE
{
    START, //Despliegue de elevador y omision de lectura de 
    POOL, //Avanzar {Ultrasonicos (prioridad) + IRs}
    LOOKFORCORNER, // Forward until finds a corner to init vision
    LINE_PID, // vision(on)
    STOP, //Temporal

};

enum class PoolSubState
{
    FORWARD,
    AVOID_LEFT,
    AVOID_RIGHT,
    WAIT_CLEAR,
    STOP,
};


//Main State Machine :: LARC_STATE

LARC_STATE currentStateLARC = LARC_STATE::START;
PoolSubState poolState = PoolSubState::FORWARD;

//millis :: LARC_state
    //----- case START
static constexpr uint32_t kInitialzedStopped = 3000;  //Initialized -> Elevator up
static constexpr uint32_t kStartIgnoreTimeMs = 1200;  //ignore IRs reads

        //millis :: POOLS_substate
        static constexpr uint32_t kClearDelayMs = 300;


// ========================== TIMERS ==========================
uint32_t stateStartMs = 0;
uint32_t clearStartMs = 0;

// ========================== HELPERS ==========================
void setMainState(LARC_STATE newState)
{
    currentStateLARC = newState;
    stateStartMs = millis();
    clearStartMs = 0;
}

void setPoolState(PoolSubState newState)
{
    poolState = newState;
    clearStartMs = 0;
}

//--------------------ends machines states



//~~~~~~~~~~~~~~~~~~~~~~~~ SETUP && LOOP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
        //Elevator
        elevator.begin();

    //MAIN State machine begin
    setMainState(LARC_STATE::START);
    setPoolState(PoolSubState::FORWARD);

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
    
    const bool leftDetected  = (FL || BL);
    const bool rightDetected = (FR || BR);
    const bool frontDetected = (FL && FR);

    //Ultrasonics
    float d1 = us1.getdistance();
    float d2 = us2.getdistance();

    bool obstacle =
    (us1.isValid() && d1 < kObstacleDistanceCm) ||
    (us2.isValid() && d2 < kObstacleDistanceCm);

    //  millis
    const uint32_t now = millis();


// ========================== STATE MACHINE ==========================
    switch (currentStateLARC)
    {
//MAIN::START
        case LARC_STATE::START:
        {
            if (now - stateStartMs < kInitialzedStopped)
            {
                LARC.allStop();
                elevator.ElevatorPosition(1); // UP
            }
            else if (now - stateStartMs < (kInitialzedStopped + kStartIgnoreTimeMs))
            {
                elevator.ElevatorPosition(0); // STOP
                LARC.forward(velocity);
            }
            else
            {
                setMainState(LARC_STATE::POOL);
                setPoolState(PoolSubState::FORWARD);
            }
            break;
        }

//MAIN::POOL
        case LARC_STATE::POOL:
        {
            switch (poolState)
            {
    // PoolSubstate :: FORWARD
                case PoolSubState::FORWARD:
                {
                    if (obstacleHandled )
                    {
                        setMainState(LARC_STATE::LOOKFORCORNER);
                    }
                    else if (obstacle)
                    {
                        setPoolState(PoolSubState::AVOID_LEFT);
                    }
                    else
                    {
                        LARC.forward(velocity);
                    }
                        break;
                        LARC.forward(velocity);
                break;
                }

                case PoolSubState::AVOID_LEFT:
                {
                    LARC.left(velocity);

                    if (!obstacle)
                    {
                        if (clearStartMs == 0)
                            clearStartMs = now;

                        if (now - clearStartMs >= kClearDelayMs)
                        {
                            obstacleHandled = true;
                            setMainState(LARC_STATE::LOOKFORCORNER);
                        }
                    }
                    else
                    {
                        clearStartMs = 0;
                    }
                
                    if (leftDetected) // LEFT lateral IRs detected
                    {
                        setPoolState(PoolSubState::AVOID_RIGHT);
                    }

                break;
                }

                case PoolSubState::AVOID_RIGHT:
                {
                    LARC.right(velocity);

                if (!obstacle)
                {
                    if (clearStartMs == 0)
                        clearStartMs = now;

                    if (now - clearStartMs >= kClearDelayMs)
                    {
                        obstacleHandled = true;
                        setMainState(LARC_STATE::LOOKFORCORNER);
                    }
                }
                else
                {
                    clearStartMs = 0;
                }
                break;
              }  //Ends PoolSubstate Avoidright

            }//Ends PoolSubstate

        break;
        }
    case LARC_STATE::LOOKFORCORNER:
        {
            LARC.forward(velocity);

            if (frontDetected)
            {
                setMainState(LARC_STATE::STOP);
            }
            break;
        }

        case LARC_STATE::LINE_PID:
        {
            LARC.allStop();
            break;
        }

        case LARC_STATE::STOP:
        {
            LARC.allStop();
            break;
        }

        default:
            break;
    }
}
