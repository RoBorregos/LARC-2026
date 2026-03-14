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
#include "ServoSystem.hpp"

//Elevator
#include "Elevator.hpp"
//Drive
#include "subsystem/Drive/Drive.hpp"
#include "PIDController.hpp"

//========================== OBJECTS ===========================
Drive LARC;
Mux74HC4067 mux;
ServoSystem servos;
Elevator elevator;

//--------------------ends objects

//========================== VARIABLES ===========================

static constexpr float velocity = 0.40f; //velocidad a 0.30f
//State Machine
    // PoolSubstate     
    static bool obstacleHandled = false;

//Last print of servo instructions (to avoid spamming serial)
static uint32_t lastPrint = 0;

//--------------------ends variables


//========================== SENSORS ===========================

    // IRs
const uint8_t irChannels[IR_mux::N] = {13, 12, 11, 10};
IR_mux ir(mux, irChannels, 0b0000); 
        /*
        //IRs calibration description:
            *invertedMask = 0b0000 -> invierte el sensor índice 1 (normalmente FR)
            * Bit 0:FL, Bit 1: FR, Bit 2: BL, Bit 3: BR.
            * Ejemplo: 0b0011 invierte FL y FR.
        */

    // Ultrasonics
Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);

    // QTR
QTR qtrFront(0, mux);

//--------------------ends sensors

//============================ LINE PID ===============================

    // PID for line qtrFront
PIDController linePID(0.000035f, 0.0f, 0.00000008f, -1.0f, 1.0f); 

static constexpr float kBaseSpeed = 0.30f; // Strafing speed for line follower

//========================== STATE MACHINEs ===========================
enum LARC_STATE
{
    START,          //Despliegue de elevador y omision de lectura de 
    POOL,           //Avanzar {Ultrasonicos (prioridad) + IRs}
    LOOKFORLINE,    // Forward until finds a corner to initialize Line PID
    LOOKFORCORNER,  // Moves left with line pid until BL detects corner
    BEANS,          // line PID while strafing right
    STOP_TO_VISION, //on the corner to INITIALIZE vision
    STOP,           //Temporal  {when rightline detected stop}

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
    //PoolSubState
    PoolSubState poolState = PoolSubState::FORWARD;

//millis :: LARC_state
    //----- case START
static constexpr uint32_t kInitialzedStopped = 9000;  //Initialized -> Elevator up
static constexpr uint32_t kStartIgnoreTimeMs = 1800;  //ignore IRs reads

        //millis :: POOLS_substate
        static constexpr float kObstacleDistanceCm = 20.0f;
        uint32_t noObstacleStartMs = 0;
        static constexpr uint32_t kClearDelayMs = 300;
        static constexpr uint32_t kNoObstacleToCornerMs = 1500; //Transition to mainState :: LOOKFORLINE
        //--------------------------------------------------


// ========================== TIMERS ==========================
uint32_t stateStartMs = 0;
uint32_t clearStartMs = 0;

// ========================== HELPERS ==========================
void setMainState(LARC_STATE newState)
{
    currentStateLARC = newState;
    stateStartMs = millis();
    clearStartMs = 0;
    noObstacleStartMs = 0;
}

void setPoolState(PoolSubState newState)
{
    poolState = newState;
    clearStartMs = 0;
}

//--------------------ends machines states



// ========== VISION ==============================
static byte visionLeft  = 0;
static byte visionRight = 0;

void readVision()
{
    if (Serial.available() >= 3) {
        if (Serial.read() == 0xFF) {
            visionLeft  = Serial.read();
            visionRight = Serial.read();
        }
    }
}
//--------------------ends vision



//~~~~~~~~ SETUP && LOOP ~~~~~~~~~~~~~
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
    qtrFront.begin();
    qtrFront.useDefaultCalibration();
        //Elevator
    elevator.begin();

    //Servos begin
    servos.begin();

    //MAIN State machine begin
    setMainState(LARC_STATE::START);
    setPoolState(PoolSubState::FORWARD);

    //Vision
    servos.intakeUpperDeploy();
    servos.intakeLowerDeploy();
    delay(600);

    Serial.println(F("TEST State Machine << complete rutine >> STARTS... now   "));
}

void loop()
{
    //update
    LARC.update();
    ir.update();
    us1.update();
    us2.update();
    qtrFront.update();
    readVision();
    

    // Line PID variables

    int   linePos = qtrFront.getBinaryPosition();
    bool  onLine   = qtrFront.onLine();
    float lineErr  = linePos - Constants::LineFollower::kSetpoint;
    float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
    float vx       = -lineCorr;

    // IRs bools
    bool FL = ir.getState(IR_mux::FL);
    bool FR = ir.getState(IR_mux::FR);
    bool BL = ir.getState(IR_mux::BL);
    bool BR = ir.getState(IR_mux::BR);
    
    const bool leftDetected  = (FL || BL);
    const bool rightDetected = (FR || BR);
    const bool frontDetected = (FL || FR);
    const bool cornerLEFTDetected = (BL);
    const bool cornerRIGHTDetected = (BR);



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
            Serial.println(F("State: START"));
            if (now - stateStartMs < kInitialzedStopped)
            {
                LARC.brake();
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
        case PoolSubState::FORWARD:
        {
            Serial.println(F("State: POOL - FORWARD"));
            if (obstacle)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::AVOID_LEFT);
            }
            else
            {
                LARC.forward(velocity);

                if (noObstacleStartMs == 0)
                    noObstacleStartMs = now;

                if (now - noObstacleStartMs >= kNoObstacleToCornerMs)
                {
                    setMainState(LARC_STATE::LOOKFORLINE);
                }
            }
            break;
        }

        case PoolSubState::AVOID_LEFT:
        {
            Serial.println(F("State: POOL - AVOID_LEFT"));
            LARC.left(velocity);

            if (leftDetected)
            {
                clearStartMs = 0;
                setPoolState(PoolSubState::AVOID_RIGHT);
            }
            else if (!obstacle)
            {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if (now - clearStartMs >= kClearDelayMs)
                {
                    noObstacleStartMs = 0;
                    setPoolState(PoolSubState::FORWARD);
                }
            }
            else
            {
                clearStartMs = 0;
            }

            break;
        }

        case PoolSubState::AVOID_RIGHT:
        {
            Serial.println(F("State: POOL - AVOID_RIGHT"));
            LARC.right(velocity);

            if (!obstacle)
            {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if (now - clearStartMs >= kClearDelayMs)
                {
                    noObstacleStartMs = 0;
                    setPoolState(PoolSubState::FORWARD);
                }
            }
            else
            {
                clearStartMs = 0;
            }

            break;
        }

        default:
            break;
    }

    break;
}

        case LARC_STATE::LOOKFORLINE:
        {
            servos.intakeUpperHome();
            servos.intakeLowerHome();
            if (frontDetected)
            {
                Serial.println(F("State: LOOKFORLINE - front detected, go find corner"));
                setMainState(LARC_STATE::LOOKFORCORNER);
            }
            else
            {
                Serial.println(F("State: LOOKFORLINE - moving forward"));
                LARC.forward(velocity);
            }
            break;
        }

        case LARC_STATE::LOOKFORCORNER:

        {
            Serial.println(F("State: LOOKFORCORNER - moving left"));

            if (cornerLEFTDetected)  // BL detects the corner
            {
                Serial.println(F("State: LOOKFORCORNER - corner found, starting BEANS"));
                LARC.brake();
                servos.intakeUpperDeploy();
                servos.intakeLowerDeploy();
                delay(600);
                setMainState(LARC_STATE::BEANS);

            }
            else
            {
                LARC.setTranslation(vx, kBaseSpeed);
            }
            break;
        }

        case LARC_STATE::BEANS:
        {
            // Stop if boundary sensors detect edge
            if (cornerRIGHTDetected)
            {
                Serial.println(F("State: BEANS - right boundary, stopping"));
                setMainState(LARC_STATE::STOP);
                break;
            }

            if (!onLine)
            {
                LARC.stop();
                    if (visionLeft)  servos.intakeUpperDeploy();
                    else servos.intakeUpperHome();

                    if (visionRight) servos.intakeLowerDeploy();
                    else servos.intakeLowerHome();

            }
            else
            {
                // vx = QTR correction, -kBaseSpeed = strafe right
                Serial.print(F("State: BEANS - on line, vx: ")); Serial.println(vx);
                LARC.setTranslation(vx, -kBaseSpeed);
            }
            break;
        }



        case LARC_STATE::STOP:  
        {
            LARC.brake(); //Provicional
            Serial.println(F("State: STOP"));
            delay(600);
            break;
        }

        default:
            break;
    }
}