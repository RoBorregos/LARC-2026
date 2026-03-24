
#include <Arduino.h>
#include "StateMachine.h"
#include "instances.h"
#include "constants.h"

namespace
{
    static constexpr float kVelocity = 0.42f;
    static constexpr float kObstacleDistanceCm = 25.0f;
    static constexpr float kBaseSpeed = 0.42f;


    static constexpr uint32_t kInitializedStoppedMs = 9000;
    static constexpr uint32_t kStartIgnoreTimeMs    = 1800; //Time to ignore IR's at the START point
    static constexpr uint32_t kClearDelayMs         = 400; //Tiempo para cambiar nuevamente a Forward
    static constexpr uint32_t kNoObstacleToCornerMs = 1500; //Time without obstacle to go forward and LOOKFORLINE -> tal vez disminuir
    static constexpr uint32_t kCornerDeployWaitMs   = 600;

    static constexpr uint32_t kMinAvoidTimeMs   = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;


    const __FlashStringHelper* mainStateName(STATES state)
    {
        switch (state)
        {
            case STATES::START:         return F("START");
            case STATES::POOL:          return F("POOL");
            case STATES::LOOKFORLINE:   return F("LOOKFORLINE");
            case STATES::LOOKFORCORNER: return F("LOOKFORCORNER");
            case STATES::BEANS:         return F("BEANS");
            case STATES::STOP:          return F("STOP");
            default:                    return F("UNKNOWN");
        }
    }

    const __FlashStringHelper* poolStateName(PoolSubState state)
    {
        switch (state)
        {
            case PoolSubState::FORWARD:     return F("FORWARD");
            case PoolSubState::AVOID_LEFT:  return F("AVOID_LEFT");
            case PoolSubState::AVOID_RIGHT: return F("AVOID_RIGHT");
            default:                        return F("UNKNOWN");
        }
    }
}

LARCStateMachine::LARCStateMachine()
{
}

void LARCStateMachine::begin()
{
    currentState = STATES::START; // siempre en START
    poolState = PoolSubState::FORWARD;

    state_start_time = millis();
    action_start_time = 0;
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    visionLeft = 0;
    visionRight = 0;

    pinMode(limitSwitch, INPUT_PULLUP);
}

void LARCStateMachine::update()
{
    LARC.update();
    ir.update();
    us1.update();
    us2.update();
    qtrFront.update();
    readVision();

    const uint32_t now = millis();
    startStateTime();

    const int   linePos  = qtrFront.getBinaryPosition();
    const bool  onLine   = qtrFront.onLine();
    const float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
    const float vx       = -lineCorr;

    const bool FL = ir.getState(IR_mux::FL);
    const bool FR = ir.getState(IR_mux::FR);
    const bool BL = ir.getState(IR_mux::BL);
    const bool BR = ir.getState(IR_mux::BR);

    ir.debugPrint();

    const bool leftDetectedPool   = (FL || BL);
    const bool rightDetectedPool  = (FR || BR);

    const bool leftDetectedLine   = FL; //Aso used for corner
    const bool rightDetectedLine  = FR; 
    const bool frontDetectedLine  = (FL && FR);  // Hacer que con el qtr tambien detecte linea


    const float d1 = us1.getdistance();
    const float d2 = us2.getdistance();

    const bool obstacleLeftNow  = us1.isValid() && d1 < kObstacleDistanceCm;
    const bool obstacleRightNow = us2.isValid() && d2 < kObstacleDistanceCm;

    static bool obstacleLatched = false;
    static uint32_t obstacleClearStartMs = 0;
    static constexpr uint32_t kObstacleReleaseMs = 200;

    if (!obstacleLatched)
    {
        if (obstacleLeftNow || obstacleRightNow)
        {
            obstacleLatched = true;
            obstacleClearStartMs = 0;
        }
    }
    else
    {
        if (obstacleLeftNow || obstacleRightNow)
        {
            obstacleClearStartMs = 0;
        }
        else
        {
            if (obstacleClearStartMs == 0)
                obstacleClearStartMs = now;

            if ((now - obstacleClearStartMs) >= kObstacleReleaseMs)
            {
                obstacleLatched = false;
                obstacleClearStartMs = 0;
            }
        }
    }

    const bool obstacle = obstacleLatched;

    /*
    Serial.print("US izquierda: ");
    Serial.print(d1);
    Serial.print(" valid1: ");
    Serial.print(us1.isValid());

    Serial.print(" | US derecha: ");
    Serial.print(d2);
    Serial.print(" valid2: ");
    Serial.print(us2.isValid());

    Serial.print(" | obstacle: ");
    Serial.println(obstacle);
    */
    
    switch (currentState)
    {
        case STATES::START:
            handleStartState(now);
            break;

    case STATES::POOL:
        handlePoolState(now, obstacle, leftDetectedPool, rightDetectedPool);
        break;

    case STATES::LOOKFORLINE:
        handleLookForLineState(now, frontDetectedLine, leftDetectedLine, rightDetectedLine);
        break;

        case STATES::LOOKFORCORNER:
            handleLookForCornerState(now, leftDetectedLine, vx);
            break;

        case STATES::BEANS:
            handleBEANS(now, leftDetectedLine, onLine, vx);
            break;

        case STATES::STOP:
            handleStopState();
            break;

        default:
            handleStopState();
            break;
    }
}

void LARCStateMachine::setState(STATES newState)
{
    if (currentState == newState) return;

    currentState = newState;
    state_start_time = millis();
    action_start_time = 0;
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    Serial.print(F("Main state -> "));
    Serial.println(mainStateName(currentState));
}

void LARCStateMachine::setPoolState(PoolSubState newState)
{
   if (poolState == newState) return;

    poolState = newState;
    clearStartMs = 0;
    sideDetectStartMs = 0;
    poolStateStartMs = millis();

    Serial.print(F("Pool substate -> "));
    Serial.println(poolStateName(poolState));
}

void LARCStateMachine::startStateTime()
{
    if (state_start_time == 0)
    {
        state_start_time = millis();
    }
}

void LARCStateMachine::readVision()
{
    if (Serial.available() >= 3)
    {
        if (Serial.read() == 0xFF)
        {
            visionLeft  = Serial.read();
            visionRight = Serial.read();
        }
    }
}

void LARCStateMachine::handleStartState(uint32_t now)
{
    const bool limitPressed = (digitalRead(limitSwitch) == HIGH); // invertido

    if (limitPressed != lastLimitPressed)
    {
        if (limitPressed)
            Serial.println("LIMIT SWITCH PRESIONADO");
        else
            Serial.println("LIMIT SWITCH LIBERADO");

        lastLimitPressed = limitPressed;
    }

    switch (action_stage)
    {
        case 0:
            // Baja hasta tocar limit
            if (!limitPressed)
            {
                elevator.ElevatorPosition(2); // bajar
                LARC.brake();
            }
            else
            {
                elevator.ElevatorPosition(0);
                LARC.brake();
                Serial.println("ELEVADOR ABAJO -> STOP 2000 ms");
                action_start_time = now;
                action_stage = 1;
            }
            break;

        case 1:
            // Stop 2000 ms
            elevator.ElevatorPosition(0);
            LARC.brake();

            if ((now - action_start_time) >= 2000)
            {
                Serial.println("ELEVADOR SUBIENDO 9000 ms");
                action_start_time = now;
                action_stage = 2;
            }
            break;

        case 2:
            // Subir 9000 ms
            elevator.ElevatorPosition(1);
            LARC.brake();

            if ((now - action_start_time) >= 9000)
            {
                elevator.ElevatorPosition(0);
                LARC.brake();
                Serial.println("ELEVADOR DETENIDO -> STOP 3000 ms");
                action_start_time = now;
                action_stage = 3;
            }
            break;

        case 3:
            // Stop 3000 ms
            elevator.ElevatorPosition(0);
            LARC.brake();

            if ((now - action_start_time) >= 3000)
            {
                Serial.println("START IGNORANDO IR Y AVANZANDO");
                action_start_time = now;
                action_stage = 4;
            }
            break;

        case 4:
            // Avanza recto ignorando IR por unos segundos
            elevator.ElevatorPosition(0);
            LARC.forward(kVelocity);

            if ((now - action_start_time) >= kStartIgnoreTimeMs)
            {
                setPoolState(PoolSubState::FORWARD);
                setState(STATES::POOL);
                Serial.println("START -> POOL");
            }
            break;
    }
}

void LARCStateMachine::handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected)
{
    switch (poolState)
    {
        case PoolSubState::FORWARD:
        {
            if (obstacle)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::AVOID_LEFT);
            }
            else
            {
                LARC.forward(kVelocity);

                if (noObstacleStartMs == 0)
                {
                    noObstacleStartMs = now;
                }

                if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
                {
                    setState(STATES::LOOKFORLINE);
                }
            }
            break;
        }

        case PoolSubState::AVOID_LEFT:
{
    LARC.left(kVelocity);

    const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

    if (leftDetected && canChangeSide)
    {
        if (sideDetectStartMs == 0)
            sideDetectStartMs = now;

        if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_RIGHT);
        }
    }
    else
    {
        sideDetectStartMs = 0;

        if (!obstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }
    }

    break;
}

    case PoolSubState::AVOID_RIGHT:
{
    LARC.right(kVelocity);

    const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

    if (rightDetected && canChangeSide)
    {
        if (sideDetectStartMs == 0)
            sideDetectStartMs = now;

        if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_LEFT);
        }
    }
    else
    {
        sideDetectStartMs = 0;

        if (!obstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }
    }

    break;

            }
        }
    }

void LARCStateMachine::handleLookForLineState(uint32_t now, bool frontDetected, bool leftDetected, bool rightDetected)
{
    servos.intakeUpperHome();
    servos.intakeLowerHome();

    switch (action_stage)
    {
        case 0:
            if (leftDetected && !rightDetected)
            {
                Serial.println("LOOKFORLINE: izquierda detectada -> corrige derecha");
                action_stage = 1;
                action_start_time = now;
                return;
            }

            if (rightDetected && !leftDetected)
            {
                Serial.println("LOOKFORLINE: derecha detectada -> corrige izquierda");
                action_stage = 2;
                action_start_time = now;
                return;
            }

            if (frontDetected)
            {
                setState(STATES::LOOKFORCORNER);
                return;
            }

            LARC.forward(kVelocity);
            break;

        case 1:
            LARC.right(kVelocity);

            if ((now - action_start_time) >= 500)
            {
                action_stage = 0;
            }
            break;

        case 2:
            LARC.left(kVelocity);

            if ((now - action_start_time) >= 500)
            {
                action_stage = 0;
            }
            break;
    }
}

void LARCStateMachine::handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx)
{
    if (action_stage == 0)
    {
        if (cornerLEFTDetected)
        {
            LARC.brake();
            servos.intakeUpperDeploy();
            servos.intakeLowerDeploy();

            action_stage = 1;
            action_start_time = now;
        }
        else
        {
            LARC.setTranslation(vx, kBaseSpeed);
        }
    }
    else if (action_stage == 1)
    {
        LARC.brake();

        if ((now - action_start_time) >= kCornerDeployWaitMs)
        {
            setState(STATES::BEANS);
        }
    }
}

void LARCStateMachine::handleBEANS(uint32_t now, bool cornerLEFTDetected, bool onLine, float vx)
{
    (void)now;

    if (cornerLEFTDetected)
    {
        setState(STATES::STOP);
        return;
    }

    if (!onLine)
    {
        LARC.stop();
        return;
    }

    LARC.setTranslation(vx, -kBaseSpeed);

    if (visionLeft)
        servos.intakeUpperDeploy();
    else
        servos.intakeUpperHome();

    if (visionRight)
        servos.intakeLowerDeploy();
    else
        servos.intakeLowerHome();
}

void LARCStateMachine::handleStopState()
{
    LARC.brake();
}