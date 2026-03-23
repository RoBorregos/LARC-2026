
#include <Arduino.h>
#include "StateMachine.h"
#include "instances.h"
#include "constants.h"

namespace
{
    static constexpr float kVelocity = 0.40f;
    static constexpr float kObstacleDistanceCm = 20.0f;
    static constexpr float kBaseSpeed = 0.30f;

    static constexpr uint32_t kInitializedStoppedMs = 9000;
    static constexpr uint32_t kStartIgnoreTimeMs    = 1800;
    static constexpr uint32_t kClearDelayMs         = 300;
    static constexpr uint32_t kNoObstacleToCornerMs = 1500;
    static constexpr uint32_t kCornerDeployWaitMs   = 600;

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
    currentState = STATES::START;
    poolState = PoolSubState::FORWARD;

    state_start_time = millis();
    action_start_time = 0;
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    visionLeft = 0;
    visionRight = 0;
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

    const bool leftDetected        = (FL || BL);
    const bool frontDetected       = (FL || FR);
    const bool cornerLEFTDetected  = BL;
    const bool cornerRIGHTDetected = BR;

    const float d1 = us1.getdistance();
    const float d2 = us2.getdistance();

    const bool obstacle =
        (us1.isValid() && d1 < kObstacleDistanceCm) ||
        (us2.isValid() && d2 < kObstacleDistanceCm);

    switch (currentState)
    {
        case STATES::START:
            handleStartState(now);
            break;

        case STATES::POOL:
            handlePoolState(now, obstacle, leftDetected);
            break;

        case STATES::LOOKFORLINE:
            handleLookForLineState(frontDetected);
            break;

        case STATES::LOOKFORCORNER:
            handleLookForCornerState(now, cornerLEFTDetected, vx);
            break;

        case STATES::BEANS:
            handleBEANS(now, cornerRIGHTDetected, onLine, vx);
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
    const uint32_t elapsed = now - state_start_time;

    if (elapsed < kInitializedStoppedMs)
    {
        LARC.brake();
        elevator.ElevatorPosition(1);
    }
    else if (elapsed < (kInitializedStoppedMs + kStartIgnoreTimeMs))
    {
        elevator.ElevatorPosition(0);
        LARC.forward(kVelocity);
    }
    else
    {
        setState(STATES::POOL);
        setPoolState(PoolSubState::FORWARD);
    }
}

void LARCStateMachine::handlePoolState(uint32_t now, bool obstacle, bool leftDetected)
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

            if (leftDetected)
            {
                clearStartMs = 0;
                setPoolState(PoolSubState::AVOID_RIGHT);
            }
            else if (!obstacle)
            {
                if (clearStartMs == 0)
                {
                    clearStartMs = now;
                }

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

            break;
        }

        case PoolSubState::AVOID_RIGHT:
        {
            LARC.right(kVelocity);

            if (!obstacle)
            {
                if (clearStartMs == 0)
                {
                    clearStartMs = now;
                }

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

            break;
        }

        default:
            LARC.brake();
            break;
    }
}

void LARCStateMachine::handleLookForLineState(bool frontDetected)
{
    servos.intakeUpperHome();
    servos.intakeLowerHome();

    if (frontDetected)
    {
        setState(STATES::LOOKFORCORNER);
    }
    else
    {
        LARC.forward(kVelocity);
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

void LARCStateMachine::handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx)
{
    (void)now;

    if (cornerRIGHTDetected)
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