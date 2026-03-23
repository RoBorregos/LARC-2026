#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include "constants.h"
enum class STATES
{
    START,
    POOL,
    LOOKFORLINE,
    LOOKFORCORNER,
    BEANS,
    STOP
};

enum class PoolSubState
{
    FORWARD,
    AVOID_LEFT,
    AVOID_RIGHT
};

class LARCStateMachine
{
public:
    LARCStateMachine();

    void begin();
    void update();

private:
    STATES currentState = STATES::START;
    PoolSubState poolState = PoolSubState::FORWARD;

    uint32_t state_start_time = 0;
    uint32_t action_start_time = 0;
    int action_stage = 0;

    uint32_t clearStartMs = 0;
    uint32_t noObstacleStartMs = 0;

    byte visionLeft = 0;
    byte visionRight = 0;

    void setState(STATES newState);
    void setPoolState(PoolSubState newState);
    void startStateTime();
    void readVision();

    void handleStartState(uint32_t now);
    void handlePoolState(uint32_t now, bool obstacle, bool leftDetected);
    void handleLookForLineState(bool frontDetected);
    void handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx);
    void handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx);
    void handleStopState();
};

#endif