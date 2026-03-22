#include "StateMachine.h"

LARCStateMachine::LARCStateMachine()
{

}

void LARCStateMachine::begin()
{
    currentState = STATES::START;
    state_start_time = 0;
}

void LARCStateMachine::update()
{
    startStateTime();
    switch (currentState)
    {
        case STATES::START:
        handleForwardState();
        break;

        case STATES::POOL:
        handlePoolState();
        break;

        case STATES::LOOKFORLINE:
        handleLookForLine();
        break;

        case STATES::LOOKFORCORNER:
        handleLookForCornerState();
        break;
        
        case STATES::BEANS:
        handleBEANS();
        break;

        case STATES::STOP_TO_VISION:
        handleStopToVisionState();
        break;

        case STATES::STOP:
        handleLookForCornerState();
        break;

    }
}

void LARCStateMachine::setState(STATES newState)
{
    state_start_time = 0;
    action_start_time = 0;
    action_stage = 0;
    currentState = newState;
}

void LARCStateMachine::startStateTime()
{
    if (state_start_time == 0)
    {
        state_start_time = millis();
    }
}

void LARCStateMachine::handleStartState()
{
            

}

void LARCStateMachine::handlePoolState()
{
    
}

void LARCStateMachine::handleLookForCornerState()
{
    
}

void LARCStateMachine::handleBEANS()
{
    
}


void LARCStateMachine::handleStopToVisionState()
{
    
}

void LARCStateMachine::handleStopState()
{
    
}

void LARCStateMachine::handleForwardState()
{
    
}

void LARCStateMachine::handleState()
{
    
}


