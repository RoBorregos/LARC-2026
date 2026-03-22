#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <Arduino.h>
#include <SoftwareSerial.h>
#include "instances.h"

enum class STATES
{
  START, // Represents the start of the robot
  POOL,
  LOOKFORLINE,
  LOOKFORCORNER,
  BEANS,
  STOP_TO_VISION,
  STOP
};

class LARCStateMachine
{
public:
  LARCStateMachine();

  void begin();
  void update();

  void setState(STATES newState);

private:
  STATES currentState = STATES::START;

  // Used for general timing within states (i.e after 3 seconds from state start, do X)
  unsigned long state_start_time = 0;

  // Used for multi-stage actions within states (i.e start an action triggered by a sensor within a state, and then after some time do another action)
  unsigned long action_start_time = 0;
  int action_stage = 0;

  std::vector<int> values = {0, 0};
  unsigned long last_coffee_value_received = 0;
  static constexpr unsigned long recive_coffee_values_interval = 800;

  //  --- Switch State  ---
  void handleStartState();
  void handlePoolState();
  void handleLookForLine();
  void handleLookForCornerState();
  void handleBEANS();
  void handleStopToVisionState();
  void handleStopState();
  void handleForwardState();

  void handleState();       //Siwtch state
  void startStateTime();    //Millis
};

#endif // STATEMACHINE_HPP