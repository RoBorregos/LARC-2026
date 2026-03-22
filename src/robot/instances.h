#pragma once
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

using namespace Constants;


extern Drive LARC;
extern Mux74HC4067 mux;
extern ServoSystem servos;
extern Elevator elevator;

extern const uint8_t irChannels[IR_mux::N];
extern IR_mux ir;
extern Ultrasonic us1;
extern Ultrasonic us2;
extern QTR qtrFront;

extern PIDController linePID;

extern uint32_t noObstacleStartMs;
extern uint32_t stateStartMs;
extern uint32_t clearStartMs;

constexpr float velocity = Constants::PID::kcurrentVelocity;
constexpr float kBaseSpeed = 0.30f;
constexpr uint32_t kInitialzedStopped = 9000;
constexpr uint32_t kStartIgnoreTimeMs = 1800;
constexpr float kObstacleDistanceCm = 20.0f;
constexpr uint32_t kClearDelayMs = 300;
constexpr uint32_t kNoObstacleToCornerMs = 1500;