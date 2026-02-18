#include <Arduino.h>
#include "pins.h"
#include "constants.h"
#include "motors.hpp"

static constexpr float diameter = Constants::DriveConstants::kWheelDiameter;

//Encoders are not used yet
// Motor 1 (UL):
static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];
static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0];
// Motor 2 (UR):
static constexpr uint8_t UR_ENC_A = Pins::kEncoders[2];
static constexpr uint8_t UR_ENC_B = Pins::kEncoders[3];
// Motor 3 (LL):
static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];
// Motor 4 (LR):
static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];


// Crea los motores con tus pines
DCMotor m1_ul(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], true, UL_ENC_A, UL_ENC_B, diameter);
DCMotor m2_ur(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], false,  UR_ENC_A, UR_ENC_B, diameter);
DCMotor m3_ll(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true,  LL_ENC_A, LL_ENC_B, diameter);
DCMotor m4_lr(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], false, LR_ENC_A, LR_ENC_B, diameter);


void setup() {
  Serial.begin(115200);
  m1_ul.begin();
  m2_ur.begin();
  m3_ll.begin(); 
  m4_lr.begin(); 
  
  Serial.println("=== Motor test: forward/backward one-by-one ===");
}

void loop() {
  Serial.begin(115200);
  m1_ul.testForwardBackward();
  m2_ur.testForwardBackward();
  m3_ll.testForwardBackward(); 
  m4_lr.testForwardBackward(); 
}