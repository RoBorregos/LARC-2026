#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "constants.h"
#include "pins.h"
#include "BNO/bno.hpp"
#include "motors.hpp"
#include "omni_motors.hpp"
#include "PIDController.hpp"

//Constants
static constexpr float diameter = Constants::DriveConstants::kWheelDiameter;

static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];
static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0];
static constexpr uint8_t UR_ENC_A = Pins::kEncoders[2];
static constexpr uint8_t UR_ENC_B = Pins::kEncoders[3];
static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];
static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];


class Drive {
public:
  Drive();

  void begin();
  void update();

  //Motion functions
  void forward(float speed);     
  void backward(float speed);    
  void left(float speed);       
  void right(float speed);       
  void stop();                   

  // ====== Yaw hold ======
  void holdYaw(bool enable);
  void setTargetYaw(float yawRad);     // setpoint in radians
  float getYaw() const;

  // ====== Optional: send direct omega (without PID) ======
  void setManualOmega(float omegaRadS); 
  void clearManualOmega();  //go back to PID

  void allStop(); //brake
  
  void testKinematics(float v = 0.45f, uint32_t T = 2000);

private:
  // Helpers
  static float rad2deg(float r);
  static float clampf(float x, float lo, float hi);

  BNO bno_;

  DCMotor m1_ul_;
  DCMotor m2_ur_;
  DCMotor m3_ll_;
  DCMotor m4_lr_;

  OmniMotors omni_;

  // ============ PID yaw-hold ============
  static constexpr float P    = Constants::PID::kKp;
  static constexpr float I  = Constants::PID::kKi;
  static constexpr float D  = Constants::PID::kKd;
  static constexpr float kOmegaMax = Constants::PID::kOmegaMax;

  PIDController yawPid_;
  float targetYaw_ = 0.0f; // rad
  bool yawHoldEnabled_ = true;

  float vxCmd_ = 0.0f;
  float vyCmd_ = 0.0f;

  // ====== Omega manual (optional) ======
  bool manualOmegaEnabled_ = false;
  float manualOmega_ = 0.0f;

  // ============ Timing ============ 
  static constexpr uint32_t kControlMs = 10;   // 100 Hz
  static constexpr uint32_t kPrintMs   = 100;  // 10 Hz

  uint32_t lastControl_ = 0;
  uint32_t lastPrint_   = 0;
};
