#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "pins.h"
#include "bno.hpp"
#include "motors.hpp"
#include "omni_motors.hpp"
#include "PIDController.hpp"

class Drive {
public:
  Drive();

  void begin();
  void update();

  // Opcional:
  void setTargetYaw(float yawRad);     // setpoint en radianes
  float getYaw() const;
  void allStop();

  void enableDemo(bool enable);

private:
  // Helpers
  static float rad2deg(float r);
  static float clampf(float x, float lo, float hi);

  void transitionTo(uint8_t next);   //estado
  void demoStateMachine(uint32_t now);
  void applyDemoVxVy(float &vx, float &vy);

private:
  // ============ Hardware ============
  BNO bno_;

  static constexpr float kWheelDiameter = 0.109f;


  static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];
  static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0];
  static constexpr uint8_t UR_ENC_A = Pins::kEncoders[2];
  static constexpr uint8_t UR_ENC_B = Pins::kEncoders[3];
  static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
  static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];
  static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
  static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];

  DCMotor m1_ul_;
  DCMotor m2_ur_;
  DCMotor m3_ll_;
  DCMotor m4_lr_;

  OmniMotors omni_;

  // ============ PID yaw-hold ============
  static constexpr float kKp = 4.8f;
  static constexpr float kKi = 0.002f;
  static constexpr float kKd = 0.06f;
  static constexpr float kOmegaMax = 0.25f;

  PIDController yawPid_;

  // ============ Timing ============
  static constexpr uint32_t kControlMs = 10;   // 100 Hz
  static constexpr uint32_t kPrintMs   = 100;  // 10 Hz

  uint32_t lastControl_ = 0;
  uint32_t lastPrint_   = 0;

  // ============ Setpoint ============
  float targetYaw_ = 0.0f; // rad

  // ============ Demo state machine ============
  bool demoEnabled_ = true;

  enum DemoState : uint8_t {
    ADELANTE,
    ATRAS,
    IZQUIERDA,
    DERECHA,
    STOP,
    STOP2,
    STOP3,
    STOP4
  };

  DemoState state_ = ADELANTE;
  uint32_t stateStartMs_ = 0;
};
