#include "Drive.hpp"

Drive::Drive()
  : bno_(),
    m1_ul_(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], false, UL_ENC_A, UL_ENC_B, kWheelDiameter),
    m2_ur_(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true,  UR_ENC_A, UR_ENC_B, kWheelDiameter),
    m3_ll_(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true,  LL_ENC_A, LL_ENC_B, kWheelDiameter),
    m4_lr_(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], false, LR_ENC_A, LR_ENC_B, kWheelDiameter),
    omni_(m1_ul_, m2_ur_, m3_ll_, m4_lr_),
    yawPid_(kKp, kKi, kKd, -kOmegaMax, +kOmegaMax)
{
}

void Drive::begin() {
  Serial.begin(115200);
  delay(1000);

  analogWriteResolution(8);

  m1_ul_.begin();
  m2_ur_.begin();
  m3_ll_.begin();
  m4_lr_.begin();

  Wire.begin();

  bno_.begin();
  bno_.update();
  delay(50);
  bno_.update();

  yawPid_.setAngleWrapping(true);
  yawPid_.reset();

  targetYaw_ = bno_.getYaw(); // rad
  transitionTo(ADELANTE);
}

void Drive::update() {
  const uint32_t now = millis();

  // ===== Control loop 100 Hz =====
  if (now - lastControl_ >= kControlMs) {
    lastControl_ = now;

    bno_.update();
    const float yaw = bno_.getYaw();          // rad
    float omega = yawPid_.update(yaw, targetYaw_);
    omega = clampf(omega, -kOmegaMax, +kOmegaMax);

    float vx = 0.0f, vy = 0.0f;

    if (demoEnabled_) {
      demoStateMachine(now);
      applyDemoVxVy(vx, vy);
    }

    omni_.MoveXYW(vx, vy, -omega);

    // ===== Print 10 Hz =====
    if (now - lastPrint_ >= kPrintMs) {
      lastPrint_ = now;

      float err = targetYaw_ - yaw;
      while (err >  M_PI) err -= 2.0f * M_PI;
      while (err < -M_PI) err += 2.0f * M_PI;

      Serial.print(now);
      Serial.print("\t");
      Serial.print(rad2deg(yaw), 2);





      
      Serial.print("\t\t");
      Serial.print(rad2deg(targetYaw_), 2);
      Serial.print("\t\t");
      Serial.print(rad2deg(err), 2);
      Serial.print("\t\t");
      Serial.print(omega, 4);
      Serial.print("\t");
      Serial.print(vx, 2);
      Serial.print("\t");
      Serial.println(vy, 2);
    }
  }
}

void Drive::setTargetYaw(float yawRad) {
  targetYaw_ = yawRad;
  yawPid_.reset();
}

float Drive::getYaw() const {
  return ((Drive*)this)->bno_.getYaw();
}

void Drive::enableDemo(bool enable) {
  demoEnabled_ = enable;
}

void Drive::allStop() {
  m1_ul_.stop();
  m2_ur_.stop();
  m3_ll_.stop();
  m4_lr_.stop();
}

float Drive::rad2deg(float r) { return r * (180.0f / M_PI); }

float Drive::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void Drive::transitionTo(uint8_t next) {
  state_ = (DemoState)next;
  stateStartMs_ = millis();
}

void Drive::demoStateMachine(uint32_t now) {
  if (now - stateStartMs_ > 1500) {
    switch (state_) {
      case ADELANTE:  transitionTo(STOP);      break;
      case STOP:      transitionTo(DERECHA);   break;
      case DERECHA:   transitionTo(STOP2);     break;
      case STOP2:     transitionTo(ATRAS);     break;
      case ATRAS:     transitionTo(STOP3);     break;
      case STOP3:     transitionTo(IZQUIERDA); break;
      case IZQUIERDA: transitionTo(STOP4);     break;
      case STOP4:     transitionTo(ADELANTE);  break;
    }
  }
}

void Drive::applyDemoVxVy(float &vx, float &vy) {
  vx = 0.0f; vy = 0.0f;

  switch (state_) {
    case ADELANTE:   vx =  0.35f; vy =  0.00f; break;
    case ATRAS:      vx = -0.35f; vy =  0.00f; break;
    case IZQUIERDA:  vx =  0.00f; vy = -0.35f; break;
    case DERECHA:    vx =  0.00f; vy =  0.35f; break;

    case STOP:
    case STOP2:
    case STOP3:
    case STOP4:
      allStop();
      vx = 0.0f; vy = 0.0f;
      break;
  }
}
