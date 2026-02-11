#include "Drive.hpp"

Drive::Drive()
  : bno_(),
    m1_ul_(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], false, UL_ENC_A, UL_ENC_B, kWheelDiameter),
    m2_ur_(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true,  UR_ENC_A, UR_ENC_B, kWheelDiameter),
    m3_ll_(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true,  LL_ENC_A, LL_ENC_B, kWheelDiameter),
    m4_lr_(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], false, LR_ENC_A, LR_ENC_B, kWheelDiameter),
    omni_(m1_ul_, m2_ur_, m3_ll_, m4_lr_),
    yawPid_(P, kKi, kKd, -kOmegaMax, +kOmegaMax)
{
}

void Drive::begin() {
  Serial.begin(115200);
  delay(500);

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
  yawHoldEnabled_ = true;

  // Arranca detenido
  vxCmd_ = 0.0f;
  vyCmd_ = 0.0f;
}

void Drive::update() {
  const uint32_t now = millis();

  if (now - lastControl_ >= kControlMs) {
    lastControl_ = now;

    bno_.update();
    const float yaw = bno_.getYaw(); // rad

    float omega = 0.0f;

    if (manualOmegaEnabled_) {
      omega = manualOmega_;
    } else if (yawHoldEnabled_) {
      omega = yawPid_.update(yaw, targetYaw_);
    } else {
      omega = 0.0f;
    }

    omega = clampf(omega, -kOmegaMax, +kOmegaMax);


    omni_.MoveXYW(vxCmd_, vyCmd_, -omega);

    //Print 10 Hz <opcional>
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
      Serial.print(vxCmd_, 2);
      Serial.print("\t");
      Serial.println(vyCmd_, 2);
    }
  }
}

// ====== Funciones de movimiento ======
void Drive::forward(float speed)  { vxCmd_ = +speed; vyCmd_ = 0.0f; }
void Drive::backward(float speed) { vxCmd_ = -speed; vyCmd_ = 0.0f; }
void Drive::left(float speed)  { vxCmd_ = 0.0f; vyCmd_ = -speed; }
void Drive::right(float speed) { vxCmd_ = 0.0f; vyCmd_ = +speed; }
void Drive::stop() {
  vxCmd_ = 0.0f;
  vyCmd_ = 0.0f;
}


void Drive::holdYaw(bool enable) {
  yawHoldEnabled_ = enable;
  if (enable) {
    targetYaw_ = bno_.getYaw();
    yawPid_.reset();
  }
}

void Drive::setTargetYaw(float yawRad) {
  targetYaw_ = yawRad;
  yawPid_.reset();
}

float Drive::getYaw() const {
  return ((Drive*)this)->bno_.getYaw();
}

// ====== Omega manual opcional ======
void Drive::setManualOmega(float omegaRadS) {
  manualOmegaEnabled_ = true;
  manualOmega_ = omegaRadS;
}

void Drive::clearManualOmega() {
  manualOmegaEnabled_ = false;
  manualOmega_ = 0.0f;
  yawPid_.reset();
}

void Drive::allStop() {
  m1_ul_.stop();
  m2_ur_.stop();
  m3_ll_.stop();
  m4_lr_.stop();
}

float Drive::rad2deg(float r) { return r * (180.0f / M_PI); } //para los prints

float Drive::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
