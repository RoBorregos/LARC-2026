#include "Drive.hpp"


Drive::Drive()
  : bno_(),
    m1_ul_(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], false, UL_ENC_A, UL_ENC_B, diameter),
    m2_ur_(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true,  UR_ENC_A, UR_ENC_B, diameter),
    m3_ll_(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true,  LL_ENC_A, LL_ENC_B, diameter),
    m4_lr_(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], false, LR_ENC_A, LR_ENC_B, diameter),
    omni_(m1_ul_, m2_ur_, m3_ll_, m4_lr_),
    yawPid_(P, I, D, -kOmegaMax, +kOmegaMax)
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

  // Starts on stop
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
      Serial.print("YAW: ");
      Serial.print(rad2deg(yaw), 2);
      Serial.print("\t\t");
      Serial.print("targuet yaw: ");
      Serial.print(rad2deg(targetYaw_), 2);
      Serial.print("\t\t");
      Serial.print("error");
      Serial.print(rad2deg(err), 2);
      Serial.print("\t\t");
      Serial.print("omega");
      Serial.print(omega, 4);
      Serial.print("\t");
      Serial.print("vxCmd");
      Serial.print(vxCmd_, 2);
      Serial.print("\t");
      Serial.print("vyCmd");
      Serial.println(vyCmd_, 2);
    }
  }
}

// ====== Funciones de movimiento ======
void Drive::left(float speed)  { vxCmd_ = +speed; vyCmd_ = 0.0f; }//before: forward
void Drive::backward(float speed) { vxCmd_ = -speed; vyCmd_ = 0.0f; }//before: backward
void Drive::backward(float speed)  { vxCmd_ = 0.0f; vyCmd_ = -speed; }   //before: left
void Drive::forward(float speed) { vxCmd_ = 0.0f; vyCmd_ = +speed; }   //before: right
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

float Drive::rad2deg(float r) { return r * (180.0f / M_PI); } //para los prints

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

float Drive::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


// Test kinematics
void Drive::testKinematics(float v, uint32_t T) {

  holdYaw(false);

  Serial.println("Forward (+vx)");
  omni_.MoveXYW(+v, 0.0f, 0.0f);
  delay(T);

  Serial.println("Backward (-vx)");
  omni_.MoveXYW(-v, 0.0f, 0.0f);
  delay(T);

  Serial.println("Right (+vy)");
  omni_.MoveXYW(0.0f, +v, 0.0f);
  delay(T);

  Serial.println("Left (-vy)");
  omni_.MoveXYW(0.0f, -v, 0.0f);
  delay(T);

  Serial.println("Diag (+vx,+vy)");
  omni_.MoveXYW(+v, +v, 0.0f);
  delay(T);

  Serial.println("Diag (-vx,-vy)");
  omni_.MoveXYW(-v, -v, 0.0f);
  delay(T);

  Serial.println("Diag (+vx,-vy)");
  omni_.MoveXYW(+v, -v, 0.0f);
  delay(T);

  Serial.println("Stop");
  omni_.Stop();
  delay(1200);
}

void Drive::setTranslation(float vx, float vy) {
  vxCmd_ = vx;
  vyCmd_ = vy;
}