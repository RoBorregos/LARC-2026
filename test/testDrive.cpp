#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "pins.h"
#include "bno.hpp"
#include "motors.hpp"
#include "omni_motors.hpp"
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"

Drive drive;

enum Step : uint8_t {
  FWD,
  STOP1,
  RIGHT,
  STOP2,
  BACK,
  STOP3,
  LEFT,
  STOP4
};

static Step step = FWD;
static uint32_t stepStart = 0;

static constexpr uint32_t kStepMs = 1500; // 1.5s cada paso
static constexpr float kSpeed = 0.35f;

void setStep(Step s) {
  step = s;
  stepStart = millis();

  drive.setTargetYaw(drive.getYaw());
}

void setup() {
  Serial.begin(115200);
  delay(500);

  drive.begin();
  drive.holdYaw(true);         // se usa el PID para mantener yaw
  drive.setTargetYaw(drive.getYaw());

  setStep(FWD);
}

void loop() {
  //Actualiza drive <<<BNO + PID + motores>>>
  drive.update();

  // Logica de secuencia (demo)
  uint32_t now = millis();
  if (now - stepStart > kStepMs) {
    switch (step) {
      case FWD:   setStep(STOP1); break;
      case STOP1: setStep(RIGHT); break;
      case RIGHT: setStep(STOP2); break;
      case STOP2: setStep(BACK);  break;
      case BACK:  setStep(STOP3); break;
      case STOP3: setStep(LEFT);  break;
      case LEFT:  setStep(STOP4); break;
      case STOP4: setStep(FWD);   break;
    }
  }

  switch (step) {
    case FWD:   drive.forward(kSpeed);  break;
    case BACK:  drive.backward(kSpeed); break;
    case LEFT:  drive.left(kSpeed);     break;
    case RIGHT: drive.right(kSpeed);    break;

    case STOP1: drive.stop(); break;
    case STOP2: drive.stop(); break;
    case STOP3: drive.stop(); break;
    case STOP4: drive.stop(); break;
  }
}


