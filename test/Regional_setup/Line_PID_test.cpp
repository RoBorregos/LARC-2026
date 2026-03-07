#include <Arduino.h>
#include "pins.h"
#include "mux.h"
#include "qtr.hpp"
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"
#include "constants.h"

Mux74HC4067 mux;
QTR qtrFront(0, mux);
Drive drive;
PIDController linePID(0.000035f, 0.0f, 0.00000008f, -1.0f, 1.0f);

static constexpr float kBaseSpeed = 0.45f;// strafing speed

void setup() {
  Serial.begin(115200);
  delay(500);

  mux.begin();
  qtrFront.begin();
  qtrFront.useDefaultCalibration();

  drive.begin();
  drive.holdYaw(true);
  drive.setTargetYaw(drive.getYaw());
  drive.holdYaw(true);
  drive.setTargetYaw(drive.getYaw());
}

void loop() {
  qtrFront.update();

  int linePos = qtrFront.getBinaryPosition();
  bool  onLine   = qtrFront.onLine();
  float lineErr  = linePos - Constants::LineFollower::kSetpoint;
  float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
  float vx       = -lineCorr;

  Serial.print("pos:");     Serial.print(linePos);
  Serial.print("\tonLine:"); Serial.print(onLine);
  Serial.print("\terr:");    Serial.print(lineErr);
  Serial.print("\tvx:");     Serial.println(vx);
  qtrFront.debugPrint();

  drive.setTranslation(vx, kBaseSpeed); //Change sign of kBaseSpeed to strafe right (+) or left (-)
  drive.update();

  delay(10);
}