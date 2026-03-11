#include <Arduino.h>
#include "pins.h"
#include "mux.h"
#include "qtr.hpp"
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"
#include "constants.h"
#include "IR_mux.hpp"

Mux74HC4067 mux;
QTR qtrFront(0, mux);
Drive LARC;
PIDController linePID(0.000035f, 0.0f, 0.00000008f, -1.0f, 1.0f); 

const uint8_t irChannels[IR_mux::N] = {
    13,
    12,
    11,
    10
};
IR_mux ir(mux, irChannels, 0b0000);

static constexpr float kBaseSpeed = 0.30f;// strafing speed

void setup() {
  Serial.begin(115200);
  delay(500);

  mux.begin();
  qtrFront.begin();
  qtrFront.useDefaultCalibration();
  ir.begin();

  LARC.begin();
  LARC.holdYaw(true);
  LARC.setTargetYaw(LARC.getYaw());
}

void loop() {
  qtrFront.update();
  ir.update();

  if (ir.getState(IR_mux::BL) || ir.getState(IR_mux::BR)) {
    LARC.allStop();
    LARC.update();
    delay(200);
    return;
  }

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

  LARC.setTranslation(vx, -kBaseSpeed); //Change sign of kBaseSpeed to strafe LEFT (+) or RIGHT (-)
  LARC.update();

  delay(5);
}