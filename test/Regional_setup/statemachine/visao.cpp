// Vision + Servo + LinePID strafe test

#include <Arduino.h>
#include "ServoSystem.hpp"
#include "subsystem/Drive/Drive.hpp"
#include "PIDController.hpp"
#include "qtr.hpp"
#include "mux.h"
#include "constants.h"



Drive LARC;
ServoSystem servos;
Mux74HC4067 mux;
QTR qtrFront(0, mux);

PIDController linePID(0.00001f, 0.0f, 0.00000008f, -1.0f, 1.0f);

static constexpr float kBaseSpeed = 0.32f;

static byte visionLeft  = 0;
static byte visionRight = 0;

void readVision()
{
    if (Serial.available() >= 3) {
        if (Serial.read() == 0xFF) {
            visionLeft  = Serial.read();
            visionRight = Serial.read();
        }
    }
}

void setup()
{
    Serial.begin(115200);

    LARC.begin();
    LARC.holdYaw(true);
    LARC.setTargetYaw(LARC.getYaw());

    qtrFront.begin();
    qtrFront.useDefaultCalibration();

    servos.begin();
    servos.intakeUpperDeploy();
    servos.intakeLowerDeploy();
    delay(600);
    Serial.write(0xFE);  //start vision
    // Serial.write(0xFD);  // stop vision
    Serial.println(F("Vision+Servo+LinePID test ready."));
}

void loop()
{
    LARC.update();
    qtrFront.update();
    readVision();

    // Line PID
    int   linePos = qtrFront.getBinaryPosition();
    bool  onLine  = qtrFront.onLine();
    float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
    float vx       = -lineCorr;

    // Servos — direct vision response, no isBusy
    if (visionLeft)  servos.intakeUpperDeploy();
    else             servos.intakeUpperHome();

    if (visionRight) servos.intakeLowerDeploy();
    else             servos.intakeLowerHome();

    // Drive — strafe left along line
    if (!onLine)
        LARC.stop();
    else
        LARC.setTranslation(vx, -kBaseSpeed);  // mirrors BEANS exactly

    // Debug
    static byte prevL = 0xFF, prevR = 0xFF;
    if (visionLeft != prevL || visionRight != prevR) {
        Serial.print(F("L=")); Serial.print(visionLeft);
        Serial.print(F(" R=")); Serial.println(visionRight);
        prevL = visionLeft;
        prevR = visionRight;
    }
}