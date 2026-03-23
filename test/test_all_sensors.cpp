#include <Arduino.h>
#include <Wire.h>
#include "TCA9548A.h"
#include "tof.hpp"
#include "IR.hpp"
#include "qtr.hpp"
#include "mux.h"
#include "ServoSystem.hpp"
#include "bno.hpp"
#include "constants.h"
#include "motors.hpp"
#include <math.h>
#include "PIDController.hpp"
#include "subsystem/Drive/Drive.hpp"

Drive LARC;

static constexpr float velocity = 0.40f; //velocidad a 0.30f

uint32_t stateStartMs = 0;
uint32_t clearStartMs = 0;

IRLine ir(15, 14, 23, 41, 0b0000);

TCA9548A mux(0x70, Wire);
ToF tof1(5, mux, ToFType::L0X);
ToF tof2(0, mux, ToFType::L1X);
ToF tof3(1, mux, ToFType::L0X);
ToF tof4(3, mux, ToFType::L0X);

BNO imuBNO;

Mux74HC4067 amux;
ServoSystem servos;
QTR qtr1(0, amux);
QTR qtr2(8, amux);

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Wire.begin();
    amux.begin(); 
    ir.begin();
    qtr1.begin();
    qtr2.begin();
    servos.begin();

      LARC.begin();
    LARC.holdYaw(true);
    LARC.setTargetYaw(LARC.getYaw());



    if (imuBNO.begin())
        Serial.println("[OK] BNO055 ready");
    else {
        Serial.println("[FAIL] BNO055 not found!");
        while (1);
    }
    pinMode(30, OUTPUT);
    digitalWrite(30, HIGH);
    delay(100);
    Serial.print("Pin 30 reads back: ");
    pinMode(30, INPUT);
    Serial.println(digitalRead(30));

    if (!mux.begin()) {
        Serial.println("[FAIL] TCA9548A not found!");
        while (true);
    }
    Serial.println("[OK] TCA9548A found at 0x70\n");
    mux.scanAll(); 
    Serial.print("ToF1 init: "); Serial.println(tof1.begin() ? "OK" : "FAIL");
Serial.print("ToF2 init: "); Serial.println(tof2.begin() ? "OK" : "FAIL");
Serial.print("ToF3 init: "); Serial.println(tof3.begin() ? "OK" : "FAIL");
Serial.print("ToF4 init: "); Serial.println(tof4.begin() ? "OK" : "FAIL");
}

void loop()
{

    tof1.update();
    tof2.update();
    tof3.update();
    tof4.update();
    ir.update();
    qtr1.update();
    qtr2.update();
    imuBNO.update();
    LARC.update();

    static enum { FWD, BWD, DONE } phase = FWD;
    static uint32_t phaseStart = millis();
    uint32_t now = millis();

    switch (phase) {
        case FWD:
            LARC.forward(velocity);
            if (now - phaseStart >= 5000) {
                phaseStart = now;
                phase = BWD;
            }
            break;

        case BWD:
            LARC.backward(velocity);
            if (now - phaseStart >= 5000) {
                LARC.backward(velocity);
                phase = DONE;
            }
            break;

        case DONE:
            // Robot is stopped, do nothing
            break;
    }


    Serial.print("Roll: ");  Serial.print(imuBNO.getRoll());
    Serial.print("  Pitch: "); Serial.print(imuBNO.getPitch());
    Serial.print("  Yaw: ");  Serial.println(imuBNO.getYaw());



    Serial.print("qtr1: ");
    qtr1.debugPrint();
    Serial.println();
    Serial.print("qtr2: ");
    qtr2.debugPrint();
    amux.debugPrint();


    // Intake Superior
    servos.intakeUpperHome();
    Serial.println("Intake upper home");
    servos.intakeLowerHome();
    Serial.println("Intake lower home");
    servos.benefitBlueOpen();
    Serial.println("Benefit blue open");
    servos.separatorLeft();
    Serial.println("Separator left");
        delay(300);
    servos.intakeUpperDeploy();
    Serial.println("Intake upper deploy");
    servos.intakeLowerDeploy();
    Serial.println("Intake lower deploy");
    servos. benefitRedClose();
    Serial.println("Benefit red close");
    servos.separatorRight();
    Serial.println("Separator right");
        delay(300);*/

    // Separator
    servos.separatorCenter();
        delay(500);
    servos.separatorLeft();
        delay(500);
    servos.separatorRight();
        delay(500);

    // Blue benefit
    servos.benefitBlueOpen();
        delay(500);
    servos.benefitBlueClose();
        delay(500);

    // Holder
    servos.holderHold();
        delay(500);
    servos.holderRelease();
        delay(500);

    amux.debugPrint();

    Serial.print(" | ToF1: ");
    Serial.print(tof1.getDistanceMm());
    Serial.println(" mm");

    Serial.print(" | ToF2: ");
    Serial.print(tof2.getDistanceMm());
    Serial.println(" mm");

    Serial.print(" | ToF3: ");
    Serial.print(tof3.getDistanceMm());
    Serial.println(" mm");
    
    
    Serial.print(" | ToF4: ");
    Serial.print(tof4.getDistanceMm());
    Serial.println(" mm");

    Serial.println();
    mux.scanAll();
    delay(1000);

    Serial.print("FL:"); Serial.print(ir.getState(IRLine::FL));
    Serial.print(" FR:"); Serial.print(ir.getState(IRLine::FR));
    Serial.print(" BL:"); Serial.print(ir.getState(IRLine::BL));
    Serial.print(" BR:"); Serial.println(ir.getState(IRLine::BR));

}