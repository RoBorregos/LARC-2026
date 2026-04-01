#include <Arduino.h>
#include "pins.h"
#include "qtr.hpp"
#include "mux.h"
#include "robot/instances/instances.h"
//This code is to be used to calibrate each qtr sensor for 10 seconds
// The idea is to paste the values on constants.h and avoid calibrating every run

 //Where the array starts and the mux being used

void setup() {
    Serial.begin(115200);
    delay(500);
    mux.begin();
    qtrFront.begin();

    Serial.println("Calibrating FRONT move robot over line for 10s...");
    qtrFront.calibrate(10000);
    qtrFront.printCalibration("FRONT");

    Serial.println("Done — copy values to constants.h");
}

void loop() {}