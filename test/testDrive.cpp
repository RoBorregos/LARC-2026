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

void setup() {
  drive.begin();
}

void loop() {
  drive.update();
}
