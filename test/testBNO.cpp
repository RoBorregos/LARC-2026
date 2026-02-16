#include <Arduino.h>
#include "subsystem/Drive/Drive.hpp"

Drive motors;

void setup() {
  Serial.begin(115200);
  delay(500);

  motors.begin();
  motors.holdYaw(true); //false to stop PID
}

void loop() {
  motors.forward(0.35f);  
  motors.update(); // this function includes a BNO lecture
}
