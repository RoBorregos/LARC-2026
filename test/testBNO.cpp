#include <Arduino.h>
#include "subsystem/Drive/Drive.hpp"
#include "BNO/bno.hpp"


BNO BNO1;
Drive motors;

void setup() {
  Serial.begin(115200);
  delay(500);
  //motors.begin();
  //motors.holdYaw(false); //"true" to initialize PID

  BNO1.begin();           // inicializa el BNO
  BNO1.runCalibration();  // corre tu test/calibración
}

void loop() {
  //motors.forward(0.35f);  
  //motors.update(); // read BNOs values by using Drive class
    BNO1.update();

    BNO1.getAngular();

    delay(500);
}
