#include <Arduino.h>
#include "pins.h"

#include "ultrasonic/ultrasonic.hpp"
#include "subsystem/Drive/Drive.hpp"

//Funciono :)      -> Faltan mejoras
Drive drive;

// 2 ultrasonicos
Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);

enum class State { FORWARD, RIGHT };
State state = State::FORWARD;
uint32_t stateStart = 0;

void setup()
{
  drive.begin();
  drive.holdYaw(true);

  us1.begin();
  us2.begin();

  state = State::FORWARD;
  stateStart = millis();
}

void loop()
{
 
  drive.update(); // control update

  // Lectura simultanea: no bloquea 
  us1.update();
  us2.update();

  // Distancias (cm)
  float d1 = us1.getdistance();
  float d2 = us2.getdistance();

  //Identifica obstaculo cuando ambos son menores a 10 cm
  bool obstacle =
    (us1.isValid() && d1 < 10.0f) ||
    (us2.isValid() && d2 < 10.0f);

  const uint32_t now = millis();

  // Logica
  if (state == State::FORWARD)
  {
    drive.forward(0.35f);

    if (obstacle)
    {
      state = State::RIGHT;
      stateStart = now;
    }
  }
  else // RIGHT
  {
    drive.right(0.35f);

    // esquiva por 400 ms y regresa
    if (now - stateStart > 400)
    {
      state = State::FORWARD;
      stateStart = now;
    }
  }
}
