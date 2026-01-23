/*
 * @file wheels.cpp
 *
 * @author Ximena Patricia García Magdaleno
 * 
 * @version 0.1
 * 
 * @date 2026-01-12
 */


//#include  "constants.h"
#include "omni_motors.hpp"
#include <math.h>


static constexpr float DEG2RAD = PI / 180.0f;

static constexpr float UL_ANGLE = 135.0f;
static constexpr float UR_ANGLE = 45.0f;
static constexpr float LR_ANGLE = -45.0f;
static constexpr float LL_ANGLE = -135.0f;


void OmniMotors::normalize4(float &a, float &b, float &c, float &d) 
{
// NORMALIZAR llantas en conjunto
  float m = fabsf(a);
  m = fmaxf(m, fabsf(b));
  m = fmaxf(m, fabsf(c));
  m = fmaxf(m, fabsf(d));

  if (m > 1.0f) { a/=m; b/=m; c/=m; d/=m; }
}



void OmniMotors::setWheelCmd(DCMotor& m, float cmd) {
  // "cdm" que tan fuerte y en que sentido gira la rueda <-1 || 1>
  cmd = constrain(cmd, -1.0f, 1.0f);

  auto dir = (cmd >= 0.0f) ? DCMotor::Direction::FORWARD
                           : DCMotor::Direction::BACKWARD;

  int pwm = (int)(fabsf(cmd) * 255.0f);
  m.move(pwm, dir); //objeti m, atributos "pwm" y "dir">>> direccion
  
}


void OmniMotors::MoveXYW(float vx, float vy, float omega)
{
  //calculo de velocidades
  float m1 = vx * cosf(UL_ANGLE * DEG2RAD)
           + vy * sinf(UL_ANGLE * DEG2RAD)
           + omega;

           
  float m2 = vx * cosf(UR_ANGLE * DEG2RAD)
           + vy * sinf(UR_ANGLE * DEG2RAD)
           + omega;


  float m3 = vx * cosf(LR_ANGLE * DEG2RAD)
           + vy * sinf(LR_ANGLE * DEG2RAD)
           + omega;


  float m4 = vx * cosf(LL_ANGLE * DEG2RAD)
           + vy * sinf(LL_ANGLE * DEG2RAD)
           + omega;


  normalize4(m1, m2, m3, m4);

  setWheelCmd(upper_left_,  m1);
  setWheelCmd(upper_right_, m2);
  setWheelCmd(lower_right_, m3);
  setWheelCmd(lower_left_,  m4);
}