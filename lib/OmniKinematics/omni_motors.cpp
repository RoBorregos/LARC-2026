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

static constexpr float M1_ANGLE = 135.0f;  //M1
static constexpr float M2_ANGLE = 45.0f;   //M2
static constexpr float M3_ANGLE = -45.0f;  //M3
static constexpr float M4_ANGLE = -135.0f; //M4


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
  m.move(pwm, dir); //objeto m, atributos "pwm" y "dir">>> direccion

}


void OmniMotors::MoveXYW(float vx, float vy, float omega)
{
  //calculo de velocidades + control con PID
  float m1 = vx * cosf(M1_ANGLE * DEG2RAD)
           + vy * sinf(M1_ANGLE * DEG2RAD) // tal vez cambie
            + omega;

           
  float m2 = vx * cosf(M2_ANGLE * DEG2RAD)
           + vy * sinf(M2_ANGLE * DEG2RAD)
           + omega;


  float m3 = vx * cosf(M3_ANGLE * DEG2RAD)
           + vy * sinf(M3_ANGLE * DEG2RAD)
           + omega;


  float m4 = vx * cosf(M4_ANGLE * DEG2RAD)
           + vy * sinf(M4_ANGLE * DEG2RAD)
           + omega;


  normalize4(m1, m2, m3, m4);

  setWheelCmd(upper_left_,  m1);
  setWheelCmd(upper_right_, m2);
  setWheelCmd(lower_right_, m3);
  setWheelCmd(lower_left_,  m4);
}


//=====POR SI NO FUNICONA LA FUNION ORIGINAL=========
/* 
// Funcion para cuando no se tiene PID
void Motors::MoveMotors(int degree, uint8_t speed)
{
    float m1 = cos(((45 + degree) * PI / 180));
    float m2 = cos(((135 + degree) * PI / 180));
    float m3 = cos(((225 + degree) * PI / 180));
    float m4 = cos(((315 + degree) * PI / 180));
    int speedA = abs(int(m1 * speed));
    int speedB = abs(int(m2 * speed));
    int speedC = abs(int(m3 * speed));
    int speedD = abs(int(m4 * speed));

    analogWrite(motor1.GetSpeed(), speedA);
    analogWrite(motor2.GetSpeed(), speedB);
    analogWrite(motor3.GetSpeed(), speedC);
    analogWrite(motor4.GetSpeed(), speedD);

    if (m1 >= 0)
    {
        motor1.MoveForward();
    }
    else
    {
        motor1.MoveBackward();
    }
    if (m2 >= 0)
    {
        motor2.MoveForward();
    }
    else
    {
        motor2.MoveBackward();
    }
    if (m3 >= 0)
    {
        motor3.MoveForward();
    }
    else
    {
        motor3.MoveBackward();
    }
    if (m4 >= 0)
    {
        motor4.MoveForward();
    }
    else
    {
        motor4.MoveBackward();
    }
}
*/


/*
void MoveMotors(float degree, floatt speed, float omega){

    float m2 = cos(((45 + degree) * PI / 180)) * speed + speed_w;
    float m3 = cos(((135 + degree) * PI / 180)) * speed + speed_w;
    float m4 = cos(((225 + degree) * PI / 180)) * speed + speed_w;
    float m1 = cos(((315 + degree) * PI / 180)) * speed + speed_w;
    int speedA = abs(int(m1));
    int speedB = abs(int(m2));
    int speedC = abs(int(m3));
    int speedD = abs(int(m4));

    Serial.print("m1: "); Serial.print(m1); Serial.print(", speedA: "); Serial.println(speedA);
    Serial.print("m2: "); Serial.print(m2); Serial.print(", speedB: "); Serial.println(speedB);
    Serial.print("m3: "); Serial.print(m3); Serial.print(", speedC: "); Serial.println(speedC);
    Serial.print("m4: "); Serial.print(m4); Serial.print(", speedD: "); Serial.println(speedD);


    analogWrite(motor1.GetSpeed(), speedA);
    analogWrite(motor2.GetSpeed(), speedB);
    analogWrite(motor3.GetSpeed(), speedC);
    analogWrite(motor4.GetSpeed(), speedD);
}
*/

