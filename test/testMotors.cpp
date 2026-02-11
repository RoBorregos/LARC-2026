// Test kinematics

#include <Arduino.h>  
#include "pins.h" 
#include "constants.h"
#include "motors.hpp"
#include "omni_motors.hpp"


static constexpr float kWheelDiameter = Constants::DriveConstants::kWheelDiameter; // 0.109f

//Encoders
static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];   
static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0]; 

static constexpr uint8_t UR_ENC_A = Pins::kEncoders[2];
static constexpr uint8_t UR_ENC_B = Pins::kEncoders[3];

static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];

static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];


//Motors
DCMotor m1_ul(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], false,  UL_ENC_A, UL_ENC_B, kWheelDiameter); //M1
DCMotor m2_ur(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true, UR_ENC_A, UR_ENC_B, kWheelDiameter);   //M2
DCMotor m3_lr(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true,  LL_ENC_A, LL_ENC_B, kWheelDiameter);  //M3
DCMotor m4_ll(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], false, LR_ENC_A, LR_ENC_B, kWheelDiameter);  //M4

OmniMotors omni(m1_ul, m2_ur, m3_lr, m4_ll);



static constexpr int kPwm = 225;                        // PWM 180 estaba antes lento pero bien
static constexpr unsigned long kRun = 1200;             
static constexpr unsigned long kPause = 600;           


static constexpr unsigned long kT = 2000; // tiempo por prueba
static constexpr float kV = 0.45f;         // magnitud (0..1)
static constexpr float kW = 0.30f;         // giro (0..1)




void setup()
{

  Serial.begin(115200);
  delay(400);
  analogWriteResolution(8);

  m1_ul.begin(); m2_ur.begin(); m3_lr.begin(); m4_ll.begin();
  Serial.println("=== Motor test: each wheel FORWARD then BACKWARD ===");

}




void loop() 
{      
  Serial.println("Forward (+vx)");          
  omni.MoveXYW(+kV, 0.0f, 0.0f); // el giro esta en cero -> omega.
  delay(kT);


  // 2) Atras
  Serial.println("Backward (-vx)");
  omni.MoveXYW(-kV, 0.0f, 0.0f);
  delay(kT);

  // 3) Derecha 
  Serial.println("Right (+vy)");
  omni.MoveXYW(0.0f, +kV, 0.0f);
  delay(kT);

  // 4) Izquierda (strafe)     >>>> ADELANTE
  Serial.println("Left (-vy)");
  omni.MoveXYW(0.0f, -kV, 0.0f);
  delay(kT);


  /*
  // 5) Giro CW/CCW  (fuerza del motor no es suficiente para poder girar)
  Serial.println("Rotate (+omega)");
  omni.MoveXYW(0.0f, 0.0f, +kW);
  delay(kT);

  Serial.println("Rotate (-omega)");
  omni.MoveXYW(0.0f, 0.0f, -kW);
  delay(kT);
  */

  // En DERECHA diagonal ADELANTE
  Serial.println("45 grados"); // nuevo
  omni.MoveXYW(+kV, +kV, 0.0f);
  delay(kT);

  // En DERECHA diagonal ATRAS
  Serial.println("45 grados"); // nuevo
  omni.MoveXYW(-kV, -kV, 0.0f);
  delay(kT);

  // En IZQUIERDA diagonal ADELANTE
  Serial.println("45 grados"); // nuevo
  omni.MoveXYW(+kV, -kV, 0.0f);
  delay(kT);

  // En IZQUIERDA diagonal ATRAS
  Serial.println("45 grados"); // nuevo
  omni.MoveXYW(-kV, -kV, 0.0f);
  delay(kT);

  // Stop
  Serial.println("Stop");
  omni.Stop();
  delay(1200);

  Serial.println("=== repeat ===");

}