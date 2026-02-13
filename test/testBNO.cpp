
#include <Arduino.h>
#include <Wire.h>
#include "constants.h"
#include "BNO/bno.hpp"
#include "pins.h" 
#include "motors.hpp"
#include "omni_motors.hpp"

BNO bno1;

static uint32_t lastPrint = 0;
static constexpr uint32_t kPrintMs = 100; // 10 Hz

//         ======== VARIABLES ========       
static constexpr float kWheelDiameter = Constants::DriveConstants::kWheelDiameter; // 0.109f

//Encoders:
static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];   
static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0]; 

static constexpr uint8_t UR_ENC_A = Pins::kEncoders[2];
static constexpr uint8_t UR_ENC_B = Pins::kEncoders[3];

static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];

static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];

// Motor object
DCMotor m1_ul(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], false,  UL_ENC_A, UL_ENC_B, kWheelDiameter); //M1
DCMotor m2_ur(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true, UR_ENC_A, UR_ENC_B, kWheelDiameter);   //M2
DCMotor m3_lr(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true,  LL_ENC_A, LL_ENC_B, kWheelDiameter);  //M3
DCMotor m4_ll(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], false, LR_ENC_A, LR_ENC_B, kWheelDiameter);  //M4

OmniMotors omni(m1_ul, m2_ur, m3_lr, m4_ll);



// Variables provicionales
static constexpr int kPwm = 225;                       
static constexpr unsigned long kRun = 1200;             
static constexpr unsigned long kPause = 600;      


static constexpr unsigned long kT = 2000;   // tiempo por prueba
static constexpr float kV = 0.45f;          // magnitud 
static constexpr float kW = 0.30f;          // giro 



void allFoward(DCMotor& m1, DCMotor& m2, DCMotor& m3, DCMotor& m4, float kPwm)
{

  Serial.println(" ALL motors going FORWARD:    on    ");


  m1.move(kPwm, DCMotor::Direction::FORWARD); 
  m2.move(kPwm, DCMotor::Direction::FORWARD);
  m3.move(kPwm, DCMotor::Direction::FORWARD); 
  m4.move(kPwm, DCMotor::Direction::FORWARD); 
}



void setup() {
  Serial.begin(115200);


  //                      ====== BNO =======
  while (!Serial) { delay(10); }

  Wire.begin();

  Serial.println("\n=== BNO test start ===");
  if (!bno1.begin()) {
    Serial.println("BNO begin failed (unexpected).");
  }

  Serial.println("Move/rotate the robot.\n");



  //                  ========== KINEMATICS =========
  m1_ul.begin(); m2_ur.begin(); m3_lr.begin(); m4_ll.begin();
  Serial.println("Kinematics going forward");
}

void loop() {


  bno1.update(); //update inputs

  const uint32_t now = millis();
  if (now - lastPrint >= kPrintMs) {
    lastPrint = now;

    float yaw   = bno1.getYaw();
    float roll  = bno1.getRoll();
    float pitch = bno1.getPitch();

    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print(" deg\tRoll: ");
    Serial.print(roll, 2);
    Serial.print(" deg\tPitch: ");
    Serial.print(pitch, 2);
    Serial.println(" deg");
    
    
    //omni.MoveXYW(+kV, 0.0f, 0.0f); // test motors with BNO (optional).

  }
  delay(2); 

}
