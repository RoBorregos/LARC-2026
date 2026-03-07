/**
 * @file constants.h
 * @date 12/01/2026
 * @author Ximena Patricia García Magdaleno
 *
 * @brief Constants for the robot.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <math.h>

namespace Constants
{
    namespace SystemConstants
    {
        constexpr float kUpdateInterval = 20.0;
    } // namespace SystemConstants

    namespace Kinematics
    {
        // "omni_motors" class
        constexpr float M1_ANGLE = 135.0f;  // M1     UL
        constexpr float M2_ANGLE = 45.0f;   // M2     UR
        constexpr float M3_ANGLE = -135.0f; // M3     LL
        constexpr float M4_ANGLE = -45.0f;  // M4     LR
    } // namespace Kinematics

    namespace DriveConstants
    {
        static constexpr float kDEG2RAD = PI / 180.0f;

        constexpr float kWheelDiameter = 0.109f;
        constexpr float kWheelRaius = kWheelDiameter / 2.0;
        constexpr float kWheelCircumference = 2 * M_PI * kWheelRaius;
    } // namespace DriveConstants
    namespace PID
    {
        static constexpr float kKp = 1.1; // 4.8f;  2.5f; 1.5f
        static constexpr float kKi = 0.0f; //0.002f
        static constexpr float kKd = 0.01f;  //0.06f; 0.0f
        static constexpr float kOmegaMax = 0.25f;
        static constexpr float kcurrentVelocity = 0.42f; //Velocity according to PID

    } // namespace PID
     namespace UltrasonicConstants
    {
        static constexpr uint32_t kPingPeriodMs  = 50;    // ms entre pings
        static constexpr uint32_t kTrigHighUs    = 10;    // µs que el trigger está en HIGH
        static constexpr uint32_t kEchoTimeoutUs = 25000; // µs (~4 m máx)
    } // namespace UltrasonicConstants

    namespace QTRCalibration
    {
        constexpr size_t kNumSensors = 8;

        struct Profile
        {
            uint16_t min[kNumSensors];
            uint16_t max[kNumSensors];
        };

        // PLACEHOLDERS

 
        constexpr Profile Front = {
            {980, 960, 960, 960, 960, 960, 960, 960},
            {1015, 1015, 1015, 1015, 1015, 1015, 1015, 1015}
        };

        constexpr Profile Rear = {
            {120,  130,  115,  140,  150,  135,  128,  122 },
            {3100, 3200, 3050, 3300, 3350, 3250, 3150, 3000}
        };

        constexpr uint16_t kBinaryThreshold = 600; // 0-1000 normalized, tune this one value

    } // namespace QTRCalibration

    namespace LineFollower 
    
    {
    constexpr int kSetpoint = 3500;   // center of 0-7000 range
    // change to 4000, 2000, etc. as needed
    }


    namespace IRCalibration
    {
        // Umbrales analógicos por sensor (0..1023).
        // Si raw >= umbral = línea detectada (antes de aplicar inversión).

        // PLACEHOLDERS
        static constexpr uint16_t kThreshFL = 312; // Front-Left
        static constexpr uint16_t kThreshFR = 200; // Front-Right
        static constexpr uint16_t kThreshBL = 312; // Back-Left
        static constexpr uint16_t kThreshBR = 312; // Back-Right
    } // namespace IRCalibration

    namespace ServoAngles
    {
        // Intake Superior

        static constexpr uint8_t kIntakeUpperHome   = 0;   // retractado, 
        static constexpr uint8_t kIntakeUpperDeploy = 80;  // posición de tumble

        // Intake Inferior

        static constexpr uint8_t kIntakeLowerHome   = 0;   // retractado
        static constexpr uint8_t kIntakeLowerDeploy = 80;  // posición de tumble

        // Compuerta Separadora
        // Compuerta que dirige la semilla al canal correcto
        static constexpr uint8_t kSeparatorCenter = 90;  // neutro/hold
        static constexpr uint8_t kSeparatorLeft   = 45;  // canal izquierdo
        static constexpr uint8_t kSeparatorRight  = 135; // canal derecho

        // Servo Beneficiadero Rojo
        // Permanece cerrado hasta recibir señal
        static constexpr uint8_t kBenefitRedClosed = 0;   // cerrado
        static constexpr uint8_t kBenefitRedOpen   = 90;  // abierto

        // Servo Beneficiadero Azul
        // Permanece cerrado hasta recibir señal
        static constexpr uint8_t kBenefitBlueClosed = 0;   // cerrado
        static constexpr uint8_t kBenefitBlueOpen   = 90;  // abierto

        //Servo Holder
        // Permanece cerrado hasta recibir señal
        static constexpr uint8_t kHolderHold = 0;   // cerrado
        static constexpr uint8_t kHolderRelease   = 20;  // abierto

        // Tiempo de movimiento (ms)
        // Ajustar si queremos que se tarde o que la respuesta sea inmediata a la señal que recibe.
        static constexpr uint32_t kIntakeUpperMoveMs  = 300;
        static constexpr uint32_t kIntakeLowerMoveMs  = 300;
        static constexpr uint32_t kSeparatorMoveMs    = 300;
        static constexpr uint32_t kBenefitRedMoveMs   = 300;
        static constexpr uint32_t kBenefitBlueMoveMs  = 300;
        static constexpr uint32_t kHolderMoveMs       = 300;
    } // namespace ServoAngles

} // namespace Constants

#endif