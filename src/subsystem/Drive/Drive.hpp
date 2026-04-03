#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "constants.h"
#include "pins.h"
#include "BNO/bno.hpp"
#include "motors.hpp"
#include "kinematics.hpp"
#include "PIDController.hpp"
#include "qtr.hpp"

// Constants
static constexpr float diameter = Constants::DriveConstants::kWheelDiameter;

static constexpr uint8_t UL_ENC_A = Pins::kEncoders[1];
static constexpr uint8_t UL_ENC_B = Pins::kEncoders[0];
static constexpr uint8_t UR_ENC_A = Pins::kEncoders[2];
static constexpr uint8_t UR_ENC_B = Pins::kEncoders[3];
static constexpr uint8_t LL_ENC_A = Pins::kEncoders[4];
static constexpr uint8_t LL_ENC_B = Pins::kEncoders[5];
static constexpr uint8_t LR_ENC_A = Pins::kEncoders[6];
static constexpr uint8_t LR_ENC_B = Pins::kEncoders[7];

class Drive {
public:
    Drive();

    void begin();
    void update();  // llamar siempre en loop()

    // ── Movimiento básico ──────────────────────────────
    void forward(float speed);
    void backward(float speed);
    void left(float speed);
    void right(float speed);
    void diagonalLeft(float speed);
    void stop();
    void brake();
    void setTranslation(float vx, float vy);

    // ── Movimiento autónomo con distancia ─────────────
    // Avanza/retrocede dist metros a speed [0..1], bloqueante NO (usa update())
    void driveDistance(float meters, float speed);
    // Strafe lateral dist metros a speed [0..1]
    void strafeDistance(float meters, float speed);
    // Gira a ángulo absoluto en grados (0=norte, +CCW), timeout en ms
    void turnTo(float targetDeg, uint32_t timeoutMs = 3000);
    // True mientras ejecuta driveDistance / strafeDistance / turnTo
    bool isBusy() const { return busy_; }

    // ── Yaw hold ──────────────────────────────────────
    void holdYaw(bool enable);
    void setTargetYaw(float yawRad);
    float getYaw() const;

    // ── Omega manual (sin PID) ────────────────────────
    void setManualOmega(float omegaRadS);
    void clearManualOmega();

    // ── QTR – seguimiento de línea ────────────────────
    // Llama una vez en setup() después de begin()
    void attachQTR(QTR& qtr);
    // Empieza a recorrer el frente de los árboles lateralmente
    void followLine(float vySpeed);
    void stopLineFollow();
    bool isFollowingLine() const { return lineFollowEnabled_; }

    // ── Odometría ─────────────────────────────────────
    void  resetOdometry();
    float getOdoX() const { return odoX_; }
    float getOdoY() const { return odoY_; }

    // ── Test ──────────────────────────────────────────
    void testKinematics(float v = 0.45f, uint32_t T = 2000);

    float getVxCmd() const { return vxCmd_; }
    float getVyCmd() const { return vyCmd_; }

    //Encoders debug
    float getM1Meters() const { return const_cast<Drive*>(this)->m1_ul_.getDistanceMeters(); }
    float getM2Meters() const { return const_cast<Drive*>(this)->m2_ur_.getDistanceMeters(); }
    float getM3Meters() const { return const_cast<Drive*>(this)->m3_ll_.getDistanceMeters(); }
    float getM4Meters() const { return const_cast<Drive*>(this)->m4_lr_.getDistanceMeters(); }

    void beginNoBNO();
    void updateNoBNO();

private:
    // ── Helpers ───────────────────────────────────────
    static float rad2deg(float r);
    static float deg2rad(float d);
    static float clampf(float x, float lo, float hi);
    static float wrapAngle(float a);   // normaliza a [-π, π]

    void updateOdometry();
    void updateBusy(uint32_t now);

    // ── Hardware ──────────────────────────────────────
    BNO      bno_;
    DCMotor  m1_ul_, m2_ur_, m3_ll_, m4_lr_;
    OmniMotors omni_;

    // ── Yaw PID ───────────────────────────────────────
    static constexpr float P         = Constants::PID::kKp;
    static constexpr float I         = Constants::PID::kKi;
    static constexpr float D         = Constants::PID::kKd;
    static constexpr float kOmegaMax = Constants::PID::kOmegaMax;

    PIDController yawPid_;
    float targetYaw_         = 0.0f;
    bool  yawHoldEnabled_    = true;
    bool  manualOmegaEnabled_= false;
    float manualOmega_       = 0.0f;

    // ── Comandos de velocidad ─────────────────────────
    float vxCmd_ = 0.0f;
    float vyCmd_ = 0.0f;

    // ── Movimiento autónomo (driveDistance / turnTo) ──
    bool     busy_          = false;
    float    distTarget_    = 0.0f;   // metros; -1 = modo giro
    float    busyVx_        = 0.0f;
    float    busyVy_        = 0.0f;
    uint32_t busyStart_     = 0;
    uint32_t busyTimeout_   = 5000;

    // ── Odometría ─────────────────────────────────────
    float odoX_ = 0.0f;
    float odoY_ = 0.0f;

    // ── QTR / line follow ─────────────────────────────
    QTR*          qtrFront_          = nullptr;
    PIDController linePid_;
    bool          lineFollowEnabled_ = false;
    float         lineSpeedVy_       = 0.0f;

    static constexpr float kLineCenter   = 3500.0f;
    static constexpr float kLinePidMax   = 0.20f;
    static constexpr float kLineDeadband = 150.0f;

    // ── Timing ────────────────────────────────────────
    static constexpr uint32_t kControlMs = 10;
    static constexpr uint32_t kPrintMs   = 100;
    uint32_t lastControl_ = 0;
    uint32_t lastPrint_   = 0;

    // Update and reset Odometry
    float prevD1_ = 0.0f;
    float prevD2_ = 0.0f;
    float prevD3_ = 0.0f;
    float prevD4_ = 0.0f; 
};
