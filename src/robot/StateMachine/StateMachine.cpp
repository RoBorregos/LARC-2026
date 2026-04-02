
#include <Arduino.h>
#include "StateMachine.h"
#include "robot/instances/instances.h"
#include "constants.h"
#include "Vision.hpp"

namespace
{
    static constexpr float kVelocity = 0.40f;
    static constexpr float kObstacleDistanceCm = 25.0f;
    static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

    static constexpr uint32_t kInitializedStoppedMs = 9000;
    static constexpr uint32_t kStartIgnoreTimeMs = 2200;    // Time to ignore IR's at the START point
    static constexpr uint32_t kClearDelayMs = 800;          // Tiempo para cambiar nuevamente a Forward
    static constexpr uint32_t kNoObstacleToCornerMs = 3800; // Time without obstacle to go forward and LOOKFORLINE -> tal vez disminuir
    static constexpr uint32_t kCornerDeployWaitMs = 1800;

    static constexpr uint32_t kMinAvoidTimeMs = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;
    static constexpr uint16_t kTofTargetMm = 120;   // distancia deseada al árbol
    static constexpr uint16_t kTofHardStopMm = 105; // stop de seguridad
    static constexpr float kTofMinSpeed = 0.10f;
    static constexpr float kTofMaxSpeed = 0.35f;

    static constexpr float kDistKp = 0.0012f;
    static constexpr float kDistKi = 0.0f;
    static constexpr float kDistKd = 0.00015f;

    //static float distIntegral = 0.0f;
    //static float distPrevError = 0.0f;
    //static uint32_t distPrevMs = 0;

    /*
    static float clampf(float x, float lo, float hi)
    {
        if (x < lo)
            return lo;
        if (x > hi)
            return hi;
        return x;
    }

    static void resetDistancePID()
    {
        distIntegral = 0.0f;
        distPrevError = 0.0f;
        distPrevMs = 0;
    }

    
    static float computeDistanceSpeed(uint32_t now, uint16_t distanceMm, float maxSpeed)
    {
        if (distanceMm == ToF::INVALID_MM)
        {
            resetDistancePID();
            return maxSpeed;
        }

        if (distanceMm <= kTofHardStopMm)
        {
            resetDistancePID();
            return 0.0f;
        }

        float dt = (distPrevMs == 0) ? 0.01f : (now - distPrevMs) * 0.001f;
        distPrevMs = now;

        if (dt <= 0.0f)
            dt = 0.01f;

        // error positivo cuando todavía estás lejos
        float error = (float)distanceMm - (float)kTofTargetMm;

        distIntegral += error * dt;
        distIntegral = clampf(distIntegral, -150.0f, 150.0f);

        float derivative = (error - distPrevError) / dt;
        distPrevError = error;

        float u = kDistKp * error + kDistKi * distIntegral + kDistKd * derivative;

        u = clampf(u, 0.0f, maxSpeed);

        if (u > 0.0f && u < kTofMinSpeed)
            u = kTofMinSpeed;

        return u;
    }
    */

    const __FlashStringHelper *mainStateName(STATES state)
    {
        switch (state)
        {
        case STATES::START:
            return F(""); // F("START ♡ ♡ ♡");
        case STATES::POOL:
            return F(""); // F("POOL");
        case STATES::LOOKFORLINE:
            return F(""); // F("LOOKFORLINE");
        case STATES::LOOKFORCORNER:
            return F(""); // F("LOOKFORCORNER");
        case STATES::BEANS:
            return F(""); // F("BEANS");
        case STATES::BEANSGOBACK:
            return F(""); // F("BEANSGOBACK");
        case STATES::POOLSGOBACK:
            return F(""); // F("POOLSGOBACK");
        case STATES::LOOKFORLINEBACKWARDS:
            return F(""); // F("LOOKFORLINEBACKWARDS");
        case STATES::BENEFITSSTARTCORNER:
            return F(""); // F("BENEFITSSTARTCORNER");
        case STATES::BENEFITS:
            return F(""); // F("BENEFITS");
        case STATES::STOP:
            return F(""); // F("STOP ♡ ♡ ♡ ♡ ♡");
        default:
            return F(""); // F("DEFAULT");
        }
    }

    const __FlashStringHelper *poolStateName(PoolSubState state)
    {
        switch (state)
        {
        case PoolSubState::FORWARD:
            return F(""); // F("FORWARD");
        case PoolSubState::AVOID_LEFT:
            return F(""); // F("AVOID_LEFT");
        case PoolSubState::AVOID_RIGHT:
            return F(""); // F("AVOID_RIGHT");
        default:
            return F(""); // F("DEFAULT");
        }
    }
}

LARCStateMachine::LARCStateMachine()
{
}

void LARCStateMachine::begin()
{
    currentState = STATES::LOOKFORLINE; // siempre en START
    poolState = PoolSubState::FORWARD;

    state_start_time = millis();
    action_start_time = 0;
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    visionLeft = 0;
    visionRight = 0;

    pinMode(limitSwitch, INPUT_PULLUP);

    vision.begin();
    vision.requestStatus();

    //QTR
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);   // FRONT
    //qtrFront.printCalibration("QTR FRONT");

    // ToF
    tofLeft.begin();
    tofRight.begin();

    tofLeft.setMaxRange(500);
    tofRight.setMaxRange(500);

    tofLeft.setUpdateInterval(30);
    tofRight.setUpdateInterval(30);
}

void LARCStateMachine::update()
{
    LARC.update();
    ir.update();
    us1.update();
    us2.update();
    qtrFront.update();
    vision.update(); // Vision
    tofLeft.update();
    tofRight.update();

    const uint32_t now = millis();
    startStateTime();

    const int linePos = qtrFront.getPosition(); // Para el PID → más suave
    const bool onLine = qtrFront.onLine();
    const float lineError = linePos - Constants::LineFollower::kSetpoint;
    const float lineCorr  = (fabsf(lineError) > 150.0f)   // zona muerta ±150
                            ? linePID.update(linePos, Constants::LineFollower::kSetpoint)
                            : 0.0f;
    const float vx = -lineCorr;

    const bool FL = ir.getState(IR_mux::FL);
    const bool FR = ir.getState(IR_mux::FR);
    const bool BL = ir.getState(IR_mux::BL);
    const bool BR = ir.getState(IR_mux::BR);

    //ir.debugPrint();
    qtrFront.debugPrint();
    //Serial.print("linePos: ");
    //Serial.println(linePos); // To know the value for the center of the qtr

    const bool frontLeftDetectedLine = FL; // Also used for corner
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine = BL;
    const bool backRightDetectedLine = BR;
    const bool frontDetectedLine = (FL || FR); // Hacer que con el qtr tambien detecte linea
    const bool backDetected = (BL || BR);
    const bool leftDetectedPool = (FL || BL);
    const bool rightDetectedPool = (FR || BR);

    const float d1 = us1.getdistance();
    const float d2 = us2.getdistance();

    const bool obstacleLeftNow = us1.isValid() && d1 < kObstacleDistanceCm;
    const bool obstacleRightNow = us2.isValid() && d2 < kObstacleDistanceCm;

    static bool obstacleLatched = false;
    static uint32_t obstacleClearStartMs = 0;
    static constexpr uint32_t kObstacleReleaseMs = 200;

    const bool obstacle = obstacleLatched;

    if (!obstacleLatched)
    {
        if (obstacleLeftNow || obstacleRightNow)
        {
            obstacleLatched = true;
            obstacleClearStartMs = 0;
        }
    }
    else
    {
        if (obstacleLeftNow || obstacleRightNow)
        {
            obstacleClearStartMs = 0;
        }
        else
        {
            if (obstacleClearStartMs == 0)
                obstacleClearStartMs = now;

            if ((now - obstacleClearStartMs) >= kObstacleReleaseMs)
            {
                obstacleLatched = false;
                obstacleClearStartMs = 0;
            }
        }
    }

    /*
    Serial.print("US izquierda: ");
    Serial.print(d1);
    Serial.print(" valid1: ");
    Serial.print(us1.isValid());

    Serial.print(" | US derecha: ");
    Serial.print(d2);
    Serial.print(" valid2: ");
    Serial.print(us2.isValid());

    Serial.print(" | obstacle: ");
    Serial.println(obstacle);
    */
    

    switch (currentState)
    {
    case STATES::START:
        handleStartState(now, backDetected);
        break;

    case STATES::POOL:
        handlePoolState(now, obstacle, leftDetectedPool, rightDetectedPool);
        break;

    case STATES::LOOKFORLINE:
        handleLookForLineState(now, frontDetectedLine, frontLeftDetectedLine, frontRightDetectedLine, onLine);
        break;

    case STATES::LOOKFORCORNER:
        handleLookForCornerState(now, backLeftDetectedLine, vx);
        break;

    case STATES::BEANS:
        handleBEANS(now, backRightDetectedLine, onLine, vx);
        break;

    case STATES::BEANSGOBACK:
        handleBEANSGoBackState(now, frontLeftDetectedLine, onLine, vx);
        break;

    case STATES::POOLSGOBACK:
        handlePOOLSGoBackState(now, obstacle, leftDetectedPool, rightDetectedPool);
        break;

    case STATES::LOOKFORLINEBACKWARDS:
        handleLookForLineBackWards(now, backDetected, backLeftDetectedLine, backRightDetectedLine);
        break;

    case STATES::BENEFITSSTARTCORNER:
        handleBenefitsStartCorner(now, frontLeftDetectedLine, vx, onLine);
        break;

    case STATES::BENEFITS:
        handleBenefits(now, backLeftDetectedLine, vx, onLine);
        break;

    case STATES::STOP:
        handleStopState();
        break;

    default:
        handleStopState();
        break;
    }
}

void LARCStateMachine::setState(STATES newState)
{
    if (currentState == newState)
        return;

    currentState = newState;
    state_start_time = millis();
    action_start_time = 0;
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;

    // Reset corrección lateral LOOKFORLINE
    lfCorrecting        = false;
    lfCorrectionDir     = 0;
    lfCorrectionStartMs = 0;

    vision.resetGuards();
    linePID.reset();   // <- AÑADE ESTO

    Serial.println(mainStateName(currentState));
}

void LARCStateMachine::startStateTime()
{
    // Serial.print("start state time");
    if (state_start_time == 0)
    {
        state_start_time = millis();
    }
}

void LARCStateMachine::setPoolState(PoolSubState newState)
{
    // Serial.print("POOL State");
    if (poolState == newState)
        return;

    poolState = newState;
    clearStartMs = 0;
    sideDetectStartMs = 0;
    poolStateStartMs = millis();

    // Serial.print(F("Pool substate -> "));
    Serial.println(poolStateName(poolState));
}

void LARCStateMachine::readVision()
{
    // Serial.print("VISION State");

    if (Serial.available() >= 3)
    {
        if (Serial.read() == 0xFF)
        {
            visionLeft = Serial.read();
            visionRight = Serial.read();
        }
    }
}

void LARCStateMachine::handleStartState(uint32_t now, bool backDetected)
{
    vision.stop();
    // Serial.print("START State");

    const bool limitPressed = (digitalRead(limitSwitch) == HIGH); // invertido

    if (limitPressed != lastLimitPressed)
    {
        if (limitPressed)
            Serial.println("LIMIT SWITCH PRESIONADO");
        else
            Serial.println("LIMIT SWITCH LIBERADO");

        lastLimitPressed = limitPressed;
    }

    switch (action_stage)
    {
    case 0:
        // Goes down until the limit switch is pressed out
        if (!limitPressed)
        {
            elevator.ElevatorPosition(2); // bajar
            LARC.brake();
        }
        else
        {
            elevator.ElevatorPosition(0);
            LARC.brake();
            // Serial.println("ELEVADOR ABAJO -> STOP 2000 ms");
            action_start_time = now;
            action_stage = 1;
        }
        break;

    case 1:
        // Stop 2000 ms
        elevator.ElevatorPosition(0);
        LARC.brake();

        if ((now - action_start_time) >= 2000)
        {
            // Serial.println("ELEVADOR SUBIENDO 9000 ms");
            action_start_time = now;
            action_stage = 2;
        }
        break;

    case 2:
        // Goes up 9000 ms
        elevator.ElevatorPosition(1);
        LARC.brake();

        if ((now - action_start_time) >= 9000)
        {
            elevator.ElevatorPosition(0);
            LARC.brake();
            // Serial.println("ELEVADOR DETENIDO -> STOP 3000 ms");
            action_start_time = now;
            action_stage = 3;
        }
        break;

    case 3:
        // Stop 3000 ms
        elevator.ElevatorPosition(0);
        LARC.brake();

        if ((now - action_start_time) >= 3000)
        {
            // Serial.println("START IGNORANDO IR Y AVANZANDO");
            action_start_time = now;
            action_stage = 4;
        }
        break;

    case 4:
        // Goes forward ignoring IR's
        elevator.ElevatorPosition(0);
        //LARC.forward(kVelocity);

        //---------INTENTO DE MEJORAR ARRANQUE ------- (todo esto esta en vez de LARC.forward(velocidad))

        static float currentSpeed = 0.0f;
        static uint32_t lastRampMs = 0;

        static constexpr float kAccelPerSec = 0.70f;  // qué tan rápido acelera
        static constexpr float kBrakePerSec = 1.20f;  // qué tan rápido frena

        if (lastRampMs == 0)

        lastRampMs = now;

        float dt = (now - lastRampMs) * 0.001f;
        lastRampMs = now;


        float speed = 0.45f;
        float targetSpeed = speed;   // la velocidad que ya calculaste
        float rate = (targetSpeed > currentSpeed) ? kAccelPerSec : kBrakePerSec;
        float maxStep = rate * dt;

        if (currentSpeed < targetSpeed)
        {
        currentSpeed += maxStep;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        }
        else if (currentSpeed > targetSpeed)
        {
        currentSpeed -= maxStep;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }

        if (currentSpeed <= 0.0f)
        LARC.brake();
            else
        LARC.forward(currentSpeed);

        static constexpr uint32_t kStartPassToPools = 1000;
        //---------FIN DE INTENTO DE MEJORAR ARRANQUE

        if ((now - action_start_time) >= kStartIgnoreTimeMs )
        {
            setPoolState(PoolSubState::FORWARD);
            setState(STATES::POOL);
            // Serial.println("START -> POOL");
        } else if(backDetected){
            if((now - action_start_time) >= kStartPassToPools) {
           setPoolState(PoolSubState::FORWARD);
            setState(STATES::POOL);
        // Serial.println("START -> POOL"); 
            }
        }
        break;
    }
}

void LARCStateMachine::handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected)
{
    vision.stop();
    vision.clearErrors();

    // Serial.print("POOL State");

    switch (poolState)
    {

    case PoolSubState::FORWARD:
{
    static bool     lineCorrectionActive   = false;
    static uint32_t lineCorrectionStartMs  = 0;
    static int8_t   lineCorrectionDir      = 0;   // -1 = left, +1 = right

    static constexpr uint32_t kLineCorrectionMs    = 120;
    static constexpr float    kLineCorrectionSpeed = 0.40f;

    if (lineCorrectionActive)
    {
        if ((now - lineCorrectionStartMs) < kLineCorrectionMs)
        {
            if (lineCorrectionDir < 0)
                LARC.left(kLineCorrectionSpeed);
            else
                LARC.right(kLineCorrectionSpeed);

            break; // mientras corrige linea, no mete logica de obstaculo
        }

        lineCorrectionActive  = false;
        lineCorrectionStartMs = 0;
        lineCorrectionDir     = 0;
    }

    if (obstacle)
    {
        noObstacleStartMs = 0;
        setPoolState(PoolSubState::AVOID_LEFT);
    }
    else
    {
        if (noObstacleStartMs == 0)
        {
            noObstacleStartMs = now;
        }

        if (rightDetected)
        {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = -1; // detecta derecha -> corrige izquierda
            LARC.left(kLineCorrectionSpeed);
            break;
        }
        else if (leftDetected)
        {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = +1; // detecta izquierda -> corrige derecha
            LARC.right(kLineCorrectionSpeed);
            break;
        }

        LARC.setTranslation(0.40, 0.08);
        //  LARC.forward(kVelocity);

        /*
        //---------INTENTO DE MEJORAR ARRANQUE ------- (todo esto esta en vez de LARC.forward(velocidad))
        LARC.forward(kVelocity);

        static float currentSpeed = 0.0f;
        static uint32_t lastRampMs = 0;

        static constexpr float kAccelPerSec = 0.70f;  // qué tan rápido acelera
        static constexpr float kBrakePerSec = 1.20f;  // qué tan rápido frena

        if (lastRampMs == 0)

        lastRampMs = now;

        float dt = (now - lastRampMs) * 0.001f;
        lastRampMs = now;


        float speed = 0.40f;
        float targetSpeed = speed;   // la velocidad que ya calculaste
        float rate = (targetSpeed > currentSpeed) ? kAccelPerSec : kBrakePerSec;
        float maxStep = rate * dt;

        if (currentSpeed < targetSpeed)
        {
        currentSpeed += maxStep;
        if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        }
        else if (currentSpeed > targetSpeed)
        {
        currentSpeed -= maxStep;
        if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }

        if (currentSpeed <= 0.0f)
        LARC.brake();
            else
        LARC.forward(currentSpeed);

        //---------FIN DE INTENTO DE MEJORAR ARRANQUE
        */

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
        {
            setState(STATES::LOOKFORLINE);
        }
    }
    break;
}

    case PoolSubState::AVOID_LEFT:
    {
        LARC.left(0.45f);

        const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

        if (leftDetected && canChangeSide)
        {
            if (sideDetectStartMs == 0)
                sideDetectStartMs = now;

            if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
            {
                clearStartMs = 0;
                sideDetectStartMs = 0;
                setPoolState(PoolSubState::AVOID_RIGHT);
            }
        }
        else
        {
            sideDetectStartMs = 0;

            if (!obstacle)
            {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= kClearDelayMs)
                {
                    noObstacleStartMs = 0;
                    setPoolState(PoolSubState::FORWARD);
                }
            }
            else
            {
                clearStartMs = 0;
            }
        }

        break;
    }

    case PoolSubState::AVOID_RIGHT:
    {
        LARC.right(kVelocity);

        const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

        if (rightDetected && canChangeSide)
        {
            if (sideDetectStartMs == 0)
                sideDetectStartMs = now;

            if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
            {
                clearStartMs = 0;
                sideDetectStartMs = 0;
                setPoolState(PoolSubState::AVOID_LEFT);
            }
        }
        else
        {
            sideDetectStartMs = 0;

            if (!obstacle)
            {
                if (clearStartMs == 0)
                    clearStartMs = now;

                if ((now - clearStartMs) >= kClearDelayMs)
                {
                    noObstacleStartMs = 0;
                    setPoolState(PoolSubState::FORWARD);
                }
            }
            else
            {
                clearStartMs = 0;
            }
        }

        break;
    }
    }
}

// qtr
uint32_t lineDetectStartMs = 0;

void LARCStateMachine::handleLookForLineState(uint32_t now,
                                              bool frontDetected,
                                              bool leftDetected,
                                              bool rightDetected,
                                              bool onLine)
{
    vision.startBeans();

    servos.intakeUpperHome();
    servos.intakeLowerHome();

    static constexpr float    kLookFastSpeed   = 0.40f;
    static constexpr float    kLookSlowSpeed   = 0.34f;
    static constexpr float    kUsSlowStartCm   = 30.0f;
    static constexpr float    kUsHardStopCm    = 7.0f;
    static constexpr float    kUsMinSpeed      = 0.28f;
    static constexpr float    kCorrectionSpeed = 0.38f;
    static constexpr uint32_t kCorrectionMs    = 450;
    static constexpr uint32_t kLateralHoldMs   = 40; // ms continuos para confirmar línea lateral

    static uint32_t lastPrintMs = 0;
    static constexpr uint32_t kPrintEveryMs = 100;

    // ── Prioridad 1: línea frontal (destino) ─────────────────────────────
    if (frontDetected && onLine)
    {
        lfCorrecting  = false;
        lfLeftHoldMs  = 0;
        lfRightHoldMs = 0;
        Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
        LARC.brake();
        setState(STATES::LOOKFORCORNER);
        return;
    }

    // ── Prioridad 2: corrección lateral en curso ──────────────────────────
if (lfCorrecting)
{
    if ((now - lfCorrectionStartMs) < kCorrectionMs)
    {
        // Rampa: arranca en 40% de la velocidad y llega al 100% en 150ms
        static constexpr uint32_t kRampMs    = 150;
        static constexpr float    kMinScale  = 0.40f;

        float t     = float(now - lfCorrectionStartMs) / float(kRampMs);
        if (t > 1.0f) t = 1.0f;

        float scale = kMinScale + (1.0f - kMinScale) * t;
        float spd   = kCorrectionSpeed * scale;

        if (lfCorrectionDir < 0)
            LARC.left(spd);
        else
            LARC.right(spd);
        return;
    }
    // Corrección terminada
    lfCorrecting  = false;
    lfLeftHoldMs  = 0;
    lfRightHoldMs = 0;
}

    // ── Prioridad 3: acumular detección lateral con debounce ──────────────
    const bool onlyRight = rightDetected && !leftDetected;
    const bool onlyLeft  = leftDetected  && !rightDetected;

    if (onlyRight)
    {
        if (lfRightHoldMs == 0) lfRightHoldMs = now;
        lfLeftHoldMs = 0;
    }
    else if (onlyLeft)
    {
        if (lfLeftHoldMs == 0) lfLeftHoldMs = now;
        lfRightHoldMs = 0;
    }
    else
    {
        lfLeftHoldMs  = 0;
        lfRightHoldMs = 0;
    }

    if (onlyRight && (now - lfRightHoldMs) >= kLateralHoldMs)
    {
        Serial.println("[LOOKFORLINE] LINEA DERECHA -> corrigiendo IZQUIERDA");
        lfCorrecting        = true;
        lfCorrectionDir     = -1;
        lfCorrectionStartMs = now;
        lfRightHoldMs       = 0;
        // LARC.left(kCorrectionSpeed);
        return;
    }
    else if (onlyLeft && (now - lfLeftHoldMs) >= kLateralHoldMs)
    {
        Serial.println("[LOOKFORLINE] LINEA IZQUIERDA -> corrigiendo DERECHA");
        lfCorrecting        = true;
        lfCorrectionDir     = +1;
        lfCorrectionStartMs = now;
        lfLeftHoldMs        = 0;
        // LARC.right(kCorrectionSpeed);
        return;
    }

    // ── Avance normal con rampa por ultrasonido ───────────────────────────
    const float d1 = us1.getdistance();
    const float d2 = us2.getdistance();

    const bool valid1 = us1.isValid();
    const bool valid2 = us2.isValid();

    float treeCm = -1.0f;

    if (valid1 && valid2)
        treeCm = min(d1, d2);
    else if (valid1)
        treeCm = d1;
    else if (valid2)
        treeCm = d2;

    float speed = kLookFastSpeed;

    if (treeCm > 0.0f)
    {
        if (treeCm <= kUsHardStopCm)
        {
            speed = 0.0f;
        }
        else if (treeCm < kUsSlowStartCm)
        {
            float alpha = (treeCm - kUsHardStopCm) / (kUsSlowStartCm - kUsHardStopCm);
            if (alpha < 0.0f) alpha = 0.0f;
            if (alpha > 1.0f) alpha = 1.0f;
            speed = kUsMinSpeed + alpha * (kLookSlowSpeed - kUsMinSpeed);
        }
        else
        {
            speed = kLookSlowSpeed;
        }
    }

    if ((now - lastPrintMs) >= kPrintEveryMs)
    {
        lastPrintMs = now;
        /*
        Serial.print("[LOOKFORLINE] ");
        Serial.print("d1: ");        Serial.print(d1);
        Serial.print("  valid1: ");  Serial.print(valid1);
        Serial.print(" | d2: ");     Serial.print(d2);
        Serial.print("  valid2: ");  Serial.print(valid2);
        Serial.print(" | treeCm: "); Serial.print(treeCm);
        Serial.print(" | speed: ");  Serial.print(speed);
        Serial.print(" | front: ");  Serial.print(frontDetected);
        Serial.print(" | onLine: "); Serial.print(onLine);
        Serial.print(" | L: ");      Serial.print(leftDetected);
        Serial.print(" | R: ");      Serial.println(rightDetected);
        */
    }

    if (speed <= 0.0f)
    {
        Serial.println("[LOOKFORLINE] BRAKE por obstaculo cercano");
        LARC.brake();
    }
    else
    {
        Serial.print("[LOOKFORLINE] FORWARD speed = "); Serial.println(speed);
        LARC.forward(speed);
    }
}

void LARCStateMachine::handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx)
{
    vision.startBeans();

    static constexpr uint32_t kCornerEntryRampMs = 300;
    static constexpr uint32_t kCornerStopMs      = 1200;
    static constexpr uint32_t kSoftStartMs       = 500;

    static constexpr float kCornerSearchVy = 0.40f;
    static constexpr float kSoftVy         = 0.27f;

    static constexpr float kEntryMinScale   = 0.55f;
    static constexpr float kRestartMinScale = 0.35f;

    auto clamp01 = [](float x) -> float
    {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    };

    switch (action_stage)
    {
    case 0:
    {
        if (action_start_time == 0)
            action_start_time = now;

        if (cornerLEFTDetected)
        {
            LARC.brake();
            servos.intakeUpperDeploy();
            servos.intakeLowerDeploy();

            action_stage = 2;
            action_start_time = now;
            return;
        }

        float t = float(now - action_start_time) / float(kCornerEntryRampMs);
        float scale = kEntryMinScale + (1.0f - kEntryMinScale) * clamp01(t);

        LARC.setTranslation(vx * scale, kCornerSearchVy * scale);

        if ((now - action_start_time) >= kCornerEntryRampMs)
        {
            action_stage = 1;
            action_start_time = 0;
        }
        break;
    }

    case 1:
    {
        if (cornerLEFTDetected)
        {
            LARC.brake();
            servos.intakeUpperDeploy();
            servos.intakeLowerDeploy();

            action_stage = 2;
            action_start_time = now;
            return;
        }

        LARC.setTranslation(vx, kCornerSearchVy);
        break;
    }

    case 2:
    {
        LARC.brake();

        if ((now - action_start_time) >= kCornerStopMs)
        {
            action_stage = 3;
            action_start_time = now;
        }
        break;
    }

    case 3:
    {
        float t = float(now - action_start_time) / float(kSoftStartMs);
        float scale = kRestartMinScale + (1.0f - kRestartMinScale) * clamp01(t);

        LARC.setTranslation(vx * scale, kSoftVy * scale);

        if ((now - action_start_time) >= kSoftStartMs)
        {
            vision.startBeans();
            setState(STATES::BEANS);
        }
        break;
    }
    }
}


void LARCStateMachine::handleBEANS(uint32_t now, bool cornerRIGHTDetected, bool onLine, float vx)
{
    static constexpr uint32_t kLostLineTimeoutMs = 1200;

    vision.startBeans();

    if (vision.hasCriticalError())
    {
        vision.stop();
        setState(STATES::STOP);
        return;
    }

    switch (action_stage)
    {
    case 0:
{
    // ── Soft start al entrar a BEANS ─────────────────────────────
    static constexpr uint32_t kEntryRampMs  = 350;
    static constexpr float    kEntryMinScale = 0.30f;

    float vyScale = 1.0f;
    if (action_start_time == 0) action_start_time = now;

    uint32_t elapsed = now - action_start_time;
    if (elapsed < kEntryRampMs) {
        float t   = float(elapsed) / float(kEntryRampMs);
        vyScale   = kEntryMinScale + (1.0f - kEntryMinScale) * t;
    }

    // cornerRight durante rampa también debe atender
    if (cornerRIGHTDetected)
    {
        LARC.brake();
        servos.intakeUpperHome();
        servos.intakeLowerHome();
        action_start_time = now;
        action_stage = 1;
        return;
    }

    if (!onLine)
    {
        // Reset rampa si pierde línea
        action_start_time = 0;

        if (/* timer lostLine */ false) { } // tu lógica existente
        LARC.backward(kBaseSpeed);

        static uint32_t lostStart = 0;
        if (lostStart == 0) lostStart = now;
        if ((now - lostStart) >= kLostLineTimeoutMs) {
            lostStart = 0;
            vision.stop();
            setState(STATES::POOL);
        }
        return;
    }

    // Reset timer de línea perdida
    // lostStart = 0;  <- necesitas declararlo static arriba si usas este patrón

    // vx ya viene filtrado con zona muerta desde update()
    // vyScale suaviza el arranque sin tocar vx
    LARC.setTranslation(vx, -0.36f * vyScale);

    if (vision.beanBottom()) servos.intakeUpperDeploy();
    else                     servos.intakeUpperHome();

    if (vision.beanTop())    servos.intakeLowerDeploy();
    else                     servos.intakeLowerHome();

    break;
}

    case 2:
    {
        LARC.brake();

        if ((now - action_start_time) >= 300)
        {
            setPoolState(PoolSubState::FORWARD);
            setState(STATES::POOLSGOBACK);
        }
        break;
    }
    }
}


void LARCStateMachine::handlePOOLSGoBackState(uint32_t now, bool rearObstacle, bool leftDetected, bool rightDetected)
{
switch (poolState)
{
case PoolSubState::FORWARD:
{
    if (rearObstacle)
    {
        noObstacleStartMs = 0;
        setPoolState(PoolSubState::AVOID_LEFT);
    }
    else
    {
        LARC.backward(kVelocity);

        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
        {
            setState(STATES::LOOKFORLINEBACKWARDS);
        }
    }
    break;
}

case PoolSubState::AVOID_LEFT:
{
    LARC.left(0.40f);

    const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

    if (leftDetected && canChangeSide)
    {
        if (sideDetectStartMs == 0)
            sideDetectStartMs = now;

        if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_RIGHT);
        }
    }
    else
    {
        sideDetectStartMs = 0;

        if (!rearObstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }
    }

    break;
}

case PoolSubState::AVOID_RIGHT:
{
    LARC.right(0.40f);

    const bool canChangeSide = (now - poolStateStartMs) >= kMinAvoidTimeMs;

    if (rightDetected && canChangeSide)
    {
        if (sideDetectStartMs == 0)
            sideDetectStartMs = now;

        if ((now - sideDetectStartMs) >= kSideDetectHoldMs)
        {
            clearStartMs = 0;
            sideDetectStartMs = 0;
            setPoolState(PoolSubState::AVOID_LEFT);
        }
    }
    else
    {
        sideDetectStartMs = 0;

        if (!rearObstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
            }
        }
        else
        {
            clearStartMs = 0;
        }
    }

    break;
}
}
}

void LARCStateMachine::handleLookForLineBackWards(uint32_t now, bool backDetected, bool backLeftDetected, bool backRightDetected)
{

    servos.intakeUpperDeploy();
    servos.intakeLowerDeploy();

    switch (action_stage)
    {
    case 0:
    {
        if (backDetected)
        {
            setState(STATES::BENEFITSSTARTCORNER);
            return;
        }

        if (backLeftDetected && !backRightDetected)
        {
            action_stage = 1;
            action_start_time = now;
            return;
        }

        if (backRightDetected && !backLeftDetected)
        {
            action_stage = 2;
            action_start_time = now;
            return;
        }

        LARC.backward(kBaseSpeed);
        break;
    }

    case 1:
    {
        if (backDetected)
        {
            setState(STATES::BENEFITSSTARTCORNER);
            return;
        }

        LARC.right(kVelocity);

        if ((now - action_start_time) >= 500)
        {
            action_stage = 0;
        }
        break;
    }

    case 2:
    {
        if (backDetected)
        {
            setState(STATES::BENEFITSSTARTCORNER);
            return;
        }

        LARC.left(kVelocity);

        if ((now - action_start_time) >= 500)
        {
            action_stage = 0;
        }
        break;
    }
    }
}

void LARCStateMachine::handleBenefitsStartCorner(uint32_t now, bool cornerLeftDetected, float vx, bool onLine)
{
    switch (action_stage)
    {
    case 0:
    {
        if (cornerLeftDetected)
        {
            LARC.brake();
            action_start_time = now;
            action_stage = 1;
            return;
        }

        if (!onLine)
        {
            LARC.left(kBaseSpeed); // <- Provicionial LARC.stop();
            return;
        }

        LARC.setTranslation(vx, 0.40f);

        break;
    }

    case 1:
    {
        LARC.brake();
        if ((now - action_start_time) >= 1000)
        {
            setState(STATES::BENEFITS);
        }
        break;
    }
    }
}

void LARCStateMachine::handleBenefits(uint32_t now, bool cornerRIGHTDetected, float vx, bool online)
{

    switch (action_stage)
    {
    case 0:
    {
        if (!cornerRIGHTDetected)
        {
            action_stage = 1;
        }
        else
        {
            LARC.brake();
            action_start_time = now;
            action_stage = 2;
        }
        break;
    }

    case 1:
    {
        LARC.setTranslation(vx, -0.40f);

        // Here goes the rutine
        if (cornerRIGHTDetected)
        {
            action_stage = 2;
        }
        break;
    }

    case 2:
    {
        LARC.brake();

        if ((now - action_start_time) >= 1000)
        {
            setState(STATES::STOP);
        }
        break;
    }
    }
}

void LARCStateMachine::handleStopState()
{
    LARC.brake();
}