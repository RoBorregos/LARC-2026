
#include <Arduino.h>
#include "StateMachine.h"
#include "robot/instances/instances.h"
#include "constants.h"
#include "Vision.hpp"
#include "testOdometry.hpp"

namespace
{
    static constexpr float kVelocity = 0.48f;
    static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

    static constexpr uint32_t kInitializedStoppedMs = 9000;
    static constexpr uint32_t kStartIgnoreTimeMs = 2200;    // Time to ignore IR's at the START point
    static constexpr uint32_t kClearDelayMs = 1100;          // Tiempo para cambiar nuevamente a Forward
    static constexpr uint32_t kNoObstacleToCornerMs = 3800; // Time without obstacle to go forward and LOOKFORLINE -> tal vez disminuir
    static constexpr uint32_t kCornerDeployWazitMs = 1800;

    static constexpr uint32_t kMinAvoidTimeMs = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;
    static constexpr uint16_t kTofTargetMm = 120;   // distancia deseada al árbol
    static constexpr uint16_t kTofHardStopMm = 105; // stop de seguridad

    static constexpr float kTofMinSpeed = 0.10f;
    static constexpr float kTofMaxSpeed = 0.35f;
    static constexpr float kObstacleDistanceCm = 30.0f;


    static constexpr float kDistKp = 0.0012f;
    static constexpr float kDistKi = 0.0f;
    static constexpr float kDistKd = 0.00015f;

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
    
    currentState = STATES::START; // siempre en START
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

    ir.begin();
    qtrRear.begin();
    qtrRear.useDefaultCalibration(1);

    odomMove_.begin();
    odomMove_.resetPose();
    odomMove_.captureCurrentYawTarget();
    //resetOdomAverages();
}

void LARCStateMachine::update()
{
    //LARC.update();
    ir.update();
    qtrFront.update();
    vision.update();
    tofLeft.update();
    tofRight.update();
    odomMove_.update();

    const uint32_t now = millis();
    startStateTime();

    const int linePos = qtrFront.getPosition(); // Para el PID → más suave
    const bool onLine = qtrFront.onLine();
    const float lineCorr = linePID.update(linePos, Constants::LineFollower::kSetpoint);
    const float vx = -lineCorr;

    const bool FL = ir.getState(IR_mux::FL);
    const bool FR = ir.getState(IR_mux::FR);
    const bool BL = ir.getState(IR_mux::BL);
    const bool BR = ir.getState(IR_mux::BR);

    // Print to debug
    //ir.debugPrint();
    //qtrFront.debugPrint();
    //Serial.print("linePos: ");
    //Serial.println(linePos); // To know the value for the center of the qtr

    static uint32_t debugPrintMs = 0;
    if ((now - debugPrintMs) >= 100)
    {
    debugPrintMs = now;
    
    // Odometría
    Serial.print(F(" ❤ Odometria❤ | X:"));    Serial.print(odomMove_.getX(),   3);
    Serial.print(F(" Y:"));      Serial.print(odomMove_.getY(),   3);
    Serial.print(F(" Yaw:"));    Serial.print(odomMove_.getThetaDeg(), 1);
    Serial.print(F(" UL:"));     Serial.print(odomMove_.getRpmUL(), 0);
    Serial.print(F(" UR:"));     Serial.print(odomMove_.getRpmUR(), 0);
    Serial.print(F(" LL:"));     Serial.print(odomMove_.getRpmLL(), 0);
    Serial.print(F(" LR:"));     Serial.print(odomMove_.getRpmLR(), 0);
    
    // Estado actual
    Serial.print(F(" ❤ State❤ | ST:")); Serial.print((int)currentState); Serial.print(")");
    Serial.print(F(" PS:")); Serial.print((int)poolState);

    // ToF
    Serial.print(F(" ❤ Tof❤ | TL:"));  Serial.print(tofLeft.getDistanceCm(),  1);
    Serial.print(F(" TR:"));    Serial.print(tofRight.getDistanceCm(), 1);
    Serial.print(F(" vL:"));    Serial.print(tofLeft.isValid());
    Serial.print(F(" vR:"));    Serial.print(tofRight.isValid());

    // Obstáculo
    //Serial.print(F(" | OBS:")); Serial.print(obstacle);
    //Serial.print(F(" OL:"));    Serial.print(obstacleLeftNow);
    //Serial.print(F(" OR:"));    Serial.print(obstacleRightNow);

    // IR
    Serial.print(F(" ❤ IR's❤ | FL:")); Serial.print(FL);
    Serial.print(F(" FR:"));   Serial.print(FR);
    Serial.print(F(" BL:"));   Serial.print(BL);
    Serial.print(F(" BR:"));   Serial.print(BR);

    // Línea
    Serial.print(F(" ❤ qtr| onLine:")); Serial.print(onLine);
    Serial.print(F(" vx:")); Serial.print(vx);
    Serial.print(F(" lPos:"));  Serial.print(qtrFront.getPosition());
    Serial.println();
    }
    
    const bool frontLeftDetectedLine = FL; // Also used for corner
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine = BL;
    const bool backRightDetectedLine = BR;
    const bool frontDetectedLine = (FL || FR); // Hacer que con el qtr tambien detecte linea
    const bool backDetected = (BL || BR);
    const bool leftDetectedPool = (FL || BL);
    const bool rightDetectedPool = (FR || BR);

    // DESPUÉS
    const bool obstacleLeftNow  = tofLeft.isValid()  && tofLeft.getDistanceCm()  < kObstacleDistanceCm;
    const bool obstacleRightNow = tofRight.isValid() && tofRight.getDistanceCm() < kObstacleDistanceCm;
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
            elevator.ElevatorPosition(0); // bajar (2)
            odomMove_.stop();
        }
        else
        {
            elevator.ElevatorPosition(0);
            odomMove_.stop();
            // Serial.println("ELEVADOR ABAJO -> STOP 2000 ms");
            action_start_time = now;
            action_stage = 1;
        }
        break;

    case 1:
        // Stop 2000 ms
        elevator.ElevatorPosition(0);
        odomMove_.stop();

        if ((now - action_start_time) >= 2000)
        {
            // Serial.println("ELEVADOR SUBIENDO 9000 ms");
            action_start_time = now;
            action_stage = 2;
        }
        break;

    case 2:
        // Goes up 9000 ms
        elevator.ElevatorPosition(0); //subir (1)
        odomMove_.stop();

        if ((now - action_start_time) >= 9000)
        {
            elevator.ElevatorPosition(0);
            odomMove_.stop();
            // Serial.println("ELEVADOR DETENIDO -> STOP 3000 ms");
            action_start_time = now;
            action_stage = 3;
        }
        break;

    case 3:
        // Stop 3000 ms
        elevator.ElevatorPosition(0);
        odomMove_.stop();

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


        float speed = 0.48f;
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
        odomMove_.stop();
            else
        odomMove_.forward(59.0f);

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
    static constexpr float    kLineCorrectionSpeed = 0.48f;
    static constexpr float    kNormalSpeed = 59.0f; // velocidad que se usa


    if (lineCorrectionActive)
    {
        if ((now - lineCorrectionStartMs) < kLineCorrectionMs)
        {
            if (lineCorrectionDir < 0)
                odomMove_.left(kNormalSpeed);

                //LARC.left(kLineCorrectionSpeed);
            else
                odomMove_.right(kNormalSpeed);
                //LARC.right(kLineCorrectionSpeed);

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
            odomMove_.left(kNormalSpeed);
            break;
        }
        else if (leftDetected)
        {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = +1; // detecta izquierda -> corrige derecha
            odomMove_.right(kNormalSpeed);
            break;
        }

        odomMove_.forward(kNormalSpeed);
        //LARC.setTranslation(0.48f, 0.08);
        //LARC.forward(kVelocity);

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
        {
            setState(STATES::LOOKFORLINE);
        }
    }
    break;
}

    case PoolSubState::AVOID_LEFT:
    {
        odomMove_.left(59.0f);
        //LARC.left(0.48f);

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
        odomMove_.right(59.0f);
        //LARC.right(kVelocity);

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

void LARCStateMachine::handleLookForLineState(uint32_t now,
                                              bool frontDetected,
                                              bool leftDetected,
                                              bool rightDetected,
                                              bool onLine)
{

    // ── Prioridad 1: línea frontal detectada -> ir a LOOKFORCORNER ────────
    if (frontDetected && onLine)
    {
        lfCorrecting  = false;
        lfLeftHoldMs  = 0;
        lfRightHoldMs = 0;
        Serial.println("[LOOKFORLINE] FRONT DETECTED -> LOOKFORCORNER");
        odomMove_.stop();
        setState(STATES::LOOKFORCORNER);
        return;
    }

    // ── Avance recto hasta encontrar la línea ─────────────────────────────
    odomMove_.forward(58.0f);
}

void LARCStateMachine::handleLookForCornerState(uint32_t now, bool cornerLEFTDetected, float vx)
{
    vision.startBeans();

    static constexpr uint32_t kCornerStopMs = 1200;
    static constexpr uint32_t kSoftStartMs  = 500;

    switch (action_stage)
    {
    case 0:
    {
        if (cornerLEFTDetected)
        {
            odomMove_.stop();
            action_stage = 1;
            action_start_time = now;
            return;
        }
        const int error = 2900 - qtrFront.getPosition(); // ← invertido
        const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
        odomMove_.setTranslation(-59.0f, corr);
        break;
    }

    case 1:
    {
        
        odomMove_.stop();

        if ((now - action_start_time) >= kCornerStopMs)
        {
            action_stage = 2;
            action_start_time = now;
        }
        break;
    }

    case 2:
    {
        odomMove_.stop();

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

    //vision.startBeans();

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
    if (cornerRIGHTDetected)
    {
        odomMove_.stop();
        action_start_time = now;
        action_stage = 1;
        return;
    }

    if (!onLine)
    {
        if (action_start_time == 0)
            action_start_time = now;

        odomMove_.backward(60.0f);

        if ((now - action_start_time) >= kLostLineTimeoutMs)
        {
            vision.stop();
            setState(STATES::POOLSGOBACK);
        }

        return;
    }

    action_start_time = 0;

    const int error = 2500 - qtrFront.getPosition(); // ← invertido
    const float corr = constrain(error * 0.03f, -30.0f, 30.0f);
    odomMove_.setTranslation(+59.0f, corr);

    break;
    }

    case 1:
    {
        odomMove_.stop();
        //LARC.brake();
        if ((now - action_start_time) >= 1000)
        {
            action_start_time = 0;
            setState(STATES::BEANSGOBACK);
        }
        break;
    }
    }
}

void LARCStateMachine::handleBEANSGoBackState(uint32_t now, bool cornerLEFTDetected, bool onLine, float vx)
{

    const bool limitPressed = (digitalRead(limitSwitch) == HIGH);

    switch (action_stage)
    {
    case 0:
    {
        // Bajar elevador hasta tocar limit
        if (!limitPressed)
        {
            elevator.ElevatorPosition(0); //Bajar (2)
            odomMove_.stop();
            return;
        }

        elevator.ElevatorPosition(0);
        action_stage = 1;
        action_start_time = now;
        return;
    }

    case 1:
    {
        if (cornerLEFTDetected)
        {
            odomMove_.stop();
            //servos.intakeUpperHome();
            //servos.intakeLowerHome();
            action_start_time = now;
            action_stage = 2;
            return;
        }

        if (!onLine)
        {
            LARC.stop();
            return;
        }

        odomMove_.setTranslation(vx * 50.0f, 0.48f); // 0.45f
        /*
        if (visionLeft)
            servos.intakeUpperDeploy();
        else
            servos.intakeUpperHome();

        if (visionRight)
            servos.intakeLowerDeploy();
        else
            servos.intakeLowerHome();
            */
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
    LARC.left(0.48f);

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
    LARC.right(0.48f);

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

    //servos.intakeUpperDeploy();
    //servos.intakeLowerDeploy();

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

        odomMove_.backward(kBaseSpeed);
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

        LARC.setTranslation(vx, 0.48f);

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
        LARC.setTranslation(vx, -0.48f);

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