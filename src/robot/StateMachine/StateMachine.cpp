#include <Arduino.h>
#include <math.h>
#include "StateMachine.h"
#include "robot/instances/instances.h"
#include "constants.h"
#include "Vision.hpp"

namespace
{
    static constexpr float kBaseSpeed = Constants::PID::kcurrentVelocity;

    // ToF thresholds (mm)
    static constexpr uint16_t kObstacleDistanceMm = 50;
    static constexpr uint16_t kTofValidMaxMm      = 500;

    static constexpr uint32_t kStartIgnoreTimeMs    = 2200;
    static constexpr uint32_t kClearDelayMs         = 800;
    static constexpr uint32_t kNoObstacleToCornerMs = 3800;

    static constexpr uint32_t kMinAvoidTimeMs   = 250;
    static constexpr uint32_t kSideDetectHoldMs = 80;

    // ========= velocidades para OdomMovement =========
    // Ajustadas pensando en tu test, donde 60 rpm sí se comporta bien.
    static constexpr float kOdomForwardRpm     = 60.0f;
    static constexpr float kOdomBackwardRpm    = 60.0f;
    static constexpr float kOdomLateralRpm     = 60.0f;
    static constexpr float kOdomCorrectionRpm  = 55.0f;
    static constexpr float kOdomAvoidRpm       = 60.0f;
    static constexpr float kOdomStartRpm       = 60.0f;

    // ========= promedio filtrado de odometría =========
    static constexpr float kOdomAvgAlpha = 0.85f;
    static float gAvgForwardProgress = 0.0f;
    static float gAvgLateralProgress = 0.0f;

    inline void resetOdomAverages()
    {
        gAvgForwardProgress = 0.0f;
        gAvgLateralProgress = 0.0f;
    }

    inline void updateOdomAverages(float forwardNow, float lateralNow)
    {
        gAvgForwardProgress =
            kOdomAvgAlpha * gAvgForwardProgress +
            (1.0f - kOdomAvgAlpha) * forwardNow;

        gAvgLateralProgress =
            kOdomAvgAlpha * gAvgLateralProgress +
            (1.0f - kOdomAvgAlpha) * lateralNow;
    }

    const __FlashStringHelper *mainStateName(STATES state)
    {
        switch (state)
        {
        case STATES::START:                return F("START");
        case STATES::POOL:                 return F("POOL");
        case STATES::LOOKFORLINE:          return F("LOOKFORLINE");
        case STATES::LOOKFORCORNER:        return F("LOOKFORCORNER");
        case STATES::BEANS:                return F("BEANS");
        case STATES::BEANSGOBACK:          return F("BEANSGOBACK");
        case STATES::POOLSGOBACK:          return F("POOLSGOBACK");
        case STATES::LOOKFORLINEBACKWARDS: return F("LOOKFORLINEBACKWARDS");
        case STATES::BENEFITSSTARTCORNER:  return F("BENEFITSSTARTCORNER");
        case STATES::BENEFITS:             return F("BENEFITS");
        case STATES::STOP:                 return F("STOP");
        default:                           return F("UNKNOWN");
        }
    }

    const __FlashStringHelper *poolStateName(PoolSubState state)
    {
        switch (state)
        {
        case PoolSubState::FORWARD:     return F("FORWARD");
        case PoolSubState::AVOID_LEFT:  return F("AVOID_LEFT");
        case PoolSubState::AVOID_RIGHT: return F("AVOID_RIGHT");
        default:                        return F("UNKNOWN");
        }
    }

    inline float clamp01(float x)
    {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }
}

LARCStateMachine::LARCStateMachine()
{
}

void LARCStateMachine::begin()
{
    currentState = STATES::POOL;
    poolState = PoolSubState::FORWARD;

    state_start_time = millis();
    action_start_time = 0;
    action_stage = 0;

    clearStartMs = 0;
    noObstacleStartMs = 0;
    sideDetectStartMs = 0;
    poolStateStartMs = 0;

    visionLeft = 0;
    visionRight = 0;

    lfCorrecting = false;
    lfCorrectionDir = 0;
    lfCorrectionStartMs = 0;
    lfLeftHoldMs = 0;
    lfRightHoldMs = 0;

    pinMode(limitSwitch, INPUT_PULLUP);

    vision.begin();
    vision.requestStatus();

    ir.begin();

    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);

    qtrRear.begin();
    qtrRear.useDefaultCalibration(1);

    tofLeft.begin();
    tofRight.begin();

    tofLeft.setMaxRange(500);
    tofRight.setMaxRange(500);

    tofLeft.setUpdateInterval(30);
    tofRight.setUpdateInterval(30);

    odomMove_.begin();
    odomMove_.resetPose();
    odomMove_.captureCurrentYawTarget();
    resetOdomAverages();
}

void LARCStateMachine::update()
{
    // Sensores
    ir.update();
    qtrFront.update();
    qtrRear.update();
    vision.update();
    tofLeft.update();
    tofRight.update();
    odomMove_.update();

    const uint32_t now = millis();
    startStateTime();

    // SOLO actualizar LARC en estados que usan setTranslation/brake como control principal
    switch (currentState)
    {
    case STATES::LOOKFORCORNER:
    case STATES::BEANS:
    case STATES::BEANSGOBACK:
    case STATES::BENEFITSSTARTCORNER:
    case STATES::BENEFITS:
        LARC.update();
        break;

    default:
        // En los estados de odometría NO dejar que LARC mande motores
        break;
    }

    updateOdomAverages(
        odomMove_.getForwardProgress(),
        odomMove_.getLateralProgress()
    );
    /*
    static uint32_t lastOdomPrint = 0;
    if (now - lastOdomPrint >= 120)
    {
        lastOdomPrint = now;
        Serial.print("x: ");
        Serial.print(odomMove_.getX(), 4);
        Serial.print(" y: ");
        Serial.print(odomMove_.getY(), 4);
        Serial.print(" th: ");
        Serial.print(odomMove_.getThetaDeg(), 2);
        Serial.print(" avgF: ");
        Serial.print(gAvgForwardProgress, 4);
        Serial.print(" avgL: ");
        Serial.println(gAvgLateralProgress, 4);
    }
        */


    const int linePos = qtrFront.getPosition();
    const bool onLine = qtrFront.onLine();
    const bool onLineRear = qtrRear.onLine();

    const float lineError = linePos - Constants::LineFollower::kSetpoint;
    const float lineCorr =
        (fabsf(lineError) > 150.0f)
            ? linePID.update(linePos, Constants::LineFollower::kSetpoint)
            : 0.0f;

    const float vx = -lineCorr;

    const bool FL = ir.getState(IR_mux::FL);
    const bool FR = ir.getState(IR_mux::FR);
    const bool BL = ir.getState(IR_mux::BL);
    const bool BR = ir.getState(IR_mux::BR);

    const bool frontLeftDetectedLine  = FL;
    const bool frontRightDetectedLine = FR;
    const bool backLeftDetectedLine   = BL;
    const bool backRightDetectedLine  = BR;

    const bool frontDetectedLine = (FL || FR);
    const bool backDetected      = (BL || BR);
    const bool leftDetectedPool  = (FL || BL);
    const bool rightDetectedPool = (FR || BR);

    const uint16_t dLeftMm  = tofLeft.getDistanceMm();
    const uint16_t dRightMm = tofRight.getDistanceMm();

    static constexpr uint16_t kTofValidMinMm = 40;
    static constexpr uint16_t kTofValidMaxMm = 500;

    const bool leftTofOk  = tofLeft.isInitialized()  && dLeftMm  >= kTofValidMinMm && dLeftMm  <= kTofValidMaxMm;
    const bool rightTofOk = tofRight.isInitialized() && dRightMm >= kTofValidMinMm && dRightMm <= kTofValidMaxMm;

    const bool obstacleLeftNow  = leftTofOk  && dLeftMm  < kObstacleDistanceMm;
    const bool obstacleRightNow = rightTofOk && dRightMm < kObstacleDistanceMm;

    static bool obstacleLatched = false;
    static uint32_t obstacleClearStartMs = 0;
    static constexpr uint32_t kObstacleReleaseMs = 200;

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

    static uint32_t lastOdomPrint = 0;
    if (now - lastOdomPrint >= 120)
    {
        lastOdomPrint = now;

        Serial.print("x: ");
        Serial.print(odomMove_.getX(), 4);
        Serial.print(" y: ");
        Serial.print(odomMove_.getY(), 4);
        Serial.print(" th: ");
        Serial.print(odomMove_.getThetaDeg(), 2);

        Serial.print(" avgF: ");
        Serial.print(gAvgForwardProgress, 4);
        Serial.print(" avgL: ");
        Serial.print(gAvgLateralProgress, 4);

        Serial.print(" | VLX_L: ");
        Serial.print(dLeftMm);
        Serial.print(" mm");

        Serial.print(" | VLX_R: ");
        Serial.print(dRightMm);
        Serial.print(" mm");

        Serial.print(" | leftOk: ");
        Serial.print(leftTofOk);

        Serial.print(" | rightOk: ");
        Serial.print(rightTofOk);

        Serial.print(" | obsL: ");
        Serial.print(obstacleLeftNow);

        Serial.print(" | obsR: ");
        Serial.println(obstacleRightNow);
    }

    const bool obstacle = obstacleLatched;

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
        handleBenefits(now, backLeftDetectedLine, vx, onLineRear);
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

    lfCorrecting = false;
    lfCorrectionDir = 0;
    lfCorrectionStartMs = 0;
    lfLeftHoldMs = 0;
    lfRightHoldMs = 0;

    vision.resetGuards();
    linePID.reset();

    odomMove_.resetPose();
    odomMove_.captureCurrentYawTarget();
    resetOdomAverages();

    Serial.println(mainStateName(currentState));
}

void LARCStateMachine::startStateTime()
{
    if (state_start_time == 0)
    {
        state_start_time = millis();
    }
}

void LARCStateMachine::setPoolState(PoolSubState newState)
{
    if (poolState == newState)
        return;

    poolState = newState;
    clearStartMs = 0;
    sideDetectStartMs = 0;
    poolStateStartMs = millis();

    odomMove_.resetPose();
    odomMove_.captureCurrentYawTarget();
    resetOdomAverages();

    Serial.println(poolStateName(poolState));
}

void LARCStateMachine::readVision()
{
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

    const bool limitPressed = (digitalRead(limitSwitch) == HIGH);

    if (limitPressed != lastLimitPressed)
    {
        if (limitPressed) Serial.println("LIMIT SWITCH PRESIONADO");
        else              Serial.println("LIMIT SWITCH LIBERADO");

        lastLimitPressed = limitPressed;
    }

    switch (action_stage)
    {
    case 0:
        if (!limitPressed)
        {
            elevator.ElevatorPosition(2);
            odomMove_.stop();
            LARC.brake();
        }
        else
        {
            elevator.ElevatorPosition(0);
            odomMove_.stop();
            LARC.brake();
            action_start_time = now;
            action_stage = 1;
        }
        break;

    case 1:
        elevator.ElevatorPosition(0);
        odomMove_.stop();
        LARC.brake();

        if ((now - action_start_time) >= 2000)
        {
            action_start_time = now;
            action_stage = 2;
        }
        break;

    case 2:
        elevator.ElevatorPosition(1);
        odomMove_.stop();
        LARC.brake();

        if ((now - action_start_time) >= 9000)
        {
            elevator.ElevatorPosition(0);
            odomMove_.stop();
            LARC.brake();
            action_start_time = now;
            action_stage = 3;
        }
        break;

    case 3:
        elevator.ElevatorPosition(0);
        odomMove_.stop();
        LARC.brake();

        if ((now - action_start_time) >= 3000)
        {
            action_start_time = now;
            action_stage = 4;
            odomMove_.resetPose();
            odomMove_.captureCurrentYawTarget();
            resetOdomAverages();
        }
        break;

    case 4:
    {
        elevator.ElevatorPosition(0);

        // igual que en tu test: movimiento cardinal por OdomMovement
        odomMove_.forward(kOdomStartRpm);

        static constexpr uint32_t kStartPassToPools = 1000;

        if ((now - action_start_time) >= kStartIgnoreTimeMs)
        {
            setPoolState(PoolSubState::FORWARD);
            setState(STATES::POOL);
        }
        else if (backDetected)
        {
            if ((now - action_start_time) >= kStartPassToPools)
            {
                setPoolState(PoolSubState::FORWARD);
                setState(STATES::POOL);
            }
        }
        break;
    }
    }
}

void LARCStateMachine::handlePoolState(uint32_t now, bool obstacle, bool leftDetected, bool rightDetected)
{
    vision.stop();
    vision.clearErrors();

    switch (poolState)
    {
    case PoolSubState::FORWARD:
    {
        static bool     lineCorrectionActive  = false;
        static uint32_t lineCorrectionStartMs = 0;
        static int8_t   lineCorrectionDir     = 0;

        static constexpr uint32_t kLineCorrectionMs  = 120;
        static constexpr float    kForwardAvgTargetM = 0.50f;
        static constexpr float    kForwardTolM       = 0.03f;

        if (lineCorrectionActive)
        {
            if ((now - lineCorrectionStartMs) < kLineCorrectionMs)
            {
                if (lineCorrectionDir < 0)
                    odomMove_.left(kOdomCorrectionRpm);
                else
                    odomMove_.right(kOdomCorrectionRpm);
                return;
            }

            lineCorrectionActive  = false;
            lineCorrectionStartMs = 0;
            lineCorrectionDir     = 0;

            odomMove_.stop();
            odomMove_.resetPose();
            odomMove_.captureCurrentYawTarget();
            resetOdomAverages();
        }

        // prioridad sensor obstáculo
        if (obstacle)
        {
            noObstacleStartMs = 0;
            setPoolState(PoolSubState::AVOID_LEFT);
            return;
        }

        // prioridad corrección lateral por sensores
        if (rightDetected)
        {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = -1;
            odomMove_.left(kOdomCorrectionRpm);
            return;
        }
        else if (leftDetected)
        {
            lineCorrectionActive  = true;
            lineCorrectionStartMs = now;
            lineCorrectionDir     = +1;
            odomMove_.right(kOdomCorrectionRpm);
            return;
        }

        // movimiento principal con odometría, como el test
        odomMove_.forward(kOdomForwardRpm);

        // respaldo odométrico
        if (gAvgForwardProgress >= (kForwardAvgTargetM - kForwardTolM))
        {
            setState(STATES::LOOKFORLINE);
            return;
        }

        // respaldo temporal
        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
        {
            setState(STATES::LOOKFORLINE);
            return;
        }

        break;
    }

    case PoolSubState::AVOID_LEFT:
    {
        static constexpr float kAvoidLatTargetM = 0.22f;
        static constexpr float kAvoidLatTolM    = 0.02f;

        odomMove_.left(kOdomAvoidRpm);

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
                return;
            }
        }
        else
        {
            sideDetectStartMs = 0;
        }

        if (gAvgLateralProgress >= (kAvoidLatTargetM - kAvoidLatTolM) && !obstacle)
        {
            noObstacleStartMs = 0;
            setPoolState(PoolSubState::FORWARD);
            return;
        }

        if (!obstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
                return;
            }
        }
        else
        {
            clearStartMs = 0;
        }

        break;
    }

    case PoolSubState::AVOID_RIGHT:
    {
        static constexpr float kAvoidLatTargetM = 0.22f;
        static constexpr float kAvoidLatTolM    = 0.02f;

        odomMove_.right(kOdomAvoidRpm);

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
                return;
            }
        }
        else
        {
            sideDetectStartMs = 0;
        }

        if (gAvgLateralProgress >= (kAvoidLatTargetM - kAvoidLatTolM) && !obstacle)
        {
            noObstacleStartMs = 0;
            setPoolState(PoolSubState::FORWARD);
            return;
        }

        if (!obstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
                return;
            }
        }
        else
        {
            clearStartMs = 0;
        }

        break;
    }
    }
}

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

    static constexpr float    kTofHardStopMmLf = 70.0f;
    static constexpr uint32_t kCorrectionMs    = 180;
    static constexpr uint32_t kLateralHoldMs   = 50;

    if (frontDetected && onLine)
    {
        lfCorrecting  = false;
        lfLeftHoldMs  = 0;
        lfRightHoldMs = 0;
        odomMove_.stop();
        setState(STATES::LOOKFORCORNER);
        return;
    }

    if (lfCorrecting)
    {
        if ((now - lfCorrectionStartMs) < kCorrectionMs)
        {
            if (lfCorrectionDir < 0)
                odomMove_.left(45.0f);
            else
                odomMove_.right(45.0f);
            return;
        }

        lfCorrecting  = false;
        lfLeftHoldMs  = 0;
        lfRightHoldMs = 0;
    }

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
        lfCorrecting        = true;
        lfCorrectionDir     = -1;
        lfCorrectionStartMs = now;
        lfRightHoldMs       = 0;
        return;
    }
    else if (onlyLeft && (now - lfLeftHoldMs) >= kLateralHoldMs)
    {
        lfCorrecting        = true;
        lfCorrectionDir     = +1;
        lfCorrectionStartMs = now;
        lfLeftHoldMs        = 0;
        return;
    }

    const uint16_t d1 = tofLeft.getDistanceMm();
    const uint16_t d2 = tofRight.getDistanceMm();

    const bool valid1 = tofLeft.isInitialized()  && d1 > 0 && d1 < 500;
    const bool valid2 = tofRight.isInitialized() && d2 > 0 && d2 < 500;

    float treeMm = -1.0f;
    if (valid1 && valid2) treeMm = min((float)d1, (float)d2);
    else if (valid1)      treeMm = d1;
    else if (valid2)      treeMm = d2;

    if (treeMm > 0.0f && treeMm <= kTofHardStopMmLf)
    {
        odomMove_.stop();
        return;
    }

    odomMove_.forward(60.0f);
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

    switch (action_stage)
    {
    case 0:
    {
        if (action_start_time == 0)
            action_start_time = now;

        if (cornerLEFTDetected)
        {
            odomMove_.stop();
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
            odomMove_.stop();
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
        odomMove_.stop();
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
        static constexpr uint32_t kEntryRampMs   = 350;
        static constexpr float    kEntryMinScale = 0.30f;

        float vyScale = 1.0f;
        if (action_start_time == 0) action_start_time = now;

        uint32_t elapsed = now - action_start_time;
        if (elapsed < kEntryRampMs)
        {
            float t = float(elapsed) / float(kEntryRampMs);
            vyScale = kEntryMinScale + (1.0f - kEntryMinScale) * t;
        }

        if (cornerRIGHTDetected)
        {
            odomMove_.stop();
            LARC.brake();
            servos.intakeUpperHome();
            servos.intakeLowerHome();
            action_start_time = now;
            action_stage = 1;
            return;
        }

        if (!onLine)
        {
            action_start_time = 0;

            // aquí sí es cardinal simple
            odomMove_.backward(kOdomBackwardRpm);

            static uint32_t lostStart = 0;
            if (lostStart == 0) lostStart = now;
            if ((now - lostStart) >= kLostLineTimeoutMs)
            {
                lostStart = 0;
                vision.stop();
                setState(STATES::POOL);
            }
            return;
        }

        // movimiento combinado: se deja con setTranslation
        LARC.setTranslation(vx, -0.36f * vyScale);

        if (vision.beanBottom()) servos.intakeUpperDeploy();
        else                     servos.intakeUpperHome();

        if (vision.beanTop())    servos.intakeLowerDeploy();
        else                     servos.intakeLowerHome();

        break;
    }

    case 1:
    {
        odomMove_.stop();
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

void LARCStateMachine::handleBEANSGoBackState(uint32_t now, bool cornerLEFTDetected, bool onLine, float vx)
{
    const bool limitPressed = (digitalRead(limitSwitch) == HIGH);

    switch (action_stage)
    {
    case 0:
    {
        if (!limitPressed)
        {
            elevator.ElevatorPosition(0);
            odomMove_.stop();
            LARC.brake();
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
            LARC.brake();
            servos.intakeUpperHome();
            servos.intakeLowerHome();
            action_start_time = now;
            action_stage = 2;
            return;
        }

        if (!onLine)
        {
            odomMove_.stop();
            LARC.stop();
            return;
        }

        LARC.setTranslation(vx, 0.40f);

        if (visionLeft)
            servos.intakeUpperDeploy();
        else
            servos.intakeUpperHome();

        if (visionRight)
            servos.intakeLowerDeploy();
        else
            servos.intakeLowerHome();

        break;
    }

    case 2:
    {
        odomMove_.stop();
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
        static constexpr float kBackwardAvgTargetM = 0.50f;
        static constexpr float kBackwardTolM       = 0.03f;

        if (rearObstacle)
        {
            noObstacleStartMs = 0;
            setPoolState(PoolSubState::AVOID_LEFT);
            return;
        }

        odomMove_.backward(kOdomBackwardRpm);

        if (gAvgForwardProgress >= (kBackwardAvgTargetM - kBackwardTolM))
        {
            setState(STATES::LOOKFORLINEBACKWARDS);
            return;
        }

        if (noObstacleStartMs == 0)
            noObstacleStartMs = now;

        if ((now - noObstacleStartMs) >= kNoObstacleToCornerMs)
        {
            setState(STATES::LOOKFORLINEBACKWARDS);
            return;
        }
        break;
    }

    case PoolSubState::AVOID_LEFT:
    {
        static constexpr float kAvoidLatTargetM = 0.22f;
        static constexpr float kAvoidLatTolM    = 0.02f;

        odomMove_.left(kOdomAvoidRpm);

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
                return;
            }
        }
        else
        {
            sideDetectStartMs = 0;
        }

        if (gAvgLateralProgress >= (kAvoidLatTargetM - kAvoidLatTolM) && !rearObstacle)
        {
            noObstacleStartMs = 0;
            setPoolState(PoolSubState::FORWARD);
            return;
        }

        if (!rearObstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
                return;
            }
        }
        else
        {
            clearStartMs = 0;
        }

        break;
    }

    case PoolSubState::AVOID_RIGHT:
    {
        static constexpr float kAvoidLatTargetM = 0.22f;
        static constexpr float kAvoidLatTolM    = 0.02f;

        odomMove_.right(kOdomAvoidRpm);

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
                return;
            }
        }
        else
        {
            sideDetectStartMs = 0;
        }

        if (gAvgLateralProgress >= (kAvoidLatTargetM - kAvoidLatTolM) && !rearObstacle)
        {
            noObstacleStartMs = 0;
            setPoolState(PoolSubState::FORWARD);
            return;
        }

        if (!rearObstacle)
        {
            if (clearStartMs == 0)
                clearStartMs = now;

            if ((now - clearStartMs) >= kClearDelayMs)
            {
                noObstacleStartMs = 0;
                setPoolState(PoolSubState::FORWARD);
                return;
            }
        }
        else
        {
            clearStartMs = 0;
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
            odomMove_.stop();
            odomMove_.resetPose();
            odomMove_.captureCurrentYawTarget();
            resetOdomAverages();
            return;
        }

        if (backRightDetected && !backLeftDetected)
        {
            action_stage = 2;
            action_start_time = now;
            odomMove_.stop();
            odomMove_.resetPose();
            odomMove_.captureCurrentYawTarget();
            resetOdomAverages();
            return;
        }

        odomMove_.backward(kOdomBackwardRpm);
        break;
    }

    case 1:
    {
        if (backDetected)
        {
            setState(STATES::BENEFITSSTARTCORNER);
            return;
        }

        odomMove_.right(kOdomLateralRpm);

        if ((now - action_start_time) >= 500)
        {
            action_stage = 0;
            odomMove_.stop();
            odomMove_.resetPose();
            odomMove_.captureCurrentYawTarget();
            resetOdomAverages();
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

        odomMove_.left(kOdomLateralRpm);

        if ((now - action_start_time) >= 500)
        {
            action_stage = 0;
            odomMove_.stop();
            odomMove_.resetPose();
            odomMove_.captureCurrentYawTarget();
            resetOdomAverages();
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
            odomMove_.stop();
            LARC.brake();
            action_start_time = now;
            action_stage = 1;
            return;
        }

        if (!onLine)
        {
            odomMove_.left(kOdomLateralRpm);
            return;
        }

        LARC.setTranslation(vx, 0.40f);
        break;
    }

    case 1:
    {
        odomMove_.stop();
        LARC.brake();

        if ((now - action_start_time) >= 1000)
        {
            setState(STATES::BENEFITS);
        }
        break;
    }
    }
}

void LARCStateMachine::handleBenefits(uint32_t now, bool cornerRIGHTDetected, float vx, bool onLine)
{
    (void)onLine;

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
            odomMove_.stop();
            LARC.brake();
            action_start_time = now;
            action_stage = 2;
        }
        break;
    }

    case 1:
    {
        LARC.setTranslation(vx, -0.40f);

        if (cornerRIGHTDetected)
        {
            action_stage = 2;
            action_start_time = now;
        }
        break;
    }

    case 2:
    {
        odomMove_.stop();
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
    odomMove_.stop();
    LARC.brake();
}