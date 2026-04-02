#include "Drive.hpp"

// ─────────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────────
Drive::Drive()
    : bno_(),
      m1_ul_(Pins::kUpperMotors[0], Pins::kUpperMotors[1], Pins::kPwmPin[0], true, UL_ENC_A, UL_ENC_B, diameter),
      m2_ur_(Pins::kUpperMotors[2], Pins::kUpperMotors[3], Pins::kPwmPin[1], true, UR_ENC_A, UR_ENC_B, diameter),
      m3_ll_(Pins::kLowerMotors[0], Pins::kLowerMotors[1], Pins::kPwmPin[2], true, LL_ENC_A, LL_ENC_B, diameter),
      m4_lr_(Pins::kLowerMotors[2], Pins::kLowerMotors[3], Pins::kPwmPin[3], true, LR_ENC_A, LR_ENC_B, diameter),
      omni_(m1_ul_, m2_ur_, m3_ll_, m4_lr_),
      yawPid_(P, I, D, -kOmegaMax, +kOmegaMax),
      linePid_(0.00008f, 0.000005f, 0.0f, -kLinePidMax, +kLinePidMax)
{}

// ─────────────────────────────────────────────────────────────────
//  begin
// ─────────────────────────────────────────────────────────────────
void Drive::begin() {
    Serial.begin(115200);
    delay(500);

    analogWriteResolution(8);
    m1_ul_.begin(); m2_ur_.begin(); m3_ll_.begin(); m4_lr_.begin();

    Wire.begin();
    bno_.begin();
    bno_.update(); delay(50); bno_.update();

    yawPid_.setAngleWrapping(true);
    yawPid_.reset();

    targetYaw_      = bno_.getYaw();
    yawHoldEnabled_ = true;
    vxCmd_ = 0.0f;
    vyCmd_ = 0.0f;

    m1_ul_.setPPR(475.0f);  // kEncoders[0,1] — UR calibration
    m2_ur_.setPPR(482.0f);  // kEncoders[2,3] — UL calibration  
    m3_ll_.setPPR(495.0f);  // kEncoders[4,5] — LL calibration
    m4_lr_.setPPR(475.0f);  // kEncoders[6,7] — espejo de UR

    targetYaw_ = bno_.getYaw();
    yawHoldEnabled_ = true;
    vxCmd_ = 0.0f;
    vyCmd_ = 0.0f;
}

// ─────────────────────────────────────────────────────────────────
//  update  — llamar siempre en loop(), sin delay()
// ─────────────────────────────────────────────────────────────────
void Drive::update() {
    const uint32_t now = millis();
    if (now - lastControl_ < kControlMs) return;
    lastControl_ = now;

    bno_.update();
    updateOdometry();

    // 1) Lógica busy (driveDistance / strafeDistance / turnTo)
    updateBusy(now);

    // 2) QTR line follow — solo toca vxCmd_, deja vyCmd_ intacto
    if (lineFollowEnabled_ && qtrFront_) {
        qtrFront_->update();

        if (qtrFront_->onLine(200)) {
            const float pos   = (float)qtrFront_->getPosition();
            const float error = pos - kLineCenter;

            float correction = 0.0f;
            if (fabsf(error) > kLineDeadband) {
                correction = linePid_.update(pos, kLineCenter);
            } else {
                linePid_.reset();  // evita integral acumulada en zona muerta
            }
            vxCmd_ = correction;
        }
        // Si no hay línea: conserva último vxCmd_ (sin cambio brusco)
    }

    // 3) Yaw PID — siempre activo si está habilitado
    const float yaw = bno_.getYaw();
    float omega = 0.0f;
    if      (manualOmegaEnabled_) omega = manualOmega_;
    else if (yawHoldEnabled_)     omega = yawPid_.update(yaw, targetYaw_);
    omega = clampf(omega, -kOmegaMax, +kOmegaMax);

    omni_.MoveXYW(vxCmd_, vyCmd_, -omega);

    // 4) Debug 10 Hz
    if (now - lastPrint_ >= kPrintMs) {
        lastPrint_ = now;
        // Descomenta para depurar:
        // float err = wrapAngle(targetYaw_ - yaw);
        // Serial.printf("yaw=%.1f target=%.1f err=%.1f omega=%.3f vx=%.2f vy=%.2f\n",
        //     rad2deg(yaw), rad2deg(targetYaw_), rad2deg(err), omega, vxCmd_, vyCmd_);
    }
}

// ─────────────────────────────────────────────────────────────────
//  updateBusy — gestiona driveDistance / strafeDistance / turnTo
// ─────────────────────────────────────────────────────────────────
void Drive::updateBusy(uint32_t now) {
    if (!busy_) return;

    bool done = false;

    if (distTarget_ < 0.0f) {
        // ── Modo giro: espera error de yaw < 2°
        const float err = fabsf(yawPid_.getError());
        if (err < deg2rad(2.0f) || (now - busyStart_) > busyTimeout_)
            done = true;

    } else {
        // ── Modo distancia: usa odometría
        const float traveled = sqrtf(odoX_ * odoX_ + odoY_ * odoY_);
        if (traveled >= distTarget_ || (now - busyStart_) > busyTimeout_)
            done = true;
    }

    if (done) {
        busy_  = false;
        stop();
    }
}

// ─────────────────────────────────────────────────────────────────
//  updateOdometry
// ─────────────────────────────────────────────────────────────────
void Drive::updateOdometry() {
    const float d1 = m1_ul_.getDistanceMeters();
    const float d2 = m2_ur_.getDistanceMeters();
    const float d3 = m3_ll_.getDistanceMeters();
    const float d4 = m4_lr_.getDistanceMeters();

    // Delta desde la última lectura
    const float dd1 = d1 - prevD1_;
    const float dd2 = d2 - prevD2_;
    const float dd3 = d3 - prevD3_;
    const float dd4 = d4 - prevD4_;

    prevD1_ = d1;
    prevD2_ = d2;
    prevD3_ = d3;
    prevD4_ = d4;

    constexpr float k = 0.7071f * 0.25f;
    odoX_ += k * (-dd1 + dd2 + dd3 - dd4);
    odoY_ += k * ( dd1 + dd2 - dd3 - dd4);
}

void Drive::resetOdometry() {
    odoX_ = 0.0f;
    odoY_ = 0.0f;
    prevD1_ = prevD2_ = prevD3_ = prevD4_ = 0.0f;
    m1_ul_.resetEncoder();
    m2_ur_.resetEncoder();
    m3_ll_.resetEncoder();
    m4_lr_.resetEncoder();
}

// ─────────────────────────────────────────────────────────────────
//  driveDistance  — avanza dist metros, no bloquea el loop
// ─────────────────────────────────────────────────────────────────
void Drive::driveDistance(float meters, float speed) {
    resetOdometry();
    distTarget_  = fabsf(meters);
    busyVx_      = (meters >= 0.0f) ? +speed : -speed;
    busyVy_      = 0.0f;
    busy_        = true;
    busyStart_   = millis();
    // Timeout = 2.5× el tiempo esperado
    busyTimeout_ = (uint32_t)(fabsf(meters) / speed * 1000.0f * 2.5f);

    vxCmd_ = busyVx_;
    vyCmd_ = 0.0f;
    holdYaw(true);   // BNO mantiene recto durante el avance
}

// ─────────────────────────────────────────────────────────────────
//  strafeDistance  — movimiento lateral dist metros
// ─────────────────────────────────────────────────────────────────
void Drive::strafeDistance(float meters, float speed) {
    resetOdometry();
    distTarget_  = fabsf(meters);
    busyVx_      = 0.0f;
    busyVy_      = (meters >= 0.0f) ? +speed : -speed;
    busy_        = true;
    busyStart_   = millis();
    busyTimeout_ = (uint32_t)(fabsf(meters) / speed * 1000.0f * 2.5f);

    vxCmd_ = 0.0f;
    vyCmd_ = busyVy_;
    holdYaw(true);
}

// ─────────────────────────────────────────────────────────────────
//  turnTo  — gira a ángulo absoluto en grados
// ─────────────────────────────────────────────────────────────────
void Drive::turnTo(float targetDeg, uint32_t timeoutMs) {
    stop();                                  // detiene traslación
    setTargetYaw(deg2rad(targetDeg));        // nuevo setpoint yaw
    distTarget_  = -1.0f;                   // flag: modo giro
    busy_        = true;
    busyStart_   = millis();
    busyTimeout_ = timeoutMs;
    holdYaw(true);
}

// ─────────────────────────────────────────────────────────────────
//  QTR – seguimiento de línea
// ─────────────────────────────────────────────────────────────────
void Drive::attachQTR(QTR& qtr) {
    qtrFront_ = &qtr;
    linePid_.reset();
}

void Drive::followLine(float vySpeed) {
    if (!qtrFront_) return;
    lineFollowEnabled_ = true;
    lineSpeedVy_       = vySpeed;
    vyCmd_             = vySpeed;
    vxCmd_             = 0.0f;
    linePid_.reset();
    holdYaw(true);   // BNO mantiene orientación durante el recorrido
}

void Drive::stopLineFollow() {
    lineFollowEnabled_ = false;
    linePid_.reset();
    stop();
}

// ─────────────────────────────────────────────────────────────────
//  Movimiento básico
// ─────────────────────────────────────────────────────────────────
void Drive::forward(float speed)      { vxCmd_ = +speed; vyCmd_ = 0.0f; }
void Drive::backward(float speed)     { vxCmd_ = -speed; vyCmd_ = 0.0f; }
void Drive::right(float speed)        { vxCmd_ = 0.0f;   vyCmd_ = -speed; }
void Drive::left(float speed)         { vxCmd_ = 0.0f;   vyCmd_ = +speed; }
void Drive::diagonalLeft(float speed) { vxCmd_ = +speed; vyCmd_ = +speed; }
void Drive::stop()                    { vxCmd_ = 0.0f;   vyCmd_ = 0.0f;   }
void Drive::setTranslation(float vx, float vy) { vxCmd_ = vx; vyCmd_ = vy; }

void Drive::brake() {
    m1_ul_.stop(); m2_ur_.stop(); m3_ll_.stop(); m4_lr_.stop();
}

// ─────────────────────────────────────────────────────────────────
//  Yaw hold
// ─────────────────────────────────────────────────────────────────
void Drive::holdYaw(bool enable) {
    yawHoldEnabled_ = enable;
    if (enable) {
        targetYaw_ = bno_.getYaw();
        // resetToMeasurement evita spike de integral al reiniciar
        yawPid_.resetToMeasurement(targetYaw_, targetYaw_);
    }
}

void Drive::setTargetYaw(float yawRad) {
    targetYaw_ = yawRad;
    yawPid_.resetToMeasurement(bno_.getYaw(), yawRad);
}

float Drive::getYaw() const {
    return const_cast<Drive*>(this)->bno_.getYaw();
}

// ─────────────────────────────────────────────────────────────────
//  Omega manual
// ─────────────────────────────────────────────────────────────────
void Drive::setManualOmega(float omegaRadS) {
    manualOmegaEnabled_ = true;
    manualOmega_        = omegaRadS;
}

void Drive::clearManualOmega() {
    manualOmegaEnabled_ = false;
    manualOmega_        = 0.0f;
    yawPid_.resetToMeasurement(bno_.getYaw(), targetYaw_);
}

// ─────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────
float Drive::rad2deg(float r)               { return r * (180.0f / M_PI); }
float Drive::deg2rad(float d)               { return d * (M_PI / 180.0f); }
float Drive::clampf(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}
float Drive::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// ─────────────────────────────────────────────────────────────────
//  testKinematics
// ─────────────────────────────────────────────────────────────────
void Drive::testKinematics(float v, uint32_t T) {
    holdYaw(false);
    Serial.println("Forward");  omni_.MoveXYW(+v, 0, 0); delay(T);
    Serial.println("Backward"); omni_.MoveXYW(-v, 0, 0); delay(T);
    Serial.println("Right");    omni_.MoveXYW(0, +v, 0); delay(T);
    Serial.println("Left");     omni_.MoveXYW(0, -v, 0); delay(T);
    Serial.println("Diag++");   omni_.MoveXYW(+v,+v, 0); delay(T);
    Serial.println("Diag--");   omni_.MoveXYW(-v,-v, 0); delay(T);
    Serial.println("Stop");     omni_.Stop(); delay(1200);
}