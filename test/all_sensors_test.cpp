// ══════════════════════════════════════════════════════════════════════
//  FULL HARDWARE TEST — comment/uncomment sections with /* */ as needed
// ══════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>

#include "constants.h"
#include "pins.h"
#include "mux.h"

#include "subsystem/Drive/Drive.hpp"
#include "ServoSystem.hpp"
#include "Elevator.hpp"
#include "TCA9548A.h"
#include "tof.hpp"
#include "qtr.hpp"
#include "IR_mux.hpp"

// ══════════════════════════════════════════════════════════════════════
//  INSTANCES
// ══════════════════════════════════════════════════════════════════════

// ── Drive (motors + BNO + encoders) ──
Drive LARC;

// ── Servos ──
ServoSystem servos;

// ── Elevator ──
Elevator elevator;

// ── I2C Mux + ToF ──
TCA9548A i2cMux;
ToF tof1(Pins::kToFchFR, i2cMux, ToFType::L0X);  // channel 0
ToF tof2(Pins::kToFchFL, i2cMux, ToFType::L0X);  // channel 1
ToF tof3(2, i2cMux, ToFType::L0X);  // channel 2
ToF tof4(3, i2cMux, ToFType::L0X);  // channel 3

// ── Analog Mux 1 (QTR) ──
Mux74HC4067 mux1;                          // Sig = pin 24
QTR qtrFront(Pins::kQtrFrontFirstCh, mux1); // C0..C7
QTR qtrRear(Pins::kQtrRearFirstCh, mux1);   // C8..C15

// ── Analog Mux 2 (IR) ──
Mux74HC4067 mux2(Pins::kMuxSig2);          // Sig = pin 20, shared S0-S3
const uint8_t irChannels[IR_mux::N] = {
    Pins::kIrChFL, Pins::kIrChFR,
    Pins::kIrChBL, Pins::kIrChBR
};
IR_mux ir(mux2, irChannels);

// ── Timing ──
uint32_t lastPrintMs = 0;
constexpr uint32_t kPrintMs = 50;

// ══════════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println(F("=== FULL HARDWARE TEST ==="));
    
    /* ── Drive ── */
    LARC.begin();
    LARC.holdYaw(true);
    LARC.setTargetYaw(LARC.getYaw());

    /* ── Servos ── */
    //servos.begin();

    /* ── Elevator ── */
    elevator.begin();
    pinMode(Pins::kLimitSwitch, INPUT_PULLUP);

    /* ── I2C Mux + ToF ── */
    Wire.begin();
    i2cMux.begin();
    tof1.begin();
    tof2.begin();
    tof3.begin();
    tof4.begin();
    tof1.setMaxRange(500);
    tof2.setMaxRange(500);
    tof3.setMaxRange(500);
    tof4.setMaxRange(500);

    /* ── QTR (mux1) ── */
    qtrFront.begin();
    qtrFront.useDefaultCalibration(0);
    qtrRear.begin();
    qtrRear.useDefaultCalibration(1);

    /* ── IR (mux2) ── */
    ir.begin();

    Serial.println(F("=== INIT DONE ===\n"));
}

// ══════════════════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════════════════
void loop()
{
    const uint32_t now = millis();

    /* -- Drive -- */
    LARC.update();

    /* -- ToF -- */
    tof1.update();
    tof2.update();
    tof3.update();
    tof4.update();

    /* -- QTR -- */
    qtrFront.update();
    qtrRear.update();

    /* -- IR -- */
    ir.update();

    /* -- Servos -- */
    servos.update();

     /* -- Elevator -- */

    // ── Print at kPrintMs rate ──
    if (now - lastPrintMs < kPrintMs) return;
    lastPrintMs = now;

    // ════════════════════════════════════════════════════════
    //  1) DRIVE — yaw + commands (Drive already prints internally at 10Hz)
    // ════════════════════════════════════════════════════════
    /*
    // Drive prints yaw/target/error/omega automatically in update()
    // To test movement, uncomment one at a time:
    // LARC.forward(0.3);
    // LARC.backward(0.3);
    // LARC.left(0.3);
    // LARC.right(0.3);
    // LARC.setTranslation(0.2, -0.3);
    // LARC.stop();
    */

    // ════════════════════════════════════════════════════════
    //  2) SERVOS — cycle through positions
    // ════════════════════════════════════════════════════════

    // ════════════════════════════════════════════════════════
    //  3) ELEVATOR + LIMIT SWITCH
    // ════════════════════════════════════════════════════════
    
    //Serial.print(F("Limit switch: "));
    //Serial.println(digitalRead(Pins::kLimitSwitch) == LOW ? "PRESSED" : "RELEASED");
    // To test: uncomment one direction at a time
    // elevator.ElevatorPosition(1);  // UP
    // elevator.ElevatorPosition(2);  // DOWN
    // elevator.ElevatorPosition(0);  // STOP
    

    // ════════════════════════════════════════════════════════
    //  5) TOF SENSORS (via TCA9548A)
    // ════════════════════════════════════════════════════════
    
    /*Serial.print(F("ToF  |  ch0 (right): "));
    Serial.print(tof1.getDistanceMm());
    Serial.print(F(" mm  init="));
    Serial.println(tof1.isInitialized());
    Serial.print(F("ToF  |  ch1 (left): "));
    Serial.print(tof2.getDistanceMm());
    Serial.print(F(" mm  init="));
    Serial.println(tof2.isInitialized());
    Serial.print(F("ToF  |  ch2: "));
    Serial.print(tof3.getDistanceMm());
    Serial.print(F(" mm  init="));
    Serial.println(tof3.isInitialized());
    Serial.print(F("ToF  |  ch4: "));
    Serial.print(tof4.getDistanceMm());
    Serial.print(F(" mm  init="));
    Serial.println(tof4.isInitialized());*/
    

    // ════════════════════════════════════════════════════════
    //  6) I2C MUX SCAN (run once then comment out)
    // ════════════════════════════════════════════════════════
    
    //i2cMux.scanAll();
    //i2cMux.scanDirect();
    

    // ════════════════════════════════════════════════════════
    //  7) QTR FRONT (mux1, channels 0-7)
    // ════════════════════════════════════════════════════════
    
    /*Serial.println(F("-- QTR FRONT --"));
    qtrFront.debugPrint();
    Serial.print(F("onLine: ")); Serial.println(qtrFront.onLine());
    

    // ════════════════════════════════════════════════════════
    //  8) QTR REAR (mux1, channels 8-15)
    // ════════════════════════════════════════════════════════
    
    Serial.println(F("-- QTR REAR --"));
    qtrRear.debugPrint();
    Serial.print(F("onLine: ")); Serial.println(qtrRear.onLine());*/
    

    // ════════════════════════════════════════════════════════
    //  9) IR SENSORS (mux2, channels 10-13)
    // ════════════════════════════════════════════════════════
    
    //Serial.println(F("-- IR --"));
    //ir.debugPrint();
    

    // ════════════════════════════════════════════════════════
    // 10) RAW MUX1 — all 16 channels (QTR debug)
    // ════════════════════════════════════════════════════════
    /*
    Serial.print(F("MUX1: "));
    mux1.debugPrint();
    */

    // ════════════════════════════════════════════════════════
    // 11) RAW MUX2 — all 16 channels (IR debug)
    // ════════════════════════════════════════════════════════
    
    /*Serial.print(F("MUX2: "));
    mux2.debugPrint();*/
    
}