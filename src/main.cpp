#include <Arduino.h>
#include "subsystem/Drive/Drive.hpp"

Drive drive;

static bool    started  = false;
static uint32_t lastPrint = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    drive.begin();
    drive.holdYaw(false);   // sin BNO, solo encoders
    drive.resetOdometry();

    Serial.println("=== TEST ODOMETRIA ===");
    Serial.println("Formato: M1 M2 M3 M4 | OdoX OdoY");
}

void loop() {
    drive.update();

    // Arranca una sola vez
    if (!started) {
        started = true;
        drive.forward(0.45f);
    }

    // Para a 0.5m
    if (fabsf(drive.getOdoX()) >= 0.5f) {
        drive.brake();
    }

    // Print 10 Hz
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.print(drive.getM1Meters(), 3); Serial.print("\t");
        Serial.print(drive.getM2Meters(), 3); Serial.print("\t");
        Serial.print(drive.getM3Meters(), 3); Serial.print("\t");
        Serial.print(drive.getM4Meters(), 3); Serial.print("\t");
        Serial.print(drive.getOdoX(), 3);     Serial.print("\t");
        Serial.println(drive.getOdoY(), 3);
    }
}