#include <Arduino.h>
#include "qtr.hpp"

QTR qtrFront(0);
QTR qtrRear(8);

static constexpr uint32_t CAL_MS = 5000;

void calibrateOne(QTR& qtr, uint16_t outMin[QTR::N], uint16_t outMax[QTR::N], uint32_t ms)
{
  // init extremes
  for (uint8_t i = 0; i < QTR::N; i++) {
    outMin[i] = 65535;
    outMax[i] = 0;
  }

  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    qtr.update();
    const uint16_t* r = qtr.getRaw();   // valores crudos 0 a 4095 aprox

    for (uint8_t i = 0; i < QTR::N; i++) {
      if (r[i] < outMin[i]) outMin[i] = r[i];
      if (r[i] > outMax[i]) outMax[i] = r[i];
    }
    delay(5);
  }
}

static void printArray(const char* name, const uint16_t a[QTR::N])
{
  Serial.print(name);
  Serial.print("[QTR::N] = {");
  for (uint8_t i = 0; i < QTR::N; i++) {
    Serial.print(a[i]);
    if (i < QTR::N - 1) Serial.print(", ");
  }
  Serial.println("};");
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  qtrFront.begin();
  qtrRear.begin();

  Serial.println("=== CALIBRACIÓN QTR (RAW MIN/MAX) ===");
  Serial.println("Mueve el robot para que cada sensor vea BLANCO y NEGRO durante 5s por cada arreglo.\n");

  uint16_t fMin[QTR::N], fMax[QTR::N];
  uint16_t rMin[QTR::N], rMax[QTR::N];

  Serial.println("[1/2] Calibrando FRONT...");
  calibrateOne(qtrFront, fMin, fMax, CAL_MS);

  Serial.println("[2/2] Calibrando REAR...");
  calibrateOne(qtrRear, rMin, rMax, CAL_MS);

  Serial.println("\n--- COPIA ESTO A qtr.cpp ---");
  printArray("FRONT_MIN", fMin);
  printArray("FRONT_MAX", fMax);
  printArray("REAR_MIN",  rMin);
  printArray("REAR_MAX",  rMax);
}

void loop() {}