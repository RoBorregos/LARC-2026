#include <Arduino.h>
#include "pins.h"
#include "qtr.hpp"

static constexpr uint32_t PRINT_EVERY_MS = 50;
static constexpr uint16_t LINE_THRESHOLD = 100;

static const QTR::MuxConfig kMux{
  Pins::kMuxS0,
  Pins::kMuxS1,
  Pins::kMuxS2,
  Pins::kMuxS3,
  Pins::kMuxSig
};

QTR qtrFront(0, kMux);

void setup()
{
  Serial.begin(115200);
  delay(300);

  qtrFront.begin();
  qtrFront.useDefaultCalibration(0);

  Serial.println(F("\nQTR FRONT TEST (Teensy defaults)"));
}

void loop()
{
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs < PRINT_EVERY_MS) return;
  lastMs = now;

  qtrFront.update();
  qtrFront.debugPrint();

  Serial.print(F("onLine: "));
  Serial.print(qtrFront.onLine(LINE_THRESHOLD) ? F("YES") : F("NO"));
  Serial.print(F("  POS: "));
  Serial.println(qtrFront.getPosition());
  Serial.println();
}
