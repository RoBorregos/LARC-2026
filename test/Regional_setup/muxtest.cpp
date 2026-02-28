// src/main.cpp
#include <Arduino.h>
#include "mux.h"   // tu clase del 74HC4067
#include "pins.h"

// ---------------------------
// Placeholders de pines MUX
// ---------------------------
static constexpr uint8_t PIN_MUX_SIG = Pins::kMuxSig;  // SIG (común) del mux -> pin analógico
static constexpr uint8_t PIN_MUX_S0  = Pins::kMuxS0;  // S0
static constexpr uint8_t PIN_MUX_S1  = Pins::kMuxS1;  // S1
static constexpr uint8_t PIN_MUX_S2  = Pins::kMuxS2;  // S2
static constexpr uint8_t PIN_MUX_S3  = Pins::kMuxS3;  // S3

Mux74HC4067 mux(PIN_MUX_SIG, PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3);

void setup() {
  Serial.begin(115200);
  delay(300);

  mux.begin();                 // configura pines y selecciona canal 0
  mux.setSigMode(INPUT);       // por claridad (tu begin ya lo hace)

  Serial.println(F("\n[MUX] Leyendo 16 canales (0-15) ..."));
  Serial.println(F("Formato: ch:valor"));
}

void loop() {
  for (uint8_t ch = 0; ch < 16; ch++) {
    uint16_t v = mux.read(ch);
    Serial.print(ch);
    Serial.print(':');
    Serial.print(v);
    if (ch < 15) Serial.print('\t');
  }
  Serial.println();

  delay(50); // ajusta si quieres más rápido/lento
}