// src/main.cpp
#include <Arduino.h>
#include "mux.h"
#include "pins.h"

// Test for the 74HC4067 multiplexer.
// Prints raw analog values of all 16 channels in a loop.

Mux74HC4067 mux;

void setup() {
  Serial.begin(115200);
  delay(300);
  mux.begin(); // configura pines y selecciona canal 0
  Serial.println(F("\n[MUX] Leyendo 16 canales (0-15) ..."));
  Serial.println(F("Formato: ch:valor"));
}

void loop() {
  mux.debugPrint();
}