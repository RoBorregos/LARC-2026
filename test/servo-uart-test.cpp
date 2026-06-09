#include <Arduino.h>
#include <Servo.h>

/*
 * servos-demo — standalone UART-driven intake servo mover
 * =======================================================
 * Minimal demo firmware: NO project libraries, only <Servo.h>.
 * Listens for the 3-byte vision protocol sent by
 * Vision/visao-debug-uart.py and drives the two intake servos.
 *
 *   Upper intake servo : pin 4   (RIGHT window in the debug viewer)
 *   Lower intake servo : pin 2   (LEFT  window in the debug viewer)
 *
 * Each servo rests at IDLE_ANGLE (55 deg) while no bean is seen — and
 * also while idle / no link — then snaps to HIT_ANGLE (88 deg) while a
 * bean sits in that side's trigger zone.
 *
 * Protocol  (Mac -> Teensy):  0xFF  <lower_hit>  <upper_hit>
 *   0xFF        frame header (never appears as a data byte, hits are 0/1)
 *   lower_hit   0 / 1  -> lower servo (pin 2)
 *   upper_hit   0 / 1  -> upper servo (pin 4)
 *
 * The onboard LED blinks as an "alive" heartbeat. If the Mac stops
 * streaming frames the link watchdog returns both servos to idle.
 */

// ---- pins ----
const uint8_t UPPER_PIN = 2;   // upper intake (RIGHT in viewer)
const uint8_t LOWER_PIN = 4;   // lower intake (LEFT  in viewer)
const int     LED       = 13;

// ---- angles ----
const uint8_t IDLE_ANGLE = 55; // no bean / idle / no link
const uint8_t HIT_ANGLE  = 88; // bean detected in trigger zone

// ---- protocol ----
const uint8_t FRAME_HEADER     = 0xFF;
const unsigned long LINK_TIMEOUT_MS = 1000; // no frames this long -> idle

Servo upperServo;
Servo lowerServo;

// ---- frame parser state ----
enum RxState { WAIT_HEADER, WAIT_LOWER, WAIT_UPPER };
RxState rxState  = WAIT_HEADER;
uint8_t lowerHit = 0;

// ---- last applied angles (skip redundant writes) ----
uint8_t curUpper = 255;
uint8_t curLower = 255;

// ---- heartbeat / link watchdog ----
unsigned long lastBlink   = 0;
bool          ledState    = false;
unsigned long lastFrameMs = 0;

void moveUpper(uint8_t angle) {
  if (angle == curUpper) return;
  upperServo.write(angle);
  curUpper = angle;
}

void moveLower(uint8_t angle) {
  if (angle == curLower) return;
  lowerServo.write(angle);
  curLower = angle;
}

void applyHits(uint8_t lower_hit, uint8_t upper_hit) {
  moveLower(lower_hit ? HIT_ANGLE : IDLE_ANGLE);
  moveUpper(upper_hit ? HIT_ANGLE : IDLE_ANGLE);
}

void readSerial() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (rxState) {
      case WAIT_HEADER:
        if (b == FRAME_HEADER) rxState = WAIT_LOWER;
        break;
      case WAIT_LOWER:
        lowerHit = b ? 1 : 0;
        rxState  = WAIT_UPPER;
        break;
      case WAIT_UPPER:
        applyHits(lowerHit, b ? 1 : 0);
        lastFrameMs = millis();
        rxState     = WAIT_HEADER;
        break;
    }
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  // No while(!Serial) — run immediately whether or not the port is open.

  upperServo.attach(UPPER_PIN);
  lowerServo.attach(LOWER_PIN);

  applyHits(0, 0);   // both servos to idle (55)
  lastFrameMs = millis();
}

void loop() {
  unsigned long now = millis();

  // ---- heartbeat ----
  if (now - lastBlink >= 500) {
    lastBlink = now;
    ledState  = !ledState;
    digitalWrite(LED, ledState);
  }

  // ---- always read/decode whatever the Mac sends ----
  readSerial();

  // ---- link watchdog: no frames -> fall back to idle ----
  if (now - lastFrameMs >= LINK_TIMEOUT_MS) {
    applyHits(0, 0);
  }
}
