/**
 * @file main.cpp
 * @brief Test state machine: STANDBY → BEANS → BENEFITS → STOP
 * 
 * 
 * HOW VISION WORKS                             │
 * 
 * The Raspberry Pi runs a "dispatcher" service that starts
 * automatically when the Pi boots. It sits idle, waiting for
 * commands from the Teensy over USB serial.
 *
 * The dispatcher does NOT run vision by default. it only
 * launches the vision scripts when the Teensy tells it to.
 * 
 * COMMANDS (Teensy → Pi):
 *  Serial.write(0xA0) = Start BEANS phase (runs raspi_visao.py + separator).
 *  Serial.write(0xA3)  = Start BENEFITS phase (stops beans, runs benefits.py).
 *  Serial.write(0xA1)  = Stop ALL scripts.
 *  Serial.write(0xA2)  = Ask for status.
 * 
 * RESPONSES (Pi → Teensy):
 *  0xB0 = dispatcher ready (Pi just booted, waiting)
 *  0xB1 = scripts starting up
 *  0xB2 = BEANS phase running OK.
 *  0xB3 = all scripts stopped.
 *  0xB4 = BENEFITS phase running Ok
 *  0xE0-0xEF = error (followed by 1 error code byte).
 * 
 * VISION DATA (Pi → Teensy, forwarded by dispatcher):
 *  [0xFF, left_hit, right_hit]  — from raspi_visao.py
 *  left_hit:  1 = bean on bottom side, 0 = nothing
 *  right_hit: 1 = bean on top side, 0 = nothing.
 * 
 *  [0xFD, warm_hit, cool_hit]   — from separator_visao.py (ball sorting)
 *  warm_hit: 1 = warm ball detected, 0 = nothing
 *  cool_hit: 1 = cool ball detected, 0 = nothing
 * 
 * [0xFE, box_type] (from benefits.py)
 *  box_type: 0 = no box, 1 = red box, 2 = blue box
 * 
 * IMPORTANT NOTES: 
 *  - Do NOT use Serial.println() while Pi is connected!
 *     Text output pollutes the serial line and breaks the protocol
 * - The dispatcher auto-reconnects if you unplug/replug the Teensy. Just send the start command again.
 * - To check dispatcher status on the Pi:
 *    journalctl -u pi-dispatcher.service -f
 * - To restart the dispatcher on the Pi:
 *    sudo systemctl restart pi-dispatcher.service
 * 
 */

#include <Arduino.h>
#include "pins.h"
#include "mux.h"
#include "IR_mux.hpp"

// ── Objects ──────────────────────────────────────────────────────────
Mux74HC4067 mux;

const uint8_t irChannels[IR_mux::N] = {13, 12, 11, 10};
IR_mux ir(mux, irChannels, 0b1111);

// ══════════════════════════════════════════════════════════════════════
//  DISPATCHER COMMANDS ( bytes we send TO the Pi)
// ══════════════════════════════════════════════════════════════════════
#define CMD_START_BEANS    0xA0  // launch raspi_visao.py + separator_visao.py
#define CMD_STOP           0xA1  // kill all running scripts.
#define CMD_STATUS         0xA2  // ask Pi what's happening.
#define CMD_START_BENEFITS 0xA3  // launch benefits.py (auto-stops beans first).

// ══════════════════════════════════════════════════════════════════════
//  DISPATCHER RESPONSES — bytes the Pi sends BACK to us
// ══════════════════════════════════════════════════════════════════════
#define ACK_READY     0xB0  // dispatcher booted, waiting for commands
#define ACK_STARTING  0xB1  // scripts are launching
#define ACK_RUNNING   0xB2  // BEANS phase running OK
#define ACK_STOPPED   0xB3  // all scripts stopped
#define ACK_BENEFITS  0xB4  // BENEFITS phase running OK
// Errors: 0xE0 = raspi_visao crashed, 0xE1 = separator crashed
//         0xE3 = camera not found, 0xE4 = benefits crashed
//
//"I received your message and here's my response."
// So when the Teensy sends 0xA0 (start beans), the Pi replies with 0xB2 (ACK_RUNNING) 
//meaning "acknowledged, beans are running." 
// It's just a name convention to make the code readable
//you could call them REPLY_RUNNING or RESPONSE_RUNNING instead, it's the same thing.
// The important part is that the Teensy and Pi have a shared understanding of what these bytes mean in the context of their communication protocol.
// ══════════════════════════════════════════════════════════════════════
//  VISION DATA — received from Pi, forwarded by the dispatcher
//
//  Bean detection:  [0xFF, left_hit, right_hit]
//    left_hit  = 1 if bean detected on bottom camera side
//    right_hit = 1 if bean detected on top camera side.
//
//  Box detection:   [0xFE, box_type].
//    box_type = 0 (none), 1 (red), 2 (blue).
// ══════════════════════════════════════════════════════════════════════
static byte visionLeft = 0; // bean detection left side.
static byte visionRight = 0; // bean detection right side.
static byte boxType = 0;   // box color: 0=none, 1=red, 2=blue

// ── Dispatcher status flags ──────────────────────────────────────────
bool piReady = false;          // Pi dispatcher is up and waiting
bool beansRunning = false;      // BEANS phase scripts are running
bool benefitsRunning = false;   // BENEFITS phase script is running

// ── State machine ────────────────────────────────────────────────────
enum State { STANDBY, BEANS, BENEFITS, STOP };
State currentState = STANDBY;
bool phaseSent = false;  // makes sure we send each command only once

// ══════════════════════════════════════════════════════════════════════
//  readSerial() — handles ALL incoming data from the Pi
//
//  Call this every loop(). It reads bytes and figures out what they are based on the first byte:
//    0xB0-0xB4 → dispatcher status update
//    0xFF      → bean vision data (2 more bytes follow)
//    0xFE      → box vision data (1 more byte follows)
//    0xE0-0xEF → error (1 more byte follows)
// ══════════════════════════════════════════════════════════════════════
void readSerial()
{
    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (b) {
            // ── Dispatcher ACKs ──
            case ACK_READY:    piReady = true;        break;
            case ACK_STARTING: /* scripts launching */ break;
            case ACK_RUNNING:  beansRunning = true;    break;
            case ACK_BENEFITS: benefitsRunning = true; break;
            case ACK_STOPPED:
                beansRunning = false;
                benefitsRunning = false;
                break;

            // ── Bean vision: [0xFF, left, right] ──
            case 0xFF:
                if (Serial.available() >= 2) {
                    visionLeft  = Serial.read();
                    visionRight = Serial.read();
                }
                break;

            // ── Box vision: [0xFE, boxType] ──
            case 0xFE:
                if (Serial.available() >= 1) {
                    boxType = Serial.read();
                }
                break;

            // ── Errors (0xE0-0xEF + 1 error code byte) ──
            default:
                if (b >= 0xE0 && b <= 0xEF) {
                    if (Serial.available()) Serial.read();
                    beansRunning = false;
                    benefitsRunning = false;
                }
                break;
        }
    }
}

// ══════════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(115200);  // USB serial to Pi dispatcher
    delay(1000);

    mux.begin();
    ir.begin();

    currentState = STANDBY;
    phaseSent = false;
}

// ══════════════════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════════════════
void loop()
{
    ir.update();
    readSerial();  // ALWAYS call, handles ACKs + vision data

    bool BL = ir.getState(IR_mux::BL);
    bool BR = ir.getState(IR_mux::BR);

    switch (currentState)
    {
        // ── STANDBY ──────────────────────────────────────────────
        // Vision is NOT running. Pi dispatcher is idle.
        // BL sensor triggers transition to BEANS.
        case STANDBY:
        {
            if (BL) {
                currentState = BEANS;
                phaseSent = false;
            }
            break;
        }

        // ── BEANS ────────────────────────────────────────────────
        // Sends 0xA0 once to Pi launches raspi_visao + separator.
        // visionLeft / visionRight update every frame (~15 FPS).
        // BR sensor triggers transition to BENEFITS.
        case BEANS:
        {
            if (!phaseSent) {
                Serial.write(CMD_START_BEANS);
                phaseSent = true;
            }

            // Use vision data for servos:
            // if (visionLeft)  servos.intakeUpperDeploy();
            // else             servos.intakeUpperHome();
            // if (visionRight) servos.intakeLowerDeploy();
            // else             servos.intakeLowerHome();

            if (BR) {
                currentState = BENEFITS;
                phaseSent = false;
            }
            break;
        }

        // ── BENEFITS ─────────────────────────────────────────────
        // Sends 0xA3 once to Pi stops beans, launches benefits.py.
        // boxType updates every frame: 0=none, 1=red, 2=blue.
        // BL sensor triggers STOP.
        case BENEFITS:
        {
            if (!phaseSent) {
                Serial.write(CMD_START_BENEFITS);
                phaseSent = true;
            }

            // Use box data:
            // if (boxType == 1) { /* red box */ }
            // if (boxType == 2) { /* blue box */ }

            if (BL) {
                currentState = STOP;
                phaseSent = false;
            }
            break;
        }

        // ── STOP ─────────────────────────────────────────────────
        // Sends 0xA1 once → Pi kills all scripts. Done.
        case STOP:
        {
            if (!phaseSent) {
                Serial.write(CMD_STOP);
                phaseSent = true;
            }
            break;
        }
    }
}