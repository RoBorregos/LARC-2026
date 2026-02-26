/**
 * @file main.cpp
 * @brief Test de sensores IR analógicos via multiplexor 74HC4067
 */

#include <Arduino.h>
#include "pins.h"
#include "mux.h"
#include "IR_mux.hpp"
#include "qtr.hpp"

Mux74HC4067 mux(
    Pins::kMuxSig,
    Pins::kMuxS0,
    Pins::kMuxS1,
    Pins::kMuxS2,
    Pins::kMuxS3
);

// Canales del mux por sensor
const uint8_t irChannels[IR_mux::N] = {
    15,
    14,
    13,
    12
};

// invertedMask = 0b0000 = ninguno invertido
IR_mux ir(mux, irChannels, 0b0000);
QTR qtr0_7(0, mux);

//  Setup 

void setup()
{

    mux.begin();
    Serial.println("Mux OK");

    Serial.begin(115200);
    while (!Serial) {}

    ir.begin();
    Serial.println(F("IR_mux listo."));
    
    qtr0_7.begin();
    Serial.println("QTR0_7 OK");
}

// Loop

void loop()
{
    
    ir.update();
    qtr0_7.update();
    Serial.print("QTR: ");
    qtr0_7.debugPrint();
    Serial.println("IR_mux:");
    ir.debugPrint();
    delay(50);

}