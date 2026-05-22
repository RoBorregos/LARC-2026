// Test servos con modulo PCA9685
// Sale bien

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "pins.h"

Adafruit_PWMServoDriver pca(0x40);

const uint8_t servos[] = {
    Pins::kUpperIntakeServo, // 15
    Pins::kLowerIntakeServo, // 13
    Pins::kSeparatorServo,   // 12
    Pins::kBenefitServo,     // 11
    //Pins::kHolderServo       // 10
};

void moveServo(uint8_t channel, int us)
{
    pca.writeMicroseconds(channel, us);
}

void setup()
{
    Serial.begin(115200);

    Wire.begin();

    pca.begin();
    pca.setOscillatorFrequency(27000000);
    pca.setPWMFreq(50);

    delay(1000);
}

void loop()
{
    // =====================================================
    // MOVER UNO POR UNO
    // =====================================================

    for (int i = 0; i < 5; i++)
    {
        Serial.print("Moving servo channel: ");
        Serial.println(servos[i]);

        // centro
        moveServo(servos[i], 1500);
        delay(1000);

        // lado 1
        moveServo(servos[i], 1000);
        delay(1000);

        // lado 2
        moveServo(servos[i], 2000);
        delay(1000);

        // regresar centro
        moveServo(servos[i], 1500);
        delay(1000);
    }
}