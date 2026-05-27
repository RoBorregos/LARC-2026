// Test BNO085 IMU, sale bien solamente inicializa en 85
#include <Arduino.h>
#include <Wire.h>

#include "BNO/BNO.hpp"

BNO bno;

void setup()
{
    Serial.begin(115200);

    while (!Serial)
        ;

    Wire.begin();

    Serial.println("Starting BNO085 test...");

    bno.begin();

    Serial.println("BNO085 ready!");
}

void loop()
{
    bno.update();

    float yaw   = bno.getYaw();
    float pitch = bno.getPitch();
    float roll  = bno.getRoll();

    Serial.print("Yaw: ");
    Serial.print(yaw * 180.0f / PI, 2);

    Serial.print(" | Pitch: ");
    Serial.print(pitch, 2);

    Serial.print(" | Roll: ");
    Serial.println(roll, 2);

    delay(50);
}