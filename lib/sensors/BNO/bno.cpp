/*
 * @file bno.cpp
 * @author Ximena Patricia García Magdaleno
 * @brief Source file for the BNO class.
 * @version 0.1
 * @date 2026-01-12
*/

#include "BNO/bno.hpp"

BNO::BNO() : bno(55, 0x28, &Wire), initialized(false)
{
}

bool BNO::begin()
{
    //To begin BNO
    Serial.println("BEGIN >>>> BNO055 <<<<");

    while (!bno.begin())
    {
        Serial.println("Failed to BEGIN >>>> BNO055 <<<<");
        delay(1000);
    }
    Serial.println(">>>> BNO055 <<<< BEGINED successfully");

    delay(2000); // Give the sensor time to initialize
    bno.setExtCrystalUse(true);
    initialized = true;
    return true;
}

// Call this function in the main loop
void BNO::update()
{
    if (!initialized) //if BNO is not initialized 
        return;

    bno.getEvent(&event);
}


float BNO::wrapAngle(float angle) const
{
    // Normalize angle to [0, 360) : range
    angle = fmod(angle, 360.0f);
    if (angle < 0)
    {
        angle += 360.0f;
    }

    // Wrap to [-180, 180] : shortest way
    if (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    else if (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}


float BNO::getYaw() const
{
    float yawDeg;
    yawDeg = -wrapAngle(event.orientation.x); //-event.orientation.x; 
    return yawDeg * (M_PI/180.0f);// convert to radians
}

float BNO::getRoll() const
{
    return event.orientation.y;
}

float BNO::getPitch() const
{
    return event.orientation.z;
}


std::tuple<float, float, float> BNO::getLinealAcceleration() //acceleration tuple <x,y,z>
{
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::adafruit_vector_type_t::VECTOR_LINEARACCEL);

    return std::make_tuple(event.acceleration.x, event.acceleration.y, event.acceleration.z);
}


void BNO::getAngular()
{
    //Get on screen Yaw, Roll and Pitch values:
    update();

    float current_yaw = getYaw();
    float current_roll = getRoll();
    float current_pitch = getPitch();

    Serial.println("CURRENT ORIENTATION:");
    Serial.print("  Yaw (Z-axis):   ");
    Serial.print(current_yaw, 2);
    Serial.println("°");
    Serial.print("  Roll (Y-axis):  ");
    Serial.print(current_roll, 2);
    Serial.println("°");
    Serial.print("  Pitch (X-axis): ");
    Serial.print(current_pitch, 2);
    Serial.println("°");
}

