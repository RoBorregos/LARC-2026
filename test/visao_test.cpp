// Vision + Servo test — all Vision functions
//Test code testing each state for beans and benefits
//Each phase has a timeout to switch to the next phase, 
//and critical error handling to stop immediately if raspi_visao dies during beans phase
#include <Arduino.h>
#include "Vision.hpp"
#include "ServoSystem.hpp"

Vision vision(Serial);
ServoSystem servos;

uint8_t phase = 0; // 0 = wait, 1 = beans, 2 = benefits, 3 = stopped
uint32_t phaseStartMs = 0;


constexpr uint32_t kBeansPhaseDurationMs    = 60000;
constexpr uint32_t kBenefitsPhaseDurationMs = 30000;

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    servos.begin();
    vision.begin();
    vision.requestStatus();

    phase = 0;
    phaseStartMs = millis();
}

void loop()
{
    vision.update();

    const uint32_t now = millis();

    switch (phase)
    {
    case 0:
    {
        if (vision.isPiReady())
        {
            vision.clearErrors();
            vision.startBeans();
            phase = 1;
            phaseStartMs = now;
        }
        break;
    }

    case 1:
    {
        if (vision.hasCriticalError())
        {
            vision.stop();
            phase = 3;
            break;
        }

        // Bean detections - intake servos
        if (vision.beanBottom())
            servos.intakeUpperDeploy();
        else
            servos.intakeUpperHome();

        if (vision.beanTop())
            servos.intakeLowerDeploy();
        else
            servos.intakeLowerHome();

        // Separator detections - separator servo
        if (vision.warmBall())
            servos.separatorRight();
        else if (vision.coolBall())
            servos.separatorLeft();
        else
            servos.separatorCenter();

        // Switch to benefits after timeout
        if ((now - phaseStartMs) >= kBeansPhaseDurationMs)
        {
            vision.stop();
            vision.resetGuards();
            vision.clearErrors();
            vision.startBenefits();
            servos.intakeUpperHome();
            servos.intakeLowerHome();
            servos.separatorCenter();
            phase = 2;
            phaseStartMs = now;
        }
        break;
    }

    case 2:
    {
        // Box detections - benefit servo
        if (vision.isRedBox())
            servos.benefitRed();
        else if (vision.isBlueBox())
            servos.benefitBlue();
        else
            servos.benefitCenter();

        if ((now - phaseStartMs) >= kBenefitsPhaseDurationMs)
        {
            vision.stop();
            servos.benefitCenter();
            phase = 3;
        }
        break;
    }

    case 3:
        break;
    }
}