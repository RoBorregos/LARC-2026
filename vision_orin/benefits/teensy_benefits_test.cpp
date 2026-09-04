/*
   teensy_benefits_test.cpp — the Teensy half of benefits.py

   PAIRS WITH   vision_orin/benefits/benefits.py
   TUNE WITH    vision_orin/benefits/benefits_debug.py
  
   WHAT IT DOES
     Holds the BENEFITS phase and reports ONLY the two doors: every phase
     change of each door, the angle it went to, and how many times each has
     opened. The intakes and separator are parked for the whole stage — that
     is the protocol, not this sketch being polite.
  
   HOW TO RUN — with the Orin
     1 Do NOT open a serial monitor — the dispatcher owns that port.
     3. journalctl -u larc-dispatcher -f (on orin)
  
   HOW TO RUN — from your pc (not orin)
     1. Same upload.
     2. python3 vision_orin/benefits/benefits_debug.py --teensy
  
 */

#include <Arduino.h>
#include "ServoSystem.hpp"
#include "VisionLink.hpp"

using namespace Constants::ServoConfig;

constexpr uint32_t kSettleMs       = 750;
constexpr uint32_t kRequestRetryMs = 1000;
constexpr uint32_t kHeartbeatMs    = 2000;

ServoSystem servos;
VisionLink  vision(Serial, servos);

ServoSystem::BenefitPhase pDoor[2] = {ServoSystem::BenefitPhase::ARMED,
                                      ServoSystem::BenefitPhase::ARMED};
bool     pLink = false;
uint32_t opens[2] = {0, 0};
uint32_t lastRequest = 0, lastBeat = 0;
bool     started = false;

const char *doorName(ServoSystem::BenefitPhase p)
{
    switch (p)
    {
        case ServoSystem::BenefitPhase::OPEN:       return "OPEN";
        case ServoSystem::BenefitPhase::WAIT_REARM: return "WAIT_REARM";
        default:                                    return "ARMED";
    }
}

void setup()
{
    Serial.begin(Constants::VisionConfig::kSerialBaud);
    servos.begin();   // closes and re-arms both doors
    vision.begin();

    Serial.println();
    Serial.println("[ben] teensy_benefits_test — BENEFITS phase, doors only");
    Serial.printf("[ben] door1 ch%u closed=%u open=%u   door2 ch%u closed=%u open=%u\n",
                  kCalib[BENEFIT_1].channel, kBenefit1Closed, kBenefit1Open,
                  kCalib[BENEFIT_2].channel, kBenefit2Closed, kBenefit2Open);
    Serial.printf("[ben] open window = %lu ms   red box -> door1, blue box -> door2\n",
                  (unsigned long)kBenefitOpenMs);
}

void loop()
{
    vision.update();
    const uint32_t now = millis();

    if (now >= kSettleMs && vision.phase() != VisionProto::Phase::BENEFITS &&
        now - lastRequest >= kRequestRetryMs)
    {
        if (!started) { Serial.println("[ben] requesting BENEFITS"); started = true; }
        lastRequest = now;
        vision.resetGuards();
        vision.startBenefits();
    }

    for (uint8_t i = 0; i < 2; ++i)
    {
        const ServoSystem::BenefitPhase phase = servos.benefitPhase(i);
        if (phase == pDoor[i])
            continue;

        if (phase == ServoSystem::BenefitPhase::OPEN)
            opens[i]++;

        Serial.printf("[ben] door%u %-10s -> %-10s %u deg   (opens: %lu / %lu)\n",
                      i + 1, doorName(pDoor[i]), doorName(phase),
                      servos.lastAngle(i == 0 ? BENEFIT_1 : BENEFIT_2),
                      (unsigned long)opens[0], (unsigned long)opens[1]);
        pDoor[i] = phase;
    }

    if (vision.isLinkUp() != pLink)
    {
        pLink = vision.isLinkUp();
        Serial.printf("[ben] link %s\n", pLink ? "UP" : "LOST -> SAFE");
    }

    if (now - lastBeat >= kHeartbeatMs)
    {
        lastBeat = now;
        Serial.printf("[ben] phase=%s door1=%s(%u deg) door2=%s(%u deg)  "
                      "opens=%lu/%lu  acc=%lu rej=%lu\n",
                      vision.inBenefitsPhase() ? "BENEFITS" : "not-BENEFITS",
                      doorName(servos.benefitPhase(0)), servos.lastAngle(BENEFIT_1),
                      doorName(servos.benefitPhase(1)), servos.lastAngle(BENEFIT_2),
                      (unsigned long)opens[0], (unsigned long)opens[1],
                      (unsigned long)vision.stats().accepted,
                      (unsigned long)vision.stats().rejected);
    }
}
