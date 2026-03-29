#include "instances.h"

Drive LARC;
Mux74HC4067 mux;
ServoSystem servos;
Elevator elevator;

const uint8_t irChannels[IR_mux::N] = {13, 12, 11, 10};
IR_mux ir(mux, irChannels, 0b0000);

Ultrasonic us1(Pins::kDistanceSensors[0][0], Pins::kDistanceSensors[0][1]);
Ultrasonic us2(Pins::kDistanceSensors[1][0], Pins::kDistanceSensors[1][1]);

QTR qtrFront(0, mux);

PIDController linePID(0.000035f, 0.0f, 0.00000008f, -1.0f, 1.0f);

Vision vision(Serial);

ToF tofLeft;
ToF tofRight;