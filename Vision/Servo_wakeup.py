#!/usr/bin/env python3
"""servo_wakeup.py — slow sweep up or down, then release pins.
Usage: servo_wakeup.py [up|down]    (default: up)
"""
from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import time, sys

Device.pin_factory = LGPIOFactory()

UPPER_PIN, LOWER_PIN = 12, 13
REST_U, REST_L   = 155, 155 # physical resting position 155, 155
DEPLOY_U, DEPLOY_L = 50, 50 # raised position 50, 50
STEP_DEG   = 1
STEP_DELAY = 0.01

def angle_to_value(a): return (a / 90.0) - 1.0

def main():
    direction = sys.argv[1] if len(sys.argv) > 1 else 'up'
    if direction not in ('up', 'down'):
        sys.exit(f"[ERROR] direction must be 'up' or 'down', got {direction!r}")

    if direction == 'up':
        start_u, end_u = REST_U, DEPLOY_U
        start_l, end_l = REST_L, DEPLOY_L
    else:  # down
        start_u, end_u = DEPLOY_U, REST_U
        start_l, end_l = DEPLOY_L, REST_L

    u = Servo(UPPER_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
    l = Servo(LOWER_PIN, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
    try:
        u.value = angle_to_value(start_u)
        l.value = angle_to_value(start_l)
        time.sleep(0.3)

        dir_u = 1 if end_u > start_u else -1
        dir_l = 1 if end_l > start_l else -1
        steps_u = abs(end_u - start_u)
        steps_l = abs(end_l - start_l)
        max_steps = max(steps_u, steps_l)

        for i in range(0, max_steps + 1, STEP_DEG):
            au = start_u + dir_u * min(i, steps_u)
            al = start_l + dir_l * min(i, steps_l)
            u.value = angle_to_value(au)
            l.value = angle_to_value(al)
            time.sleep(STEP_DELAY)
    finally:
        u.value = None
        l.value = None
        u.close()
        l.close()

if __name__ == "__main__":
    main()
    sys.exit(0)