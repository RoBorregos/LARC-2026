from gpiozero import Servo
from gpiozero.pins.lgpio import LGPIOFactory
from gpiozero import Device
import time

Device.pin_factory = LGPIOFactory()

# ══════════════════════════════════════════════════════════════════════
#  TUNING CONSTANTS
# ══════════════════════════════════════════════════════════════════════

# ── Intake servos (pins 12, 13) ──
INTAKE_UPPER_HOME   = 50
INTAKE_UPPER_DEPLOY = 81
INTAKE_LOWER_HOME   = 50
INTAKE_LOWER_DEPLOY = 85

# ── Separator servo (pins 17, 27) ──
HOLDER_HOME         = 50
HOLDER_RELEASE      = 90
HOLDER_DEBOUNCE     = 2  # consecutive black detections required to trigger
HOLDER_PULSE_MS     = 400 # how long holder stays in RELEASE before auto-returning
HOLDER_COOLDOWN_MS  = 80 # dead time after retract before next trigger allowed

SEPARATOR_MATURE    = 30 # warm color - mature bin
SEPARATOR_IMMATURE  = 130 # cool color - immature bin
SEPARATOR_DEBOUNCE  = 2 # consecutive same-color detections before moving

# ── Servo hold/detach (applies to all) ──
SERVO_HOLD_MS       = 400 # how long PWM stays active after a move

# ══════════════════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════════════════

def _angle_to_value(angle: int) -> float:
    return (angle / 90.0) - 1.0


# ══════════════════════════════════════════════════════════════════════
#  BASIC SERVO (debounce + auto-detach)
# ══════════════════════════════════════════════════════════════════════

class ServoMotor:
    def __init__(self, gpio_pin: int, debounce: int = 1, hold_ms: int = SERVO_HOLD_MS):
        self._servo = Servo(gpio_pin, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
        self._gpio_pin = gpio_pin
        self._servo.value = None
        self._current_angle = None
        self._pending_angle = None
        self._pending_count = 0
        self._debounce = debounce
        self._hold_ms = hold_ms
        self._detach_at = 0.0

    def request(self, angle: int):
        if angle == self._current_angle:
            self._pending_angle = None
            self._pending_count = 0
            return
        if angle == self._pending_angle:
            self._pending_count += 1
        else:
            self._pending_angle = angle
            self._pending_count = 1
        if self._pending_count >= self._debounce:
            self._current_angle = angle
            self._servo.value = _angle_to_value(angle)
            self._detach_at = time.monotonic() + (self._hold_ms / 1000.0)
            self._pending_angle = None
            self._pending_count = 0

    def force(self, angle: int):
        """Bypass debounce — immediate move."""
        self._current_angle = angle
        self._servo.value = _angle_to_value(angle)
        self._detach_at = time.monotonic() + (self._hold_ms / 1000.0)
        self._pending_angle = None
        self._pending_count = 0

    def update(self):
        if self._detach_at and time.monotonic() >= self._detach_at:
            self._servo.value = None
            self._detach_at = 0.0

    def close(self):
        self._servo.value = None
        self._servo.close()


# ══════════════════════════════════════════════════════════════════════
#  HOLDER SERVO
# ══════════════════════════════════════════════════════════════════════

class HolderServo:
    IDLE      = 0
    RELEASING = 1
    COOLDOWN  = 2

    def __init__(self, gpio_pin: int,
                 home_angle: int = HOLDER_HOME,
                 release_angle: int = HOLDER_RELEASE,
                 debounce: int = HOLDER_DEBOUNCE,
                 pulse_ms: int = HOLDER_PULSE_MS,
                 cooldown_ms: int = HOLDER_COOLDOWN_MS):
        self._motor = ServoMotor(gpio_pin, debounce=1, hold_ms=SERVO_HOLD_MS)
        self._home = home_angle
        self._release = release_angle
        self._debounce = debounce
        self._pulse_s = pulse_ms / 1000.0
        self._cooldown_s = cooldown_ms / 1000.0
        self._trigger_count = 0
        self._state = self.IDLE
        self._phase_until = 0.0

    def home(self):
        self._motor.force(self._home)
        self._state = self.IDLE

    def trigger(self, detected: bool):
        if self._state != self.IDLE:
            return
        if detected:
            self._trigger_count += 1
            if self._trigger_count >= self._debounce:
                self._motor.force(self._release)
                self._state = self.RELEASING
                self._phase_until = time.monotonic() + self._pulse_s
                self._trigger_count = 0
        else:
            self._trigger_count = 0

    def update(self):
        now = time.monotonic()
        if self._state == self.RELEASING and now >= self._phase_until:
            self._motor.force(self._home)
            self._state = self.COOLDOWN
            self._phase_until = now + self._cooldown_s
        elif self._state == self.COOLDOWN and now >= self._phase_until:
            self._state = self.IDLE
        self._motor.update()

    def close(self):
        self._motor.close()


# ══════════════════════════════════════════════════════════════════════
#  SERVO SYSTEM
# ══════════════════════════════════════════════════════════════════════

class ServoSystem:
    INTAKE_UPPER_HOME   = INTAKE_UPPER_HOME
    INTAKE_UPPER_DEPLOY = INTAKE_UPPER_DEPLOY
    INTAKE_LOWER_HOME   = INTAKE_LOWER_HOME
    INTAKE_LOWER_DEPLOY = INTAKE_LOWER_DEPLOY

    def __init__(self,
                 upper_pin: int = 12,
                 lower_pin: int = 13,
                 holder_pin: int = 17,
                 separator_pin: int = 27):
        # intake
        self.upper = ServoMotor(upper_pin)
        self.lower = ServoMotor(lower_pin)
        # separator
        self.holder = HolderServo(holder_pin)
        self.separator = ServoMotor(separator_pin, debounce=SEPARATOR_DEBOUNCE)

    def update(self):
        self.upper.update()
        self.lower.update()
        self.holder.update()
        self.separator.update()

    # ── intake ──
    def intake_upper_home(self):   self.upper.request(INTAKE_UPPER_HOME)
    def intake_upper_deploy(self): self.upper.request(INTAKE_UPPER_DEPLOY)
    def intake_lower_home(self):   self.lower.request(INTAKE_LOWER_HOME)
    def intake_lower_deploy(self): self.lower.request(INTAKE_LOWER_DEPLOY)

    def intake_home(self):
        self.intake_upper_home()
        self.intake_lower_home()

    # ── separator ──
    def separator_mature(self):
        self.separator.request(SEPARATOR_MATURE)

    def separator_immature(self):
        self.separator.request(SEPARATOR_IMMATURE)

    def holder_trigger(self, detected: bool):
        self.holder.trigger(detected)

    def force_home(self):
        self.upper.force(INTAKE_UPPER_HOME)
        self.lower.force(INTAKE_LOWER_HOME)
        self.holder.home()
        self.separator.force(SEPARATOR_MATURE)

    def close(self):
        self.upper.close()
        self.lower.close()
        self.holder.close()
        self.separator.close()