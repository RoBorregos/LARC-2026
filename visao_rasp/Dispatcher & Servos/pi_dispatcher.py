#!/usr/bin/env python3
"""
pi_dispatcher.py v5 — Multi-phase vision dispatcher with servo wakeup/sleep
==========================================================================
The dispatcher OWNS the serial port. Vision scripts do NOT open serial.
Scripts print tagged lines to stdout:
    VISION:05    → dispatcher merges bits and sends combined byte to Teensy

Vision data is a single byte bitfield (0x00–0x0F):
    bit 0 = beanTop        bit 1 = beanBottom
    bit 2 = warmBall       bit 3 = coolBall

Each script owns its bits (raspi_visao: 0-1, separator: 2-3).
The dispatcher merges them so one script can't clobber the other's bits.

IMPORTANT: Scripts must print VISION:00 every frame when nothing is detected,
not just VISION:XX when something IS detected. Otherwise bits stay stuck.

Box data (benefits phase) still uses header: VISION:FE:02

SERVO WAKEUP/SLEEP:
  On Teensy connect    → servo_wakeup.py up   (sweep intake servos up)
  On kill_all          → servo_wakeup.py down (sweep intake servos home)
                           covers: CMD_STOP, disconnect, all-scripts-dead,
                           clean shutdown. Skipped on phase-to-phase swaps.
  CMD_STOP and serial disconnect always force the down-sweep, even when
  no vision scripts were running, so a round reset can never leave the
  intake servos deployed (and tossing beans).

The wakeup script is a short-lived subprocess that opens GPIO pins, sweeps,
then releases them — so vision scripts can claim the same pins later without
conflict.

PHASES:
  BEANS    → raspi_visao.py + separator_visao.py
  BENEFITS → benefits.py

Teensy → Pi:
    0xA0 = START BEANS     0xA1 = STOP ALL
    0xA2 = STATUS          0xA3 = START BENEFITS

Pi → Teensy:
    0xB0 = ready   0xB1 = starting   0xB2 = beans running
    0xB3 = stopped 0xB4 = benefits running
    0xE0-0xEF = errors (+ 1 byte error code)
    0x00-0x0F = vision bitfield (single byte, no header)
    0xFE + 1 byte = box type (benefits phase)
"""

import serial as pyserial
import subprocess
import threading
import time
import sys
import os
import re

# ── CONFIG ────────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyTeensy'
SERIAL_BAUD = 115200

# ── VISION SELECTION ──────────────────────────────────────────
# 'original' — runs raspi_visao.py (production, stable)
VISION_VARIANT = 'original' # Had a version before to run different versions, unused now
_VISION_PATHS = {
    'original': '/home/maximo/raspi_visao.py',
    'buffed':   '/home/maximo/buffed_raspi_visao.py',
}
if VISION_VARIANT not in _VISION_PATHS:
    raise ValueError(
        f"VISION_VARIANT must be one of {list(_VISION_PATHS)}, got {VISION_VARIANT!r}"
    )

SCRIPTS = {
    'raspi_visao':  _VISION_PATHS[VISION_VARIANT],
    'separator':    '/home/maximo/separator_visao.py',
    'benefits':     '/home/maximo/benefits.py',
}

SERVO_WAKEUP_SCRIPT = '/home/maximo/servo_wakeup.py'

DEBUG_MODE = False
DEBUG_PHASE = 'beans'

WORK_DIR = '/home/maximo'

PHASES = {
    'beans':    ['raspi_visao', 'separator'],
    'benefits': ['benefits'],
}

STARTUP_GRACE_SEC   = 5
HEALTH_CHECK_SEC    = 2
RECONNECT_DELAY_SEC = 2
SERVO_SWEEP_TIMEOUT = 10
POST_KILL_SETTLE    = 0.2 # let GPIO file descriptors release before sweep

# Scripts that hold the servo GPIO pins (12/13). If one of these was
# running when kill_all is called, we wait POST_KILL_SETTLE before running
# servo_wakeup so gpiozero can release the pins cleanly.
SERVO_GPIO_HOLDERS = {'raspi_visao'}

# ── PROTOCOL ──────────────────────────────────────────────────────────
CMD_START_BEANS    = 0xA0
CMD_STOP           = 0xA1
CMD_STATUS         = 0xA2
CMD_START_BENEFITS = 0xA3

ACK_READY    = 0xB0
ACK_STARTING = 0xB1
ACK_RUNNING  = 0xB2
ACK_STOPPED  = 0xB3
ACK_BENEFITS = 0xB4

ERR_RASPI_VISAO = 0xE0
ERR_SEPARATOR   = 0xE1
ERR_BOTH_BEANS  = 0xE2
ERR_CAMERA      = 0xE3
ERR_BENEFITS    = 0xE4
ERR_UNKNOWN     = 0xEF

ECODE_EXIT   = 0x01
ECODE_EXCEPT = 0x02
ECODE_NOCAM  = 0x04

ERR_MAP = {
    'raspi_visao': ERR_RASPI_VISAO,
    'separator':   ERR_SEPARATOR,
    'benefits':    ERR_BENEFITS,
}

# ── STATE ─────────────────────────────────────────────────────────────
_procs = {}
_outputs = {}
_phase = None
_lock = threading.Lock()
_ser = None
_ser_lock = threading.Lock()

MAX_OUTPUT_LINES = 100

# Pattern to detect vision data lines: VISION:05 or VISION:FE:02
VISION_PATTERN = re.compile(r'^VISION:([0-9A-Fa-f:]+)$')

# ── SHARED VISION STATE ──────────────────────────────────────────────
_vision_state = 0
_vision_lock = threading.Lock()

# Each script owns specific bits — only those bits get updated when that script sends
VISION_MASKS = {
    'raspi_visao': 0x03,  # bits 0-1 (beanTop, beanBottom)
    'separator':   0x0C,  # bits 2-3 (warmBall, coolBall)
}


def log(msg):
    ts = time.strftime('%H:%M:%S')
    print(f"[{ts}] {msg}", flush=True)


# ── SERVO WAKEUP ──────────────────────────────────────────────────────
def run_servo_sweep(direction):
    """
    Blocking subprocess call to servo_wakeup.py.
    direction: 'up' (home → deploy) or 'down' (deploy → home).
    Runs synchronously so pins are fully released before returning.
    """
    if not os.path.exists(SERVO_WAKEUP_SCRIPT):
        log(f"[WARN] Servo wakeup script not found: {SERVO_WAKEUP_SCRIPT}")
        return

    log(f"Running servo sweep: {direction}")
    try:
        subprocess.run(
            ['python3', '-u', SERVO_WAKEUP_SCRIPT, direction],
            cwd=WORK_DIR,
            timeout=SERVO_SWEEP_TIMEOUT,
            check=True,
        )
        log(f"Sweep {direction} complete, pins released")
    except subprocess.TimeoutExpired:
        log(f"[WARN] Sweep {direction} timed out")
    except subprocess.CalledProcessError as e:
        log(f"[WARN] Sweep {direction} failed (rc={e.returncode})")
    except Exception as e:
        log(f"[WARN] Sweep {direction} failed: {e}")


# ── SERIAL ────────────────────────────────────────────────────────────
def open_serial():
    global _ser
    while True:
        try:
            s = pyserial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            time.sleep(0.5)
            s.reset_input_buffer()
            s.reset_output_buffer()
            with _ser_lock:
                _ser = s
            log(f"Serial opened: {SERIAL_PORT} @ {SERIAL_BAUD}")
            return s
        except Exception as e:
            log(f"Serial open failed: {e} -- retrying in {RECONNECT_DELAY_SEC}s...")
            time.sleep(RECONNECT_DELAY_SEC)


def send(*data_bytes):
    """Thread-safe serial send."""
    with _ser_lock:
        if _ser is None:
            return
        try:
            _ser.write(bytes(data_bytes))
        except Exception as e:
            log(f"Serial send error: {e}")


def serial_read(n=1):
    """Thread-safe serial read. Re-raises on disconnect for main() to handle."""
    with _ser_lock:
        if _ser is None:
            return b''
        try:
            return _ser.read(n)
        except (pyserial.SerialException, OSError):
            raise


def forward_vision_bytes(hex_str, script_name):
    """Merge script bits into shared vision state and send combined byte."""
    global _vision_state
    try:
        data = bytes(int(b, 16) for b in hex_str.split(':'))

        # Multi-byte (box data etc) — forward as-is
        if len(data) > 1:
            with _ser_lock:
                if _ser:
                    _ser.write(data)
            return

        # Single byte — merge only this script's bits into shared state
        mask = VISION_MASKS.get(script_name, 0x0F)
        with _vision_lock:
            _vision_state = (_vision_state & ~mask) | (data[0] & mask)
            combined = _vision_state

        with _ser_lock:
            if _ser:
                _ser.write(bytes([combined]))

    except Exception as e:
        log(f"[Forward] Error: {e}")


# ── OUTPUT CAPTURE (with vision forwarding) ───────────────────────────
def _capture_output(proc, name):
    """Read stdout, log it, and forward VISION: lines to Teensy."""
    try:
        for line in proc.stdout:
            text = line.strip()
            if not text:
                continue

            # Check for vision data tag
            m = VISION_PATTERN.match(text)
            if m:
                forward_vision_bytes(m.group(1), name)
                continue  # don't log every vision frame

            # Regular output — store and log
            with _lock:
                if name not in _outputs:
                    _outputs[name] = []
                _outputs[name].append(text)
                if len(_outputs[name]) > MAX_OUTPUT_LINES:
                    _outputs[name] = _outputs[name][-MAX_OUTPUT_LINES:]
            log(f"[{name}] {text}")
    except Exception:
        pass


def get_last_output(name, n=10):
    with _lock:
        return _outputs.get(name, [])[-n:]


# ── LAUNCH / KILL ────────────────────────────────────────────────────
def kill_all(quiet=False, sweep_home=True, force_sweep=False):
    """
    Kill all running vision scripts and (by default) sweep servos home.

    sweep_home=True: after killing, wait for GPIO pins to release and run
      servo_wakeup.py down. This fires on every 'vision off' event coming
      from the Teensy side — CMD_STOP, disconnect, shutdown, all-dead —
      so the intake servos always go home and don't drop balls.
    sweep_home=False: used internally by launch_phase when swapping from
      one vision phase to another, since the next phase will reclaim the
      pins immediately and we don't want to drop+raise+redeploy.
    force_sweep=True: sweep home even if no procs were running. Used on
      'reset the round' events (CMD_STOP, serial disconnect) where the
      previous round may have left servos deployed and we cannot tolerate
      tossing beans during the reset.
    quiet=True: don't send ACK_STOPPED to Teensy (internal transitions).

    A no-op sweep when nothing was running is skipped — servos are
    already home in that case — unless force_sweep overrides that.
    """
    global _phase, _vision_state

    # Remember whether any servo-holding script was running — if so we
    # need the settle delay before running servo_wakeup so gpiozero can
    # release pins 12/13 cleanly.
    had_servo_holder = any(
        name in SERVO_GPIO_HOLDERS and proc and proc.poll() is None
        for name, proc in _procs.items()
    )
    had_any_proc = bool(_procs)

    for name, proc in list(_procs.items()):
        if proc and proc.poll() is None:
            log(f"  Killing {name} PID={proc.pid}")
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
            log(f"  {name} stopped (rc={proc.returncode})")
    _procs.clear()
    _phase = None
    _vision_state = 0

    if not quiet:
        send(ACK_STOPPED)
        log("All scripts stopped")

    if sweep_home and (had_any_proc or force_sweep):
        if had_servo_holder:
            time.sleep(POST_KILL_SETTLE)  # let gpiozero release pins 12/13
        run_servo_sweep('down')


def launch_phase(phase_name):
    global _phase

    if _procs:
        log(f"Stopping current phase ({_phase}) before starting {phase_name}")
        # Don't sweep home between phases — scripts of the next phase will
        # reclaim the servos immediately.
        kill_all(quiet=True, sweep_home=False)

    script_names = PHASES.get(phase_name)
    if not script_names:
        log(f"[ERROR] Unknown phase: {phase_name}")
        send(ERR_UNKNOWN, ECODE_EXCEPT)
        return

    send(ACK_STARTING)
    log(f"Launching phase: {phase_name} -- scripts: {script_names}")

    for name in script_names:
        path = SCRIPTS.get(name)
        if not path or not os.path.exists(path):
            log(f"[ERROR] Script not found: {name} -> {path}")
            send(ERR_MAP.get(name, ERR_UNKNOWN), ECODE_EXCEPT)
            return

    for name in script_names:
        path = SCRIPTS[name]
        try:
            proc = subprocess.Popen(
                ['python3', '-u', path],
                cwd=WORK_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            _procs[name] = proc
            _outputs[name] = []
            t = threading.Thread(target=_capture_output, args=(proc, name), daemon=True)
            t.start()
            log(f"  {name} PID={proc.pid} ({path})")
        except Exception as e:
            log(f"[ERROR] Failed to launch {name}: {e}")
            send(ERR_MAP.get(name, ERR_UNKNOWN), ECODE_EXCEPT)
            kill_all(quiet=True)
            return

    log(f"Waiting {STARTUP_GRACE_SEC}s for startup...")
    time.sleep(STARTUP_GRACE_SEC)

    failed = []
    for name in script_names:
        proc = _procs.get(name)
        if proc and proc.poll() is not None:
            rc = proc.returncode
            log(f"[ERROR] {name} exited during startup (rc={rc})")
            last = ' '.join(get_last_output(name, 5)).lower()
            if 'cannot open camera' in last or 'cannot open /dev/video' in last:
                send(ERR_MAP.get(name, ERR_UNKNOWN), ECODE_NOCAM)
            else:
                send(ERR_MAP.get(name, ERR_UNKNOWN), ECODE_EXIT)
            failed.append(name)

    if failed:
        for name in failed:
            for line in get_last_output(name, 10):
                log(f"    | {line}")
            if name in _procs:
                del _procs[name]

    alive = [n for n, p in _procs.items() if p and p.poll() is None]
    if alive:
        _phase = phase_name
        ack = ACK_BENEFITS if phase_name == 'benefits' else ACK_RUNNING
        send(ack)
        log(f"Phase {phase_name}: running with {alive} (failed: {failed or 'none'})")
    else:
        log(f"Phase {phase_name}: ALL scripts failed — nothing running")
        kill_all(quiet=True)


# ── HEALTH CHECK ─────────────────────────────────────────────────────
def check_health():
    if not _phase:
        return

    crashed = []
    for name, proc in list(_procs.items()):
        if proc and proc.poll() is not None:
            rc = proc.returncode
            log(f"[HEALTH] {name} exited unexpectedly (rc={rc})")
            for line in get_last_output(name, 10):
                log(f"    | {line}")
            crashed.append(name)

    if not crashed:
        return

    # Report errors to Teensy
    if _phase == 'beans':
        for name in crashed:
            send(ERR_MAP.get(name, ERR_UNKNOWN), ECODE_EXIT)
    elif _phase == 'benefits':
        send(ERR_BENEFITS, ECODE_EXIT)

    # Remove dead processes but keep alive ones running
    for name in crashed:
        if name in _procs:
            del _procs[name]

    # Only kill everything if ALL scripts are dead
    alive = [n for n, p in _procs.items() if p and p.poll() is None]
    if not alive:
        log("[HEALTH] All scripts dead — phase over")
        kill_all(quiet=True)
    else:
        log(f"[HEALTH] Still running: {alive}")


def report_status():
    log(f"== STATUS REPORT ==")
    log(f"  Phase: {_phase or 'IDLE'}")

    if not _phase:
        send(ACK_STOPPED)
        log(f"  No scripts running")
        import glob
        videos = sorted(glob.glob('/dev/video*'))
        log(f"  Video devices: {videos}")
        try:
            result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=3)
            for line in result.stdout.splitlines():
                log(f"  USB: {line}")
        except Exception:
            pass
        log(f"== END STATUS ==")
        return

    all_ok = True
    for name, proc in _procs.items():
        alive = proc and proc.poll() is None
        status = "RUNNING" if alive else f"DEAD (rc={proc.returncode if proc else '?'})"
        pid = proc.pid if proc else "?"
        log(f"  {name}: {status} (PID={pid})")
        for line in get_last_output(name, 3):
            log(f"    | {line}")
        if not alive:
            all_ok = False

    if all_ok:
        ack = ACK_BENEFITS if _phase == 'benefits' else ACK_RUNNING
        send(ack)
    else:
        send(ERR_UNKNOWN, ECODE_EXIT)
    log(f"== END STATUS ==")


# ── MAIN ──────────────────────────────────────────────────────────────
def main():
    global _ser

    log(f"Pi Dispatcher v5 starting... (vision variant: {VISION_VARIANT})")
    log("Scripts configured:")
    for name, path in SCRIPTS.items():
        exists = "OK" if os.path.exists(path) else "MISSING"
        log(f"  {name}: {path} [{exists}]")
    wakeup_ok = "OK" if os.path.exists(SERVO_WAKEUP_SCRIPT) else "MISSING"
    log(f"  servo_wakeup: {SERVO_WAKEUP_SCRIPT} [{wakeup_ok}]")
    log("Phases:")
    for phase, scripts in PHASES.items():
        log(f"  {phase}: {scripts}")

    ser = None

    try:
        if DEBUG_MODE:
            log(f"=== DEBUG MODE — no Teensy, auto-starting '{DEBUG_PHASE}' ===")
            launch_phase(DEBUG_PHASE)
            while True:
                time.sleep(HEALTH_CHECK_SEC)
                check_health()
        else:
            while True:
                if ser is None:
                    ser = open_serial()         # blocks until connected
                    run_servo_sweep('up')       # wake servos on connect
                    send(ACK_READY)
                    log("Dispatcher ready -- waiting for Teensy commands")
                    last_health = time.time()

                try:
                    data = serial_read(1)
                    if data:
                        cmd = data[0]
                        if cmd == CMD_START_BEANS:
                            log(">> CMD: START BEANS")
                            launch_phase('beans')
                        elif cmd == CMD_STOP:
                            log(">> CMD: STOP")
                            kill_all(force_sweep=True)  # always sweep — Teensy is resetting the round
                        elif cmd == CMD_STATUS:
                            log(">> CMD: STATUS")
                            report_status()
                        elif cmd == CMD_START_BENEFITS:
                            log(">> CMD: START BENEFITS")
                            launch_phase('benefits')
                        else:
                            log(f">> Unknown byte: 0x{cmd:02X}")

                    now = time.time()
                    if now - last_health >= HEALTH_CHECK_SEC:
                        check_health()
                        last_health = now

                except (pyserial.SerialException, OSError) as e:
                    log(f"[Serial] Disconnected: {e}")
                    log("[Serial] Killing scripts and reconnecting...")
                    kill_all(quiet=True, force_sweep=True)  # always sweep — Teensy is resetting
                    with _ser_lock:
                        try:
                            if _ser:
                                _ser.close()
                        except Exception:
                            pass
                        _ser = None
                    ser = None
                    time.sleep(RECONNECT_DELAY_SEC)

    except KeyboardInterrupt:
        log("Dispatcher shutting down...")
    finally:
        # kill_all handles the 'down' sweep internally. No-op if nothing
        # was running, which is exactly what we want.
        kill_all(quiet=True)
        with _ser_lock:
            if _ser:
                try:
                    _ser.close()
                except Exception:
                    pass
                _ser = None

        log("Dispatcher exited.")


if __name__ == "__main__":
    main()