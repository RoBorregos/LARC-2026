#!/usr/bin/env python3
"""
dispatcher.py — Multi-phase vision dispatcher (v5)
Purpose of this code : Own the serial link to the Teensy and, on command,
                       launch/stop the right vision scripts per phase. It merges
                       the VISION bitfields the scripts print and forwards one
                       combined byte to the Teensy. All servo / actuation logic
                       lives on the Teensy; this process only moves data.
Serial  : /dev/ttyTeensy @ 115200  (this process is the ONLY serial owner)
Runs on : the Orin
Ctrl+C  : stop (kills scripts, closes serial)

Protocol
    Scripts print tagged lines to stdout:
        VISION:05    to merge bits, send combined byte to the Teensy
    The vision byte is a bitfield (0x00-0x0F):
        bit 0 = beanTop     bit 1 = beanBottom
        bit 2 = warmBall    bit 3 = coolBall
    Each script owns its own bits (orin_vision: 0-1, separator: 2-3), so the
    dispatcher merges them and one script can't clobber the other's bits.
    Scripts MUST print VISION:00 every frame when nothing is detected, not only
    when something IS detected — otherwise bits stay stuck.
    Box data (benefits phase) uses a header byte: VISION:FE:02

Phases
    BEANS    to orin_vision.py + separator_vision.py
    BENEFITS to benefits.py

Teensy to Orin:
    0xA0 = START BEANS     0xA1 = STOP ALL
    0xA2 = STATUS          0xA3 = START BENEFITS

Orin to Teensy:
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

# - Config
SERIAL_PORT = '/dev/ttyTeensy'
SERIAL_BAUD = 115200

# - Scripts
SCRIPTS = {
    'orin_vision':  '/home/maximo/orin_vision.py',
    'separator':    '/home/maximo/separator_vision.py',
    'benefits':     '/home/maximo/benefits.py',
}

DEBUG_MODE = False
DEBUG_PHASE = 'beans'

WORK_DIR = '/home/maximo'

PHASES = {
    'beans':    ['orin_vision', 'separator'],
    'benefits': ['benefits'],
}

STARTUP_GRACE_SEC   = 5
HEALTH_CHECK_SEC    = 2
RECONNECT_DELAY_SEC = 2

# - Protocol
CMD_START_BEANS    = 0xA0
CMD_STOP           = 0xA1
CMD_STATUS         = 0xA2
CMD_START_BENEFITS = 0xA3

ACK_READY    = 0xB0
ACK_STARTING = 0xB1
ACK_RUNNING  = 0xB2
ACK_STOPPED  = 0xB3
ACK_BENEFITS = 0xB4

ERR_ORIN_VISION = 0xE0
ERR_SEPARATOR   = 0xE1
ERR_BOTH_BEANS  = 0xE2
ERR_CAMERA      = 0xE3
ERR_BENEFITS    = 0xE4
ERR_UNKNOWN     = 0xEF

ECODE_EXIT   = 0x01
ECODE_EXCEPT = 0x02
ECODE_NOCAM  = 0x04

ERR_MAP = {
    'orin_vision': ERR_ORIN_VISION,
    'separator':   ERR_SEPARATOR,
    'benefits':    ERR_BENEFITS,
}

# - State
_procs = {}
_outputs = {}
_phase = None
_lock = threading.Lock()
_ser = None
_ser_lock = threading.Lock()

MAX_OUTPUT_LINES = 100

# Matches vision data lines: VISION:05 or VISION:FE:02
VISION_PATTERN = re.compile(r'^VISION:([0-9A-Fa-f:]+)$')

# - Shared vision state
_vision_state = 0
_vision_lock = threading.Lock()

# Each script owns specific bits — only those bits update when that script sends.
VISION_MASKS = {
    'orin_vision': 0x03,  # bits 0-1 (beanTop, beanBottom)
    'separator':   0x0C,  # bits 2-3 (warmBall, coolBall)
}


def log(msg):
    ts = time.strftime('%H:%M:%S')
    print(f"[{ts}] {msg}", flush=True)


# - Serial
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
    """Merge a script's bits into the shared vision state and send the combined byte."""
    global _vision_state
    try:
        data = bytes(int(b, 16) for b in hex_str.split(':'))

        # Multi-byte (box data etc.) — forward as-is
        if len(data) > 1:
            with _ser_lock:
                if _ser:
                    _ser.write(data)
            return

        # Single byte — merge only this script's bits into the shared state
        mask = VISION_MASKS.get(script_name, 0x0F)
        with _vision_lock:
            _vision_state = (_vision_state & ~mask) | (data[0] & mask)
            combined = _vision_state

        with _ser_lock:
            if _ser:
                _ser.write(bytes([combined]))

    except Exception as e:
        log(f"[Forward] Error: {e}")


# - Output capture (with vision forwarding)
def _capture_output(proc, name):
    """Read a script's stdout, log it, and forward VISION: lines to the Teensy."""
    try:
        for line in proc.stdout:
            text = line.strip()
            if not text:
                continue

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


# - Launch / kill
def kill_all(quiet=False):
    """
    Kill all running vision scripts and reset the shared vision state.

    quiet=True: don't send ACK_STOPPED to the Teensy (internal transitions).
    """
    global _phase, _vision_state

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


def launch_phase(phase_name):
    global _phase

    if _procs:
        log(f"Stopping current phase ({_phase}) before starting {phase_name}")
        kill_all(quiet=True)

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
            log(f"[ERROR] Script not found: {name} to {path}")
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


# - Health check
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

    # Report errors to the Teensy
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


# - Main
def main():
    global _ser

    log("Vision Dispatcher v5 starting...")
    log("Scripts configured:")
    for name, path in SCRIPTS.items():
        exists = "OK" if os.path.exists(path) else "MISSING"
        log(f"  {name}: {path} [{exists}]")
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
                            kill_all()  # Teensy is resetting the round
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
                    kill_all(quiet=True)
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
