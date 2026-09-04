#!/usr/bin/env python3
"""
dispatcher.py — the Orin's one long running process (protocol v2)

Purpose : Own the serial link to the Teensy, start and stop the vision
          scripts for the phase the Teensy asks for, turn what they print
          into protocol-v2 command frames, and keep that stream alive so the
          Teensy's watchdog stays happy. No servo logic lives here.
Link    : link/teensy_link.py (the only serial owner)
Runs on : the Orin, normally as a systemd service
Logs    : stdout -> journald.  journalctl -u larc-dispatcher -f
Stop    : Ctrl+C or systemctl stop — both park the robot in IDLE first

Script contract
    intake     VISION:XX        bit 0 = intake upper, bit 1 = intake lower
    separator  VISION:FD:WW:CC  WW = warm hit, CC = cool hit (00 or 01)
    benefits   VISION:FE:XX     00 none, 01 red, 02 blue
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

# vision_orin/link is a sibling of this directory.
REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "link"))

import vision_protocol as vp
from teensy_link import TeensyLink, find_teensy

SCRIPTS = {
    "intake":    Path(os.environ.get("LARC_INTAKE_PY",
                                     REPO / "main_vision" / "orin_vision.py")),
    "separator": Path(os.environ.get("LARC_SEPARATOR_PY",
                                     REPO / "separator" / "separator_vision.py")),
    "benefits":  Path(os.environ.get("LARC_BENEFITS_PY",
                                     REPO / "benefits" / "benefits.py")),
}

PHASES = {
    "beans":    ["intake", "separator"],
    "benefits": ["benefits"],
}

SERIAL_DEVICE = os.environ.get("LARC_TEENSY_DEV") or None

AUTOSTART_PHASE = os.environ.get("LARC_AUTOSTART_PHASE", "").strip().lower() or None

# Tuning
LOOP_HZ = 100 # how often we recompute the command
STARTUP_GRACE_SEC = 5.0 # how long a script gets to prove it survived
HEALTH_CHECK_SEC = 2.0
LINK_RETRY_SEC = 2.0

# A source that has not printed for this long stops counting.
STALE_MS = {"intake": 500, "separator": 500, "benefits": 800}

# How long the separator holds a side after the last hit for that side.
SEPARATOR_HOLD_MS = 250

# How long we assert "open" on a benefit door. The Teensy runs its own
# open window (kBenefitOpenMs); this only has to be long enough for
# REQUIRED_CONFIRMATION_FRAMES frames to carry the bit.
BENEFIT_PULSE_MS = 120

# Semantic mapping — the only place these meanings are written down
INTAKE_UPPER_BIT = 0x01
INTAKE_LOWER_BIT = 0x02

WARM_IS = vp.SEP_LEFT # warm ball - separator LEFT  (mature)
COOL_IS = vp.SEP_RIGHT # cool ball - separator RIGHT (overmature)

BOX_NONE, BOX_RED, BOX_BLUE = 0, 1, 2
BOX_DOOR = {BOX_RED: 0, BOX_BLUE: 1}   # box type - which door opens
BOX_NAMES = {BOX_NONE: "NONE", BOX_RED: "RED", BOX_BLUE: "BLUE"}

FAULT_BIT = {
    "intake":    vp.STATUS_MAIN_FAULT, # critical on the Teensy
    "separator": vp.STATUS_SEPARATOR_FAULT,
    "benefits":  vp.STATUS_BENEFITS_FAULT,
}

VISION_LINE = re.compile(r"^VISION:([0-9A-Fa-f:]+)\s*$")
MAX_KEPT_LINES = 40


def log(message: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)


def now_ms() -> int:
    return int(time.monotonic() * 1000)


# What the vision scripts are currently telling us
class VisionState:

    def __init__(self) -> None:
        self._lock = threading.Lock()

        self.intake_upper = False
        self.intake_lower = False
        self.intake_ms = 0

        self.sep_side = vp.SEP_NEUTRAL
        self.sep_until_ms = 0
        self.separator_ms = 0

        self.box = BOX_NONE
        self.box_ms = 0
        self.door_pulse_until = [0, 0]
        self.door_armed = [True, True] # mirrors the Teensy's rearm rule

        self.stale = set() # sources that have gone quiet

    def feed(self, source: str, payload: str) -> bool:
        try:
            fields = [int(part, 16) for part in payload.split(":")]
        except ValueError:
            return False
        if not fields:
            return False

        stamp = now_ms()
        with self._lock:
            if source == "intake" and len(fields) == 1:
                self.intake_upper = bool(fields[0] & INTAKE_UPPER_BIT)
                self.intake_lower = bool(fields[0] & INTAKE_LOWER_BIT)
                self.intake_ms = stamp
                return True

            if source == "separator" and len(fields) == 3 and fields[0] == 0xFD:
                warm, cool = bool(fields[1]), bool(fields[2])
                if warm != cool:
                    self.sep_side = WARM_IS if warm else COOL_IS
                    self.sep_until_ms = stamp + SEPARATOR_HOLD_MS
                self.separator_ms = stamp
                return True

            if source == "benefits" and len(fields) == 2 and fields[0] == 0xFE:
                box = fields[1]
                self.box_ms = stamp
                if box != self.box:
                    self.box = box
                    if box == BOX_NONE:
                        self.door_armed = [True, True]
                    else:
                        door = BOX_DOOR.get(box)
                        if door is not None and self.door_armed[door]:
                            self.door_pulse_until[door] = stamp + BENEFIT_PULSE_MS
                            self.door_armed[door] = False
                return True

        return False

    def note_launched(self, sources) -> None:
        stamp = now_ms()
        with self._lock:
            for source in sources:
                offset = int(STARTUP_GRACE_SEC * 1000)
                if source == "intake":
                    self.intake_ms = stamp + offset
                elif source == "separator":
                    self.separator_ms = stamp + offset
                elif source == "benefits":
                    self.box_ms = stamp + offset
            self.stale -= set(sources)

    def reset(self) -> None:
        with self._lock:
            self.intake_upper = self.intake_lower = False
            self.sep_side = vp.SEP_NEUTRAL
            self.sep_until_ms = 0
            self.box = BOX_NONE
            self.door_pulse_until = [0, 0]
            self.door_armed = [True, True]
            self.stale.clear()

    # called from the main loop
    def beans_command(self):
        stamp = now_ms()
        with self._lock:
            stale = set()

            if stamp - self.intake_ms > STALE_MS["intake"]:
                stale.add("intake")
                upper = lower = False
            else:
                upper, lower = self.intake_upper, self.intake_lower

            if stamp - self.separator_ms > STALE_MS["separator"]:
                stale.add("separator")
                separator = vp.SEP_NEUTRAL
            else:
                separator = (self.sep_side if stamp < self.sep_until_ms
                             else vp.SEP_NEUTRAL)

            self.stale = stale
            return upper, lower, separator, stale

    def benefits_command(self):
        stamp = now_ms()
        with self._lock:
            stale = set()
            if stamp - self.box_ms > STALE_MS["benefits"]:
                stale.add("benefits")
                self.stale = stale
                return False, False, stale

            doors = [stamp < self.door_pulse_until[0],
                     stamp < self.door_pulse_until[1]]
            self.stale = stale
            return doors[0], doors[1], stale


# Child processes
class ScriptRunner:

    def __init__(self, state: VisionState) -> None:
        self.state = state
        self.procs = {}
        self.tails = {}
        self.phase = None
        self._lock = threading.Lock()

    def _reader(self, proc, name: str) -> None:
        for raw in proc.stdout:
            text = raw.rstrip()
            if not text:
                continue
            match = VISION_LINE.match(text)
            if match:
                if not self.state.feed(name, match.group(1)):
                    log(f"[{name}] unparsable vision line: {text}")
                continue
            with self._lock:
                tail = self.tails.setdefault(name, [])
                tail.append(text)
                del tail[:-MAX_KEPT_LINES]
            log(f"[{name}] {text}")

    def tail(self, name: str, count: int = 8):
        with self._lock:
            return list(self.tails.get(name, []))[-count:]

    def start(self, phase: str) -> bool:
        self.stop(quiet=True)

        names = PHASES.get(phase)
        if not names:
            log(f"[ERROR] unknown phase '{phase}'")
            return False

        missing = [n for n in names if not SCRIPTS[n].exists()]
        if missing:
            for name in missing:
                log(f"[ERROR] script missing: {name} -> {SCRIPTS[name]}")
            return False

        log(f"launching phase '{phase}': {names}")
        for name in names:
            path = SCRIPTS[name]
            try:
                proc = subprocess.Popen(
                    [sys.executable, "-u", str(path)],
                    cwd=str(path.parent), # so it finds its *_config.json
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
            except Exception as exc:
                log(f"[ERROR] could not launch {name}: {exc}")
                self.stop(quiet=True)
                return False

            self.procs[name] = proc
            self.tails[name] = []
            threading.Thread(target=self._reader, args=(proc, name),
                             name=f"read-{name}", daemon=True).start()
            log(f"  {name} pid={proc.pid}  ({path})")

        self.state.note_launched(names)
        self.phase = phase
        return True

    def stop(self, quiet: bool = False) -> None:
        for name, proc in list(self.procs.items()):
            if proc and proc.poll() is None:
                if not quiet:
                    log(f"  stopping {name} pid={proc.pid}")
                try:
                    proc.terminate()
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        self.procs.clear()
        self.phase = None
        self.state.reset()

    def alive(self):
        return [n for n, p in self.procs.items() if p and p.poll() is None]

    def reap(self):
        dead = []
        for name, proc in list(self.procs.items()):
            if proc and proc.poll() is not None:
                log(f"[HEALTH] {name} exited (rc={proc.returncode})")
                for line in self.tail(name, 10):
                    log(f"    | {line}")
                dead.append(name)
                del self.procs[name]
        return dead


# Main
class Dispatcher:
    def __init__(self, args) -> None:
        self.args = args
        self.state = VisionState()
        self.runner = ScriptRunner(self.state)
        self.link = None
        self.running = True
        self._last_reported = None
        self._stale_reported = set()
        self._warned_missing = False

    def stop(self, *_signal) -> None:
        self.running = False

    # -- link
    def connect(self) -> None:
        while self.running and self.link is None:
            try:
                self.link = TeensyLink(device=SERIAL_DEVICE)
                self._warned_missing = False
                log(f"serial open: {self.link.device} @ {vp.BAUD}")
                self.link.set_ready(True)
                log("dispatcher ready — waiting for the Teensy to ask for a phase")
            except Exception as exc:
                if not self._warned_missing:
                    log(f"serial not available ({exc})")
                    log(f"waiting for a Teensy — retrying every {LINK_RETRY_SEC}s, "
                        f"quietly from here")
                    self._warned_missing = True
                time.sleep(LINK_RETRY_SEC)

    def drop_link(self, reason: str) -> None:
        log(f"[serial] lost: {reason}")
        self.runner.stop(quiet=True)
        try:
            if self.link:
                self.link.close()
        except Exception:
            pass
        self.link = None

    def handle_messages(self) -> None:
        for line in self.link.take_messages():
            log(f"[teensy] {line}")

    # -- requests from the Teensy
    def handle_requests(self) -> None:
        for request in self.link.take_requests():
            name = vp.CMD_NAMES.get(request, f"0x{request:02X}")
            log(f">> Teensy requests {name}")
            if request == vp.CMD_START_BEANS:
                self.enter("beans")
            elif request == vp.CMD_START_BENEFITS:
                self.enter("benefits")
            elif request == vp.CMD_STOP:
                self.enter(None)
            elif request == vp.CMD_STATUS:
                self.report_status()

    def enter(self, phase) -> None:
        if phase == self.runner.phase:
            log(f"already in phase '{phase}' — nothing to do")
            return

        # Park the link before the scripts change under it.
        self.link.set_idle()
        self.link.set_beans_running(False)
        self.link.set_benefits_running(False)

        if phase is None:
            self.runner.stop()
            log("phase -> IDLE")
            return

        if not self.runner.start(phase):
            self.link.set_fault(FAULT_BIT.get(phase, vp.STATUS_MAIN_FAULT), True)
            return

        if phase == "beans":
            self.link.set_phase_beans()
            self.link.set_beans_running(True)
        else:
            self.link.set_phase_benefits()
            self.link.set_benefits_running(True)
        log(f"phase -> {phase.upper()}")

    # pushing the command
    def push_command(self) -> None:
        phase = self.runner.phase
        if phase == "beans":
            upper, lower, separator, stale = self.state.beans_command()
            self.link.set_beans(upper, lower, separator)
            summary = (f"BEANS upper={int(upper)} lower={int(lower)} "
                       f"sep={vp.SEP_NAMES[separator]}")
        elif phase == "benefits":
            door1, door2, stale = self.state.benefits_command()
            self.link.set_benefits(door1, door2)
            summary = f"BENEFITS door1={int(door1)} door2={int(door2)}"
        else:
            return

        for source in stale - self._stale_reported:
            log(f"[STALE] {source} stopped reporting — its outputs are now safe")
            self.link.set_fault(FAULT_BIT[source], True)
        for source in self._stale_reported - stale:
            log(f"[STALE] {source} is reporting again")
            self.link.set_fault(FAULT_BIT[source], False)
        self._stale_reported = set(stale)

        if summary != self._last_reported:
            log(summary)
            self._last_reported = summary

    # health
    def check_health(self) -> None:
        if not self.runner.phase:
            return
        dead = self.runner.reap()
        if not dead:
            return
        for name in dead:
            self.link.set_fault(FAULT_BIT[name], True)
        if not self.runner.alive():
            log("[HEALTH] every script for this phase is dead — going IDLE")
            self.enter(None)

    def report_status(self) -> None:
        log("== STATUS ==")
        log(f"  phase        {self.runner.phase or 'IDLE'}")
        log(f"  serial       {self.link.device}")
        log(f"  status byte  0x{self.link.status:02X}")
        if self.runner.phase:
            for name in PHASES[self.runner.phase]:
                proc = self.runner.procs.get(name)
                alive = proc and proc.poll() is None
                log(f"  {name:<10}  {'RUNNING' if alive else 'DEAD'}"
                    f"  pid={proc.pid if proc else '-'}")
                for line in self.runner.tail(name, 3):
                    log(f"      | {line}")
        else:
            import glob
            log(f"  video nodes  {sorted(glob.glob('/dev/video*'))}")
        log("== END STATUS ==")

    # -- run --
    def run(self) -> int:
        log("LARC vision dispatcher (protocol v2) starting")
        log(f"  repo         {REPO}")
        for name, path in SCRIPTS.items():
            log(f"  {name:<10}  {path}  [{'OK' if path.exists() else 'MISSING'}]")
        log(f"  teensy       {SERIAL_DEVICE or find_teensy() or 'not found yet'}")

        signal.signal(signal.SIGTERM, self.stop)
        signal.signal(signal.SIGINT, self.stop)

        period = 1.0 / LOOP_HZ
        last_health = time.monotonic()

        try:
            while self.running:
                if self.link is None:
                    self.connect()
                    if self.link is None:
                        break
                    phase = self.args.phase or AUTOSTART_PHASE
                    if phase:
                        source = "--phase" if self.args.phase else "LARC_AUTOSTART_PHASE"
                        log(f"{source}={phase}: starting it now without waiting "
                            f"for the Teensy to ask")
                        self.enter(phase)
                if not self.link.healthy:
                    self.drop_link(str(self.link.error or "transmit thread stopped"))
                    time.sleep(LINK_RETRY_SEC)
                    continue

                try:
                    self.handle_messages()
                    self.handle_requests()
                    self.push_command()

                    now = time.monotonic()
                    if now - last_health >= HEALTH_CHECK_SEC:
                        self.check_health()
                        last_health = now
                except OSError as exc:
                    self.drop_link(str(exc))
                    time.sleep(LINK_RETRY_SEC)
                    continue

                time.sleep(period)

        finally:
            log("shutting down — parking the robot")
            self.runner.stop(quiet=True)
            if self.link:
                try:
                    self.link.set_idle()
                    self.link.close() # sends IDLE frames, then closes
                except Exception:
                    pass
            log("dispatcher exited")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="LARC Orin vision dispatcher (protocol v2).")
    parser.add_argument("--phase", choices=sorted(PHASES),
                        help="start this phase immediately instead of waiting "
                             "for the Teensy to ask (bench testing)")
    return Dispatcher(parser.parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
