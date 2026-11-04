#!/usr/bin/env python3
"""Fake test bench board for trying the scripts without hardware (macOS/Linux).

It speaks the same serial protocol as firmware/testbench (0.2.1), including
timed commands, the actuator travel estimate and limit, and MARK. The numbers
are made up but plausible: thrust rises with pump PWM, and an1/an2 act like
actuator position sensors that follow the actuators.

  python datalog/sim_device.py                 # prints a port such as /dev/ttys012
  python datalog/log_data.py --port /dev/ttys012
"""

import argparse
import fcntl
import os
import pty
import random
import select
import time
import tty

VERSION = "0.2.1"
HEADER = "H,t_ms,mode,pump,act_a,act_b,pos_a,pos_b,force_g,an1,an2,pot,joy,estop,mark"
MAX_DURATION_MS = 600000
MARK_MS = 200


class Timed:
    def __init__(self):
        self.cmd = 0
        self.start = 0
        self.dur = 0

    def set(self, cmd, dur, now):
        self.cmd, self.start, self.dur = cmd, now, (dur if cmd else 0)

    def expired(self, now):
        if not self.dur or now - self.start < self.dur:
            return False
        self.cmd = self.dur = 0
        return True


class FakeBoard:
    def __init__(self, limit_ms=2000, hx711=True):
        self.t0 = time.time()
        self.mode = "S"
        self.pump = Timed()
        self.act = {"A": Timed(), "B": Timed()}
        self.travel = {"A": 0.0, "B": 0.0}  # duty x ms, like the firmware
        self.limit_hit = {"A": False, "B": False}
        self.limit_ms = limit_ms
        self.sensor = {"A": 512.0, "B": 512.0}  # fake position pots on an1/an2
        self.hx711 = hx711
        self.force_raw = 0.0  # grams before tare
        self.offset = 0.0
        self.scale = 455.0
        self.rate = 20
        self.timeout = 0
        self.last_cmd = 0
        self.mark_start = None
        self.stream_now = False
        self.last_ms = self.millis()

    def millis(self):
        return int((time.time() - self.t0) * 1000)

    def flow_ml_min(self):
        duty = abs(self.pump.cmd)
        return 0.0 if duty < 50 else (duty - 50) * 2.4

    def past_limit(self, name, cmd):
        limit = self.limit_ms * 255
        t = self.travel[name]
        return bool(limit) and ((cmd > 0 and t >= limit) or (cmd < 0 and t <= -limit))

    def step(self):
        """Advance the model; returns status lines to print."""
        now = self.millis()
        dt = now - self.last_ms
        self.last_ms = now
        out = []
        for name, t in self.act.items():
            self.travel[name] += t.cmd * dt
            # Under load the real actuator is slower than the estimate.
            self.sensor[name] = min(1023.0, max(0.0, self.sensor[name] + t.cmd / 255.0 * 0.12 * dt))
        target = 1.2e-5 * self.flow_ml_min() ** 2  # grams of thrust
        self.force_raw += (target - self.force_raw) * min(1.0, dt / 500.0)

        if self.pump.expired(now):
            out.append("# done PUMP")
        for name, t in self.act.items():
            if t.expired(now):
                out.append(f"# done ACT {name}")
        busy = self.pump.cmd or any(t.cmd for t in self.act.values())
        if self.timeout and now - self.last_cmd > self.timeout and busy:
            self.stop_all()
            out.append("# command timeout: all outputs off")
        for name, t in self.act.items():
            if self.past_limit(name, t.cmd):
                t.set(0, 0, now)
                if not self.limit_hit[name]:
                    self.limit_hit[name] = True
                    out.append(f"# act {name} reached its travel limit and stopped; "
                               "it can only move back (or send ZERO / LIMIT)")
            elif t.cmd:
                self.limit_hit[name] = False
        if self.mark_start is not None and now - self.mark_start >= MARK_MS:
            self.mark_start = None
        return out

    def stop_all(self):
        now = self.millis()
        self.pump.set(0, 0, now)
        for t in self.act.values():
            t.set(0, 0, now)

    def telemetry(self):
        force = f"{self.force_raw - self.offset + random.gauss(0, 0.05):.2f}" if self.hx711 else ""
        a, b = self.act["A"].cmd, self.act["B"].cmd
        pos_a, pos_b = int(self.travel["A"] / 255), int(self.travel["B"] / 255)
        return (f"D,{self.millis()},{self.mode},{self.pump.cmd},{a},{b},{pos_a},{pos_b},{force},"
                f"{int(self.sensor['A'])},{int(self.sensor['B'])},0,512,0,"
                f"{int(self.mark_start is not None)}")

    def info(self):
        hx = "ok" if self.hx711 else "missing"
        return [f"I,fams-sr-testbench-sim,{VERSION},scale={self.scale:.3f},hx711={hx},"
                f"limit_ms={self.limit_ms},driver=en_in", HEADER]

    def handle(self, line):
        parts = line.split()
        if not parts:
            return []
        cmd, args = parts[0].upper(), parts[1:]
        now = self.millis()
        self.last_cmd = now

        def num(i, lo, hi, default=None):
            if i >= len(args):
                return default
            try:
                v = int(args[i])
            except ValueError:
                return None
            return v if lo <= v <= hi else None

        def duration(i):
            return num(i, 1, MAX_DURATION_MS, default=0)

        def blocked(name, v):
            if self.past_limit(name, v):
                return (f"ERR act {name} is at its travel limit; it can only move back "
                        "(or send ZERO / LIMIT)")
            return None

        if cmd == "PING":
            return ["OK PONG"]
        if cmd == "INFO":
            return self.info()
        if cmd == "HELP":
            return ["# see firmware/testbench/README.md"]
        if cmd == "MODE" and args and args[0].upper() in ("SERIAL", "MANUAL"):
            self.mode = "S" if args[0].upper() == "SERIAL" else "M"
            self.stop_all()
            return [f"OK MODE {args[0].upper()}"]
        if cmd in ("PUMP", "ACT", "STEER") and self.mode != "S":
            return ["ERR in manual mode; send MODE SERIAL first"]
        if cmd == "PUMP":
            v, d = num(0, -255, 255), duration(1)
            if v is None or d is None:
                return ["ERR usage: PUMP <-255..255> [ms]"]
            self.pump.set(v, d, now)
            return ["OK PUMP"]
        if cmd == "ACT":
            v, d = num(1, -255, 255), duration(2)
            if v is None or d is None or not args or args[0].upper() not in ("A", "B"):
                return ["ERR usage: ACT A|B <-255..255> [ms]"]
            name = args[0].upper()
            err = blocked(name, v)
            if err:
                return [err]
            self.act[name].set(v, d, now)
            return ["OK ACT"]
        if cmd == "STEER":
            v, d = num(0, -255, 255), duration(1)
            if v is None or d is None:
                return ["ERR usage: STEER <-255..255> [ms]"]
            err = blocked("A", v) or blocked("B", -v)
            if err:
                return [err]
            self.act["A"].set(v, d, now)
            self.act["B"].set(-v, d, now)
            return ["OK STEER"]
        if cmd == "STOP":
            self.stop_all()
            return ["OK STOP"]
        if cmd == "ZERO":
            self.travel = {"A": 0.0, "B": 0.0}
            self.limit_hit = {"A": False, "B": False}
            return ["OK ZERO"]
        if cmd == "LIMIT":
            v = num(0, 0, MAX_DURATION_MS)
            if v is None:
                return ["ERR usage: LIMIT <ms> (0 = off)"]
            self.limit_ms = v
            return [f"OK LIMIT {v}"]
        if cmd == "MARK":
            self.mark_start = now
            self.stream_now = True
            return ["OK MARK"]
        if cmd in ("TARE", "CALIB") and not self.hx711:
            return ["ERR hx711 missing"]
        if cmd == "TARE":
            self.offset = self.force_raw
            return ["OK TARE"]
        if cmd == "CALIB":
            return [f"OK CALIB scale={self.scale:.3f}"]
        if cmd == "SCALE":
            return ["OK SCALE"]
        if cmd == "RATE":
            v = num(0, 0, 100)
            if v is None:
                return ["ERR usage: RATE <0..100>"]
            self.rate = v
            return ["OK RATE"]
        if cmd == "TIMEOUT":
            v = num(0, 0, MAX_DURATION_MS)
            if v is None:
                return ["ERR usage: TIMEOUT <ms>"]
            self.timeout = v
            return ["OK TIMEOUT"]
        return ["ERR unknown command; send HELP"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--limit", type=int, default=2000, help="travel limit in ms (firmware default 2000)")
    ap.add_argument("--no-hx711", action="store_true", help="act as if no load cell is connected")
    args = ap.parse_args()

    master, slave = pty.openpty()
    tty.setraw(slave)
    fcntl.fcntl(master, fcntl.F_SETFL, fcntl.fcntl(master, fcntl.F_GETFL) | os.O_NONBLOCK)
    print(f"Simulated board on {os.ttyname(slave)}  (Ctrl+C to quit)", flush=True)

    board = FakeBoard(limit_ms=args.limit, hx711=not args.no_hx711)
    buf = b""
    next_sample = time.time()

    def send(lines):
        for line in lines:
            try:
                os.write(master, (line + "\r\n").encode())
            except (BlockingIOError, OSError):
                pass  # nobody is reading; drop it

    send(board.info())
    try:
        while True:
            ready, _, _ = select.select([master], [], [], 0.002)
            if ready:
                try:
                    buf += os.read(master, 1024)
                except (BlockingIOError, OSError):
                    pass
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    send(board.step())
                    send(board.handle(raw.decode(errors="replace").strip()))
            send(board.step())
            if board.rate and (board.stream_now or time.time() >= next_sample):
                board.stream_now = False
                next_sample = time.time() + 1.0 / board.rate
                send([board.telemetry()])
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
