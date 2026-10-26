#!/usr/bin/env python3
"""Fake test bench board for trying the scripts without hardware (macOS/Linux).

It speaks the same serial protocol as firmware/testbench and makes up
plausible numbers: thrust rises with pump PWM, and the actuator "positions"
(an1, an2) move with the actuator commands.

  python datalog/sim_device.py                 # prints a port such as /dev/ttys012
  python datalog/log_data.py --port /dev/ttys012
"""

import fcntl
import os
import pty
import random
import select
import time
import tty

HEADER = "H,t_ms,mode,pump,act_a,act_b,force_g,an1,an2,pot,joy,estop"


class FakeBoard:
    def __init__(self):
        self.t0 = time.time()
        self.mode = "S"
        self.pump = 0
        self.act = {"A": 0, "B": 0}
        self.pos = {"A": 512.0, "B": 512.0}
        self.force_raw = 0.0  # grams before tare
        self.offset = 0.0
        self.scale = 455.0
        self.rate = 20
        self.last = time.time()

    def millis(self):
        return int((time.time() - self.t0) * 1000)

    def flow_ml_min(self):
        duty = abs(self.pump)
        return 0.0 if duty < 50 else (duty - 50) * 2.4

    def step(self):
        now = time.time()
        dt = now - self.last
        self.last = now
        target = 1.2e-5 * self.flow_ml_min() ** 2  # grams of thrust
        self.force_raw += (target - self.force_raw) * min(1.0, dt / 0.5)
        for k in self.pos:
            self.pos[k] = min(1023.0, max(0.0, self.pos[k] + self.act[k] / 255.0 * 150.0 * dt))

    def telemetry(self):
        force = self.force_raw - self.offset + random.gauss(0, 0.05)
        return (f"D,{self.millis()},{self.mode},{self.pump},{self.act['A']},{self.act['B']},"
                f"{force:.2f},{int(self.pos['A'])},{int(self.pos['B'])},0,512,0")

    def info(self):
        return [f"I,fams-sr-testbench-sim,0.1.0,scale={self.scale:.3f},hx711=ok", HEADER]

    def handle(self, line):
        parts = line.split()
        if not parts:
            return []
        cmd, args = parts[0].upper(), parts[1:]

        def num(i, lo, hi):
            try:
                v = int(args[i])
            except (IndexError, ValueError):
                return None
            return v if lo <= v <= hi else None

        if cmd == "PING":
            return ["OK PONG"]
        if cmd == "INFO":
            return self.info()
        if cmd == "HELP":
            return ["# see firmware/testbench/README.md"]
        if cmd == "MODE" and args and args[0].upper() in ("SERIAL", "MANUAL"):
            self.mode = "S" if args[0].upper() == "SERIAL" else "M"
            self.pump, self.act = 0, {"A": 0, "B": 0}
            return [f"OK MODE {args[0].upper()}"]
        if cmd in ("PUMP", "ACT", "STEER") and self.mode != "S":
            return ["ERR in manual mode; send MODE SERIAL first"]
        if cmd == "PUMP":
            v = num(0, -255, 255)
            if v is None:
                return ["ERR usage: PUMP <-255..255>"]
            self.pump = v
            return ["OK PUMP"]
        if cmd == "ACT":
            v = num(1, -255, 255)
            if v is None or not args or args[0].upper() not in ("A", "B"):
                return ["ERR usage: ACT A|B <-255..255>"]
            self.act[args[0].upper()] = v
            return ["OK ACT"]
        if cmd == "STEER":
            v = num(0, -255, 255)
            if v is None:
                return ["ERR usage: STEER <-255..255>"]
            self.act = {"A": v, "B": -v}
            return ["OK STEER"]
        if cmd == "STOP":
            self.pump, self.act = 0, {"A": 0, "B": 0}
            return ["OK STOP"]
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
            return ["OK TIMEOUT"]
        return ["ERR unknown command; send HELP"]


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)
    fcntl.fcntl(master, fcntl.F_SETFL, fcntl.fcntl(master, fcntl.F_GETFL) | os.O_NONBLOCK)
    print(f"Simulated board on {os.ttyname(slave)}  (Ctrl+C to quit)", flush=True)

    board = FakeBoard()
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
            ready, _, _ = select.select([master], [], [], 0.01)
            if ready:
                try:
                    buf += os.read(master, 1024)
                except (BlockingIOError, OSError):
                    pass
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    send(board.handle(raw.decode(errors="replace").strip()))
            board.step()
            if board.rate and time.time() >= next_sample:
                next_sample = time.time() + 1.0 / board.rate
                send([board.telemetry()])
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
