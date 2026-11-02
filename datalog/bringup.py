#!/usr/bin/env python3
"""First power-on check for the test bench, one part at a time.

Run it after wiring and flashing, before any real test. It checks the
connection, telemetry, emergency stop, load cell, pump, both actuators and the
spare analog inputs, and asks you to confirm what you see. Every move is short
and timed by the firmware, so it stops by itself even if this script dies.

The results are printed at the end and saved to data/<date>_bringup.txt; the
telemetry goes to data/<date>_bringup.csv.

Examples:
  python datalog/bringup.py
  python datalog/bringup.py --skip estop loadcell   # parts that are not wired yet
  python datalog/bringup.py --mass 50               # also calibrate with a 50 g mass
"""

import argparse
import pathlib
import statistics
import sys
import threading
import time

from recorder import Recorder, default_path, parse_meta
from testbench import TestBench, TestBenchError

STEPS = ["telemetry", "estop", "loadcell", "pump", "act_a", "act_b", "steer", "analog"]


class Abort(Exception):
    pass


def ask(prompt, choices="yn"):
    """Ask until one of the choice letters is typed. Returns that letter."""
    while True:
        try:
            answer = input(f"{prompt} [{'/'.join(choices)}] ").strip().lower()[:1]
        except EOFError:
            raise Abort("no more input")
        if answer in choices:
            return answer


def pause(prompt):
    try:
        input(prompt)
    except EOFError:
        raise Abort("no more input")


def ready(prompt):
    """Enter = go, s = skip this step, q = stop the check."""
    try:
        answer = input(f"{prompt}\n  Enter to start, s to skip, q to quit: ").strip().lower()
    except EOFError:
        raise Abort("no more input")
    if answer.startswith("q"):
        raise Abort("stopped by user")
    return not answer.startswith("s")


class Check:
    def __init__(self, bench, rec, args):
        self.bench = bench
        self.rec = rec
        self.args = args
        self.results = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._record, daemon=True)
        self._thread.start()

    def _record(self):
        while not self._stop.is_set():
            self.rec.poll(timeout=0.05)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    # ------------------------------------------------------------ helpers

    def wait(self, seconds):
        """Wait while the background thread records; return the rows from that time."""
        t_start = time.time() - self.rec.t0
        time.sleep(seconds)
        time.sleep(0.1)  # let the last samples arrive
        t_end = time.time() - self.rec.t0
        return [r for r in list(self.rec.recent) if t_start <= r["t_s"] <= t_end]

    def last(self, column):
        row = self.rec.last or {}
        return row.get(column)

    def result(self, step, status, detail):
        self.results.append((step, status, detail))
        print(f"  -> {status}: {detail}")

    def move(self, command, seconds):
        """Send a timed command and record until it has finished."""
        self.bench.command(command)
        rows = self.wait(seconds + 0.3)
        notes = [m for m in self.bench.pop_messages() if "travel limit" in m]
        if notes:
            print(f"  {notes[0]}")
        return rows

    # ------------------------------------------------------------ steps

    def telemetry(self):
        print("\n[telemetry] Checking the data stream for 2 s...")
        rows = self.wait(2.0)
        hz = len(rows) / 2.0
        if 15 <= hz <= 25:
            self.result("telemetry", "PASS", f"{hz:.0f} samples/s")
        else:
            self.result("telemetry", "FAIL", f"{hz:.0f} samples/s, expected about 20 (USB cable? another program on the port?)")

    def estop(self):
        print("\n[estop]")
        if self.last("estop") == 1:
            return self.result("estop", "FAIL", "reads pressed with nobody touching it: the button should connect pin 10 to GND "
                                                "only while pressed (normally open), or set USE_ESTOP 0 if none is wired")
        if not ready("Press and HOLD the emergency stop."):
            return self.result("estop", "SKIP", "skipped")
        pause("  Holding it? Press Enter. ")
        self.wait(0.3)
        pressed = self.last("estop")
        pause("  Now release it and press Enter. ")
        self.wait(0.3)
        released = self.last("estop")
        if pressed == 1 and released == 0:
            self.result("estop", "PASS", "reads pressed and released")
        else:
            self.result("estop", "FAIL", f"pressed read {pressed}, released read {released}: check the button wiring to pin 10 and GND")

    def loadcell(self):
        print("\n[loadcell]")
        if self.bench.info_fields.get("hx711") != "ok":
            return self.result("loadcell", "FAIL", "HX711 not found at power-up: check DT -> 2, SCK -> 3, VCC -> 5V, GND, then reset the board")
        if not ready("Leave the load cell unloaded (nothing touching it)."):
            return self.result("loadcell", "SKIP", "skipped")
        self.bench.command("TARE", timeout=10)
        rest = [r["force_g"] for r in self.wait(2.0) if isinstance(r.get("force_g"), (int, float))]
        if len(rest) < 2:
            return self.result("loadcell", "FAIL", "no force readings")
        noise = statistics.stdev(rest)
        if self.args.mass:
            pause(f"  Put the {self.args.mass:g} g mass on the load cell and press Enter. ")
            reply = self.bench.command(f"CALIB {self.args.mass:g}", timeout=10)
            print(f"  {reply}")
            loaded = [r["force_g"] for r in self.wait(1.5) if isinstance(r.get("force_g"), (int, float))]
            reading = statistics.fmean(loaded)
            ok = abs(reading - self.args.mass) < 0.02 * self.args.mass + 0.5
            status = "PASS" if ok and noise < 0.5 else "WARN"
            return self.result("loadcell", status, f"calibrated; reads {reading:.2f} g with {self.args.mass:g} g on, "
                                                   f"noise {noise:.2f} g sd at rest")
        pause("  Press gently on the load cell (or hang any weight on it), keep it there, and press Enter. ")
        loaded = [r["force_g"] for r in self.wait(1.0) if isinstance(r.get("force_g"), (int, float))]
        change = statistics.fmean(loaded) - statistics.fmean(rest)
        if abs(change) > 2.0:
            status = "PASS" if noise < 0.5 else "WARN"
            self.result("loadcell", status, f"responds ({change:+.1f} g), noise {noise:.2f} g sd at rest"
                        + ("" if noise < 0.5 else ": noisy, check the wiring and keep it away from the pump"))
        else:
            self.result("loadcell", "FAIL", f"no change ({change:+.2f} g): check E+/E-/A+/A- between the load cell and the HX711")

    def pump(self):
        pwm, secs = self.args.pump_pwm, self.args.pump_s
        print("\n[pump]")
        if not ready(f"The pump will run at PWM {pwm} for {secs:g} s. Put the outlet in a container."):
            return self.result("pump", "SKIP", "skipped")
        rows = self.move(f"PUMP {pwm} {int(secs * 1000)}", secs)
        commanded = any(r.get("pump") == pwm for r in rows)
        stopped = self.last("pump") == 0
        answer = ask("  Did the pump run and water come out?")
        if not (commanded and stopped):
            self.result("pump", "FAIL", "the board did not run the timed command as expected")
        elif answer == "y":
            self.result("pump", "PASS", f"runs at PWM {pwm} and stops by itself")
        else:
            self.result("pump", "FAIL", "no flow: check the 24 V supply, the BTS7960 wiring (pins 8, 9, 11, 12), "
                                        "a common ground, and that the tube is in the pump head")

    def actuator(self, name):
        step = f"act_{name.lower()}"
        pwm, ms = self.args.act_pwm, self.args.act_ms
        print(f"\n[{step}]")
        if not ready(f"Actuator {name} will EXTEND for {ms} ms at PWM {pwm}, then come back.\n"
                     "  Check its syringe is not at the end of its stroke."):
            return self.result(step, "SKIP", "skipped")
        rows = self.move(f"ACT {name} {pwm} {ms}", ms / 1000)
        answer = ask(f"  What did actuator {name} do? (e = extended, r = retracted, n = nothing)", "ern")
        self.move(f"ACT {name} {-pwm} {ms}", ms / 1000)
        an = "an1" if name == "A" else "an2"
        values = [r.get(an) for r in rows if isinstance(r.get(an), int)]
        sensed = f"; {an} moved {max(values) - min(values)} counts" if values and max(values) - min(values) > 20 else ""
        if answer == "e":
            self.result(step, "PASS", f"extends and comes back{sensed}")
        elif answer == "r":
            self.result(step, "WARN", f"runs backwards: set ACT_{name}_REVERSED 1 in config.h and re-flash (or swap its two motor wires){sensed}")
        else:
            pins = "ENA 5, IN1 4, IN2 7" if name == "A" else "ENB 6, IN3 A1, IN4 A2"
            self.result(step, "FAIL", f"did not move: check {pins}, the EN jumper removed, the 12 V supply and a common ground")

    def steer(self):
        pwm, ms = self.args.act_pwm, self.args.act_ms
        print("\n[steer]")
        if not ready(f"STEER {pwm} for {ms} ms, then back (A extends while B retracts). Watch the tip."):
            return self.result("steer", "SKIP", "skipped")
        self.move(f"STEER {pwm} {ms}", ms / 1000)
        self.move(f"STEER {-pwm} {ms}", ms / 1000)
        answer = ask("  Did the tip bend to one side and come back?", "yn")
        pos = (self.last("pos_a"), self.last("pos_b"))
        if answer == "y":
            self.result("steer", "PASS", f"bends and returns (travel estimate back at {pos[0]}, {pos[1]} ms)")
        else:
            self.result("steer", "FAIL", "no bend: check the syringes, tubing and bellows for leaks or air")

    def analog(self):
        print("\n[analog] Reading an1/an2 (spare inputs: actuator position or pressure sensors) for 2 s...")
        rows = self.wait(2.0)
        parts = []
        for col in ("an1", "an2"):
            values = [r[col] for r in rows if isinstance(r.get(col), int)]
            if not values:
                continue
            spread = max(values) - min(values)
            state = "floating, probably not connected" if spread > 50 else "steady"
            parts.append(f"{col} {statistics.fmean(values):.0f} ({state})")
        self.result("analog", "INFO", ", ".join(parts) or "no readings")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skip", nargs="*", default=[], choices=STEPS, help="steps to leave out")
    ap.add_argument("--mass", type=float, help="known mass in grams: calibrate the load cell with it")
    ap.add_argument("--pump-pwm", type=int, default=80, help="pump test PWM (default 80)")
    ap.add_argument("--pump-s", type=float, default=2.0, help="pump test time in s (default 2)")
    ap.add_argument("--act-pwm", type=int, default=150, help="actuator test PWM (default 150)")
    ap.add_argument("--act-ms", type=int, default=500, help="actuator test move in ms (default 500)")
    ap.add_argument("--port", help="serial port (default: first Arduino found)")
    ap.add_argument("--meta", nargs="*", metavar="KEY=VALUE", help="metadata saved in the file header")
    ap.add_argument("--out", help="CSV path (default: data/<date>_bringup.csv); the report goes next to it")
    ap.add_argument("--boot-wait", type=float, default=2.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    print("Connecting...")
    try:
        bench = TestBench(args.port, boot_wait=args.boot_wait)
    except (TestBenchError, OSError) as e:
        sys.exit(f"FAIL connect: {e}")
    fields = bench.info_fields
    if not str(fields.get("name", "")).startswith("fams-sr-testbench"):
        bench.close()
        sys.exit(f"FAIL connect: {bench.port} answered as {bench.info!r}; flash firmware/testbench first")
    print(f"Connected to {bench.port}: {fields['name']} {fields['version']}, load cell {fields.get('hx711')}, "
          f"travel limit {fields.get('limit_ms')} ms")

    path = pathlib.Path(args.out) if args.out else default_path("bringup")
    with bench:
        for command in ("MODE SERIAL", "STOP", "TIMEOUT 0", "RATE 20", "ZERO"):
            bench.command(command)
        rec = Recorder(bench, path, parse_meta(args.meta), extra_columns=["step"])
        check = Check(bench, rec, args)
        check.results.append(("connect", "PASS", f"{fields['name']} {fields['version']} on {bench.port}"))
        steps = {"telemetry": check.telemetry, "estop": check.estop, "loadcell": check.loadcell,
                 "pump": check.pump, "act_a": lambda: check.actuator("A"),
                 "act_b": lambda: check.actuator("B"), "steer": check.steer, "analog": check.analog}
        try:
            for name in STEPS:
                if name in args.skip:
                    check.results.append((name, "SKIP", "--skip"))
                    continue
                rec.extra = {"step": name}
                try:
                    steps[name]()
                except TestBenchError as e:
                    check.result(name, "FAIL", str(e))
        except (Abort, KeyboardInterrupt) as e:
            print(f"\nStopped early ({e or 'Ctrl+C'}).")
        finally:
            bench.stop_all()
            check.close()
            rec.close()

    lines = [f"Bring-up check {time.strftime('%Y-%m-%d %H:%M')}  ({bench.info})", ""]
    lines += [f"{step:10s} {status:5s} {detail}" for step, status, detail in check.results]
    report = path.with_suffix(".txt")
    report.write_text("\n".join(lines) + "\n")
    print("\n" + "\n".join(lines))
    print(f"\nSaved {report} and {path}")
    sys.exit(1 if any(status == "FAIL" for _, status, _ in check.results) else 0)


if __name__ == "__main__":
    main()
