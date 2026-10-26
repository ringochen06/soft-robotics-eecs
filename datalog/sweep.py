#!/usr/bin/env python3
"""Step the pump through PWM values and measure thrust or flow at each one.

thrust: the tip is fixed to the load cell. The load cell is tared with the pump
        off, then at each PWM the pump runs for --settle s and the force is
        averaged over the next --hold s.
flow:   the outlet goes into a cup. At each PWM you press Enter, the pump runs
        for --hold s and stops, and you type the mass of water collected.
        Flow (mL/min) = mass (g) / hold (s) * 60, taking water as 1 g/mL.

Every sample is saved to the main CSV; one row per step goes to *_summary.csv.

Examples:
  python datalog/sweep.py thrust --meta tip=V21 tubing=0.8mm
  python datalog/sweep.py flow --pwm 100 150 200 255 --hold 30 --repeat 3
"""

import argparse
import csv
import statistics
import sys

from recorder import Recorder, default_path, parse_meta
from testbench import TestBench, TestBenchError

G_TO_MN = 9.80665  # 1 gram-force in millinewtons

DEFAULT_PWM = {
    "thrust": [0, 60, 90, 120, 150, 180, 210, 240, 255],
    "flow": [80, 120, 160, 200, 240, 255],
}
DEFAULT_HOLD = {"thrust": 5.0, "flow": 30.0}


def mean_std(values):
    values = [v for v in values if isinstance(v, (int, float))]
    if not values:
        return None, None, 0
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return statistics.fmean(values), std, len(values)


def run_thrust(bench, rec, args):
    if not args.no_tare:
        print("Taring the load cell with the pump off...")
        bench.command("TARE", timeout=10)
    rows = []
    for trial in range(1, args.repeat + 1):
        for pwm in args.pwm:
            bench.command(f"PUMP {pwm}")
            rec.extra = {"trial": trial, "setpoint": pwm, "phase": "settle"}
            rec.record_for(args.settle)
            rec.extra["phase"] = "hold"
            samples = rec.record_for(args.hold)
            mean, std, n = mean_std([s.get("force_g") for s in samples])
            if n == 0:
                print("  no force readings; is the HX711 connected?")
            row = {
                "trial": trial,
                "pwm": pwm,
                "n": n,
                "force_mean_g": None if mean is None else round(mean, 3),
                "force_std_g": None if std is None else round(std, 3),
                "thrust_mN": None if mean is None else round(mean * G_TO_MN, 2),
            }
            rows.append(row)
            print(f"  trial {trial}  PWM {pwm:3d}: {row['force_mean_g']} g "
                  f"(sd {row['force_std_g']}, n={n}) = {row['thrust_mN']} mN")
    bench.command("PUMP 0")
    rec.extra = {"trial": "", "setpoint": 0, "phase": "settle"}
    rec.record_for(args.settle)
    rec.extra["phase"] = "rest"
    mean, _, n = mean_std([s.get("force_g") for s in rec.record_for(args.hold)])
    if n:
        print(f"  zero after sweep: {mean:.2f} g (tare drift; should be close to 0)")
    return rows


def run_flow(bench, rec, args):
    rows = []
    for trial in range(1, args.repeat + 1):
        for pwm in args.pwm:
            input(f"\nTrial {trial}, PWM {pwm}: put the outlet in an empty cup, then press Enter.")
            rec.extra = {"trial": trial, "setpoint": pwm, "phase": "collect"}
            bench.command(f"PUMP {pwm}")
            rec.record_for(args.hold)
            bench.command("PUMP 0")
            rec.extra = {"trial": trial, "setpoint": 0, "phase": "weigh"}
            while True:
                text = input("  Water collected (g): ").strip()
                try:
                    mass = float(text)
                    break
                except ValueError:
                    print("  Enter a number, e.g. 85.3")
            rec.poll(timeout=0)
            flow = mass / args.hold * 60.0
            rows.append({"trial": trial, "pwm": pwm, "hold_s": args.hold,
                         "mass_g": mass, "flow_ml_min": round(flow, 1)})
            print(f"  PWM {pwm}: {flow:.1f} mL/min")
    fit_flow(rows)
    return rows


def fit_flow(rows):
    points = [(r["pwm"], r["flow_ml_min"]) for r in rows if r["flow_ml_min"] > 0]
    if len({p for p, _ in points}) < 2:
        return
    n = len(points)
    mx = sum(p for p, _ in points) / n
    my = sum(f for _, f in points) / n
    sxx = sum((p - mx) ** 2 for p, _ in points)
    slope = sum((p - mx) * (f - my) for p, f in points) / sxx
    intercept = my - slope * mx
    print(f"\nLinear fit: flow = {slope:.3f} * PWM + {intercept:.1f}  (mL/min)")
    if slope > 0:
        print(f"Flow reaches zero near PWM {-intercept / slope:.0f}")


def save_summary(path, rows):
    if not rows:
        return None
    out = path.with_name(path.stem + "_summary.csv")
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["thrust", "flow"])
    ap.add_argument("--pwm", type=int, nargs="+", help="PWM steps, 0-255")
    ap.add_argument("--repeat", type=int, default=1, help="how many times to run the sweep")
    ap.add_argument("--settle", type=float, default=2.0, help="thrust: seconds to wait after each change")
    ap.add_argument("--hold", type=float, help="seconds measured (thrust) or pumped (flow) per step")
    ap.add_argument("--no-tare", action="store_true", help="thrust: skip the tare at the start")
    ap.add_argument("--port", help="serial port (default: first Arduino found)")
    ap.add_argument("--out", help="CSV path (default: data/<date>_<mode>.csv)")
    ap.add_argument("--meta", nargs="*", metavar="KEY=VALUE", help="metadata saved in the file header")
    ap.add_argument("--boot-wait", type=float, default=2.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    args.pwm = args.pwm or DEFAULT_PWM[args.mode]
    args.hold = args.hold or DEFAULT_HOLD[args.mode]
    if any(not 0 <= p <= 255 for p in args.pwm):
        sys.exit("error: PWM values must be 0-255")

    meta = parse_meta(args.meta)
    meta.update({"test": args.mode, "pwm_steps": " ".join(map(str, args.pwm)),
                 "hold_s": args.hold, "settle_s": args.settle, "repeat": args.repeat})
    try:
        bench = TestBench(args.port, boot_wait=args.boot_wait)
    except (TestBenchError, OSError) as e:
        sys.exit(f"error: {e}")

    with bench:
        bench.command("MODE SERIAL")
        path = args.out or default_path(args.mode)
        rec = Recorder(bench, path, meta, extra_columns=["trial", "setpoint", "phase"])
        print(f"Connected to {bench.port} ({bench.info})")
        print(f"Recording to {rec.path}")
        rows = []
        try:
            rows = run_thrust(bench, rec, args) if args.mode == "thrust" else run_flow(bench, rec, args)
        except KeyboardInterrupt:
            print("\nStopped.")
        except TestBenchError as e:
            print(f"\nerror: {e}")
        finally:
            bench.stop_all()
            rec.close()
        summary = save_summary(rec.path, rows)
        print(f"\nSaved {rec.count} samples to {rec.path}")
        if summary:
            print(f"Summary: {summary}")


if __name__ == "__main__":
    main()
