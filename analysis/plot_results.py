#!/usr/bin/env python3
"""Figures from the test data: flow, thrust (with the ideal jet thrust), steering, speed.

  flow    PWM -> flow rate, from sweep.py flow summaries (mean +/- SD over trials, linear fit)
  thrust  PWM -> thrust, from sweep.py thrust summaries. With --flow, also thrust
          vs. the measured flow next to the ideal jet thrust F = rho Q^2 cos(theta) / A_jets
  steer   bend angle vs. actuator stroke, from merge_video_log.py output (one line per file)
  speed   PWM -> forward speed, from track_tip.py CSVs given as PWM:FILE

Examples:
  python analysis/plot_results.py flow data/*_flow_summary.csv
  python analysis/plot_results.py thrust data/*_thrust_summary.csv --flow data/*_flow_summary.csv
  python analysis/plot_results.py steer steer_1_merged.csv steer_2_merged.csv --x pos_a
  python analysis/plot_results.py speed 120:fwd_1_track.csv 120:fwd_2_track.csv 180:fwd_3_track.csv

Each command prints the numbers behind the figure and saves a PNG (--out to choose the name).
"""

import argparse
import math
import pathlib
import statistics
import sys

import numpy as np

from csvdata import column, read_csv

G_TO_MN = 9.80665
RHO_WATER = 998.0  # kg/m^3 at 20 C

INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3dd"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]


def plt_module():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 10, "axes.titlesize": 11, "axes.titlelocation": "left"})
    return plt


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)
    ax.xaxis.label.set_color(INK)
    ax.yaxis.label.set_color(INK)
    ax.title.set_color(INK)


def by_pwm(rows, value):
    """{pwm: [values]} for one column of summary rows."""
    groups = {}
    for r in rows:
        v = value(r)
        if isinstance(r.get("pwm"), (int, float)) and isinstance(v, (int, float)):
            groups.setdefault(int(r["pwm"]), []).append(float(v))
    return dict(sorted(groups.items()))


def mean_sd(groups):
    pwm = np.array(list(groups))
    mean = np.array([statistics.fmean(v) for v in groups.values()])
    sd = np.array([statistics.stdev(v) if len(v) > 1 else 0.0 for v in groups.values()])
    n = np.array([len(v) for v in groups.values()])
    return pwm, mean, sd, n


def load_rows(paths):
    rows = []
    for p in paths:
        rows += read_csv(p)[1]
    return rows


def flow_fit(rows):
    """Least-squares flow = a * pwm + b over the trials with flow > 0."""
    pts = [(r["pwm"], r["flow_ml_min"]) for r in rows
           if isinstance(r.get("flow_ml_min"), (int, float)) and r["flow_ml_min"] > 0]
    if len({p for p, _ in pts}) < 2:
        return None
    x, y = np.array(pts, float).T
    a, b = np.polyfit(x, y, 1)
    return a, b


def ideal_thrust_mn(q_ml_min, jets, d_mm, angle_deg):
    """Momentum flux of the jets: F = rho Q v cos(theta), v = Q / (jets * pi d^2 / 4)."""
    q = np.asarray(q_ml_min, float) * 1e-6 / 60.0
    area = jets * math.pi * (d_mm * 1e-3) ** 2 / 4.0
    return RHO_WATER * q ** 2 / area * math.cos(math.radians(angle_deg)) * 1000.0


def out_path(args, default):
    return pathlib.Path(args.out) if args.out else default


# ------------------------------------------------------------------ commands

def cmd_flow(args):
    rows = load_rows(args.files)
    pwm, mean, sd, n = mean_sd(by_pwm(rows, lambda r: r.get("flow_ml_min")))
    fit = flow_fit(rows)
    print("PWM   flow mL/min (mean +/- SD, n)")
    for p, m, s, k in zip(pwm, mean, sd, n):
        print(f"{p:3d}   {m:7.1f} +/- {s:5.1f}  ({k})")
    plt = plt_module()
    fig, ax = plt.subplots(figsize=(5.5, 4))
    ax.errorbar(pwm, mean, yerr=sd, fmt="o", color=SERIES[0], markersize=6, capsize=3,
                elinewidth=1.2, label="measured (mean ± SD)")
    if fit:
        a, b = fit
        xs = np.linspace(0, 255, 100)
        ax.plot(xs, a * xs + b, "--", color=MUTED, linewidth=1.2, label=f"fit: {a:.2f}·PWM {b:+.0f}")
        print(f"fit: flow = {a:.3f} * PWM {b:+.1f} mL/min" + (f"; zero flow near PWM {-b / a:.0f}" if a > 0 else ""))
    ax.set_xlim(0, 260)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("pump PWM (0-255)")
    ax.set_ylabel("flow (mL/min)")
    ax.set_title("Pump flow through the tether")
    ax.legend(frameon=False)
    style(ax)
    fig.tight_layout()
    path = out_path(args, pathlib.Path(args.files[0]).with_name("flow.png"))
    fig.savefig(path, dpi=200)
    print(f"Saved {path}")


def cmd_thrust(args):
    rows = load_rows(args.files)
    thrust = lambda r: r.get("thrust_mN") if r.get("thrust_mN") is not None else (
        r["force_mean_g"] * G_TO_MN if isinstance(r.get("force_mean_g"), (int, float)) else None)
    pwm, mean, sd, n = mean_sd(by_pwm(rows, thrust))
    plt = plt_module()
    panels = 2 if args.flow else 1
    fig, axes = plt.subplots(1, panels, figsize=(5.5 * panels, 4), squeeze=False)
    ax = axes[0, 0]
    ax.errorbar(pwm, mean, yerr=sd, fmt="o", color=SERIES[0], markersize=6, capsize=3, elinewidth=1.2)
    ax.set_xlim(0, 260)
    ax.set_xlabel("pump PWM (0-255)")
    ax.set_ylabel("thrust (mN)")
    ax.set_title("Measured thrust (mean ± SD)")
    style(ax)

    if not args.flow:
        print("PWM   thrust mN (mean +/- SD, n)")
        for p, m, s, k in zip(pwm, mean, sd, n):
            print(f"{p:3d}   {m:7.2f} +/- {s:5.2f}  ({k})")
    else:
        flow_rows = load_rows(args.flow)
        fpwm, fmean, _, _ = mean_sd(by_pwm(flow_rows, lambda r: r.get("flow_ml_min")))
        fit = flow_fit(flow_rows)
        # Flow at each thrust PWM: between measured points interpolate, outside use the fit.
        q = np.interp(pwm, fpwm, fmean)
        if fit:
            outside = (pwm < fpwm.min()) | (pwm > fpwm.max())
            q[outside] = np.maximum(fit[0] * pwm[outside] + fit[1], 0.0)
        ideal = ideal_thrust_mn(q, args.jets, args.jet_d_mm, args.jet_angle_deg)
        print(f"Ideal jet thrust: {args.jets} jets of {args.jet_d_mm} mm at {args.jet_angle_deg} deg, rho {RHO_WATER:g} kg/m^3")
        print("PWM   flow mL/min   measured mN        ideal mN   measured/ideal")
        for p, qq, m, s, i in zip(pwm, q, mean, sd, ideal):
            ratio = f"{m / i:.2f}" if i > 0.5 else "-"
            print(f"{p:3d}   {qq:9.1f}   {m:7.2f} +/- {s:5.2f}   {i:8.2f}   {ratio}")
        ax = axes[0, 1]
        qs = np.linspace(0, max(q.max(), 1.0) * 1.05, 100)
        ax.plot(qs, ideal_thrust_mn(qs, args.jets, args.jet_d_mm, args.jet_angle_deg), color=SERIES[1],
                linewidth=1.5, label="ideal jet thrust ρQ²cosθ/A")
        ax.errorbar(q, mean, yerr=sd, fmt="o", color=SERIES[0], markersize=6, capsize=3, elinewidth=1.2,
                    label="measured")
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.set_xlabel("flow (mL/min, measured)")
        ax.set_ylabel("thrust (mN)")
        ax.set_title("Thrust vs. flow")
        ax.legend(frameon=False)
        style(ax)
    fig.tight_layout()
    path = out_path(args, pathlib.Path(args.files[0]).with_name("thrust.png"))
    fig.savefig(path, dpi=200)
    print(f"Saved {path}")


def cmd_steer(args):
    if len(args.files) > len(SERIES):
        sys.exit(f"error: at most {len(SERIES)} files per figure; split them into several figures")
    labels = args.labels or [pathlib.Path(f).stem.replace("_merged", "") for f in args.files]
    plt = plt_module()
    fig, ax = plt.subplots(figsize=(5.5, 4.5))
    print(f"file   max bend +   max bend -   ({args.x} range)")
    for f, label, color in zip(args.files, labels, SERIES):
        _, rows = read_csv(f)
        if "bend_deg" not in rows[0] or args.x not in rows[0]:
            sys.exit(f"error: {f} needs bend_deg and {args.x} columns (track with --hsv2, then merge_video_log.py)")
        x, bend = column(rows, args.x), column(rows, "bend_deg")
        ok = np.isfinite(x) & np.isfinite(bend)
        ax.plot(x[ok], bend[ok], color=color, linewidth=1.2, label=label)
        print(f"{label}   {np.nanmax(bend[ok]):+.1f} deg   {np.nanmin(bend[ok]):+.1f} deg   "
              f"({np.nanmin(x[ok]):.0f} to {np.nanmax(x[ok]):.0f})")
    ax.axhline(0, color=MUTED, linewidth=0.8)
    ax.axvline(0, color=MUTED, linewidth=0.8)
    ax.set_xlabel({"pos_a": "actuator A travel estimate (ms at full speed)",
                   "an1": "actuator A position (an1, ADC counts)"}.get(args.x, args.x))
    ax.set_ylabel("bend angle (deg)")
    ax.set_title("Bidirectional steering")
    if len(args.files) > 1:
        ax.legend(frameon=False)
    style(ax)
    fig.tight_layout()
    path = out_path(args, pathlib.Path(args.files[0]).with_name("steer.png"))
    fig.savefig(path, dpi=200)
    print(f"Saved {path}")


def cmd_speed(args):
    groups, unit = {}, None
    for item in args.files:
        pwm_text, _, f = item.partition(":")
        if not f or not pwm_text.isdigit():
            sys.exit(f"error: give speed files as PWM:FILE, got {item!r}")
        meta, _ = read_csv(f)
        key = next((k for k in meta if k.startswith("mean_speed_")), None)
        if key is None:
            sys.exit(f"error: {f} has no mean_speed line; is it a track_tip.py CSV?")
        unit = key[len("mean_speed_"):].replace("_s", "/s")
        groups.setdefault(int(pwm_text), []).append(float(meta[key]))
    pwm, mean, sd, n = mean_sd(dict(sorted(groups.items())))
    print(f"PWM   speed {unit} (mean +/- SD, n)")
    for p, m, s, k in zip(pwm, mean, sd, n):
        print(f"{p:3d}   {m:7.2f} +/- {s:5.2f}  ({k})")
    plt = plt_module()
    fig, ax = plt.subplots(figsize=(5.5, 4))
    ax.errorbar(pwm, mean, yerr=sd, fmt="o", color=SERIES[0], markersize=6, capsize=3, elinewidth=1.2)
    ax.set_xlim(0, 260)
    ax.set_ylim(bottom=0)
    ax.set_xlabel("pump PWM (0-255)")
    ax.set_ylabel(f"forward speed ({unit})")
    ax.set_title("Forward speed (mean ± SD)")
    style(ax)
    fig.tight_layout()
    path = out_path(args, pathlib.Path(args.files[0].partition(":")[2]).with_name("speed.png"))
    fig.savefig(path, dpi=200)
    print(f"Saved {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("flow", help="PWM -> flow")
    p.add_argument("files", nargs="+")
    p.set_defaults(run=cmd_flow)

    p = sub.add_parser("thrust", help="PWM -> thrust, and thrust vs flow with --flow")
    p.add_argument("files", nargs="+")
    p.add_argument("--flow", nargs="+", help="flow summary CSV(s) from the same tip and tubing")
    p.add_argument("--jets", type=int, default=2, help="number of jets (default 2)")
    p.add_argument("--jet-d-mm", type=float, default=0.8, help="jet diameter in mm (default 0.8)")
    p.add_argument("--jet-angle-deg", type=float, default=4.0, help="jet angle from the axis (default 4)")
    p.set_defaults(run=cmd_thrust)

    p = sub.add_parser("steer", help="bend angle vs stroke")
    p.add_argument("files", nargs="+")
    p.add_argument("--x", default="pos_a", help="stroke column: pos_a (default) or an1 if a position sensor is wired")
    p.add_argument("--labels", nargs="+", help="legend labels, one per file")
    p.set_defaults(run=cmd_steer)

    p = sub.add_parser("speed", help="PWM -> forward speed")
    p.add_argument("files", nargs="+", metavar="PWM:FILE")
    p.set_defaults(run=cmd_speed)

    for p in sub.choices.values():
        p.add_argument("--out", help="PNG path")
    args = ap.parse_args()
    args.run(args)


if __name__ == "__main__":
    main()
