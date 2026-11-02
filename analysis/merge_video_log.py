#!/usr/bin/env python3
"""Line up a tracked video with the data log and merge them frame by frame.

The video and the log start at different times. MARK flashes the board's
status LED and sets mark=1 in the log (sweep.py steer sends one at the start
and end; in log_data.py just type MARK). Track the video with --led so the LED
brightness is in the track CSV, and this script matches the first flash in the
video with the first mark in the log. With no LED in view, give the offset by
hand: --offset S means video time = log time (t_s) + S.

  python analysis/merge_video_log.py steer_1_track.csv data/2026-11-04_120000_steer.csv --plot
  python analysis/merge_video_log.py run_track.csv data/run.csv --offset 3.42

Output: <track>_merged.csv, the track CSV plus the log columns at each frame
(commands held from the last sample; pos, force and analog values interpolated).
With --plot, a PNG of bend angle and actuator travel over time and against each other.
"""

import argparse
import pathlib
import sys

import numpy as np

from csvdata import column, read_csv, write_csv

HELD = ["trial", "setpoint", "phase", "pump", "act_a", "act_b", "mark"]
INTERPOLATED = ["pos_a", "pos_b", "force_g", "an1", "an2"]

INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3dd"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]


def rising_edges(values, min_range, low_before=False):
    """Indices where the values cross from the low half into the high half.
    low_before: count a high first value as an edge (the log can start on a MARK)."""
    v = np.asarray(values, float)
    if np.isfinite(v).sum() < 1 or np.nanmax(v) - np.nanmin(v) < min_range:
        return []
    threshold = (np.nanmax(v) + np.nanmin(v)) / 2
    high = np.where(np.isfinite(v), v >= threshold, False)
    edges = [i for i in range(1, len(v)) if high[i] and not high[i - 1]]
    return ([0] if low_before and high[0] else []) + edges


def log_time(log):
    """Log time in seconds on the t_s scale, taken from the board clock (t_ms),
    which has no USB timing jitter."""
    t_s = column(log, "t_s")
    if "t_ms" not in log[0]:
        return t_s
    board = column(log, "t_ms") / 1000.0
    return board + np.nanmedian(t_s - board)


def find_offset(track, log, t_video, t_log):
    # The LED came on some time during the frame before the first bright one:
    # take the middle of that interval (error within +/- half a frame).
    half_frame = float(np.nanmedian(np.diff(t_video))) / 2
    flashes = [t_video[i] - half_frame for i in rising_edges(column(track, "led"), 30.0)]
    marks = [t_log[i] for i in rising_edges(column(log, "mark"), 0.5, low_before=True)]
    if not flashes:
        sys.exit("error: no LED flash in the track CSV. Track with --led X Y W H (find it with "
                 "--select-led), or pass --offset.")
    if not marks:
        sys.exit("error: the log has no MARK (mark column never 1). Pass --offset instead.")
    offset = flashes[0] - marks[0]
    info = {"sync": "led", "offset_s": round(offset, 4),
            "led_flashes_s": " ".join(f"{t:.3f}" for t in flashes),
            "log_marks_s": " ".join(f"{t:.3f}" for t in marks)}
    print(f"First LED flash at {flashes[0]:.3f} s in the video, first MARK at {marks[0]:.3f} s in the log: "
          f"offset {offset:+.3f} s")
    if len(flashes) >= 2 and len(flashes) == len(marks):
        residual = flashes[-1] - (marks[-1] + offset)
        info["last_mark_error_s"] = round(residual, 4)
        print(f"Check with the last mark: {residual * 1000:+.0f} ms "
              + ("(good)" if abs(residual) < 0.1 else "(more than 100 ms: dropped frames or wrong --fps?)"))
    elif len(flashes) != len(marks):
        print(f"note: {len(flashes)} flashes in the video but {len(marks)} marks in the log; "
              "only the first of each was used")
    return offset, info


def merge(track, log, t_video, t_log, offset):
    t_at = t_video - offset
    inside = (t_at >= t_log[0]) & (t_at <= t_log[-1])
    idx = np.clip(np.searchsorted(t_log, t_at, side="right") - 1, 0, len(log) - 1)
    present = log[0].keys()
    interp = {}
    for name in INTERPOLATED:
        if name in present:
            y = column(log, name)
            ok = np.isfinite(y)
            interp[name] = np.interp(t_at, t_log[ok], y[ok]) if ok.sum() >= 2 else np.full_like(t_at, np.nan)
    out = []
    for i, row in enumerate(track):
        merged = dict(row)
        merged["log_t_s"] = round(float(t_at[i]), 4) if inside[i] else None
        for name in HELD:
            if name in present:
                merged[name] = log[idx[i]].get(name) if inside[i] else None
        for name, values in interp.items():
            merged[name] = round(float(values[i]), 3) if inside[i] and np.isfinite(values[i]) else None
        out.append(merged)
    return out


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)


def save_plot(rows, x_name, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = column(rows, "t_s")
    xs = column(rows, x_name)
    x_label = {"pos_a": "actuator A travel estimate (ms at full speed)",
               "an1": "actuator A position (an1, ADC counts)"}.get(x_name, x_name)
    if "bend_deg" in rows[0]:
        bend = column(rows, "bend_deg")
        fig = plt.figure(figsize=(10, 5.5))
        grid = fig.add_gridspec(2, 2, width_ratios=[1.4, 1])
        ax_bend = fig.add_subplot(grid[0, 0])
        ax_x = fig.add_subplot(grid[1, 0], sharex=ax_bend)
        ax_loop = fig.add_subplot(grid[:, 1])
        ax_bend.plot(t, bend, color=SERIES[0], linewidth=1.5)
        ax_bend.set_ylabel("bend (deg)", color=INK)
        ax_x.plot(t, xs, color=SERIES[0], linewidth=1.5)
        ax_x.set_ylabel({"pos_a": "A travel (ms)", "an1": "an1 (counts)"}.get(x_name, x_name), color=INK)
        ax_x.set_xlabel("video time (s)", color=INK)
        ax_loop.plot(xs, bend, color=SERIES[0], linewidth=1.0, alpha=0.9)
        ax_loop.set_xlabel(x_label, color=INK)
        ax_loop.set_ylabel("bend (deg)", color=INK)
        ax_loop.set_title("bend vs. stroke", color=INK, fontsize=11, loc="left")
        for ax in (ax_bend, ax_x, ax_loop):
            style(ax)
    else:
        unit = "mm" if "x_mm" in rows[0] else "px"
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 5))
        axes[0].plot(t, column(rows, f"x_{unit}"), color=SERIES[0], linewidth=1.5, label="x")
        axes[0].plot(t, column(rows, f"y_{unit}"), color=SERIES[1], linewidth=1.5, label="y")
        axes[0].set_ylabel(f"tip position ({unit})", color=INK)
        axes[0].legend(frameon=False)
        axes[1].plot(t, column(rows, "pump"), color=SERIES[0], linewidth=1.5, drawstyle="steps-post")
        axes[1].set_ylabel("pump PWM", color=INK)
        axes[1].set_xlabel("video time (s)", color=INK)
        for ax in axes:
            style(ax)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("track", type=pathlib.Path, help="CSV from tracking/track_tip.py")
    ap.add_argument("log", type=pathlib.Path, help="CSV from datalog/ (log_data.py or sweep.py)")
    ap.add_argument("--offset", type=float, help="video time = log t_s + OFFSET (skips the LED sync)")
    ap.add_argument("--x", default="pos_a", help="stroke column for the plot: pos_a (default) or an1 if wired")
    ap.add_argument("--out", type=pathlib.Path, help="output CSV (default: <track>_merged.csv)")
    ap.add_argument("--plot", action="store_true", help="save a PNG next to the output CSV")
    args = ap.parse_args()

    track_meta, track = read_csv(args.track)
    _, log = read_csv(args.log)
    t_video, t_log = column(track, "t_s"), log_time(log)

    if args.offset is not None:
        offset, info = args.offset, {"sync": "manual", "offset_s": args.offset}
    else:
        offset, info = find_offset(track, log, t_video, t_log)

    rows = merge(track, log, t_video, t_log, offset)
    covered = sum(r["log_t_s"] is not None for r in rows)
    if covered == 0:
        sys.exit("error: the video and the log do not overlap with this offset")
    print(f"{covered}/{len(rows)} frames fall inside the log")

    out = args.out or args.track.with_name(args.track.stem.replace("_track", "") + "_merged.csv")
    meta = {"video": track_meta.get("video", args.track.name), "track": args.track.name, "log": args.log.name}
    meta.update(info)
    write_csv(out, rows, meta)
    print(f"Saved {out}")
    if args.plot:
        png = out.with_suffix(".png")
        save_plot(rows, args.x, png)
        print(f"Saved {png}")


if __name__ == "__main__":
    main()
