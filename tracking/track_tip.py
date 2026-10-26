#!/usr/bin/env python3
"""Track the robot tip in a video and measure its speed and bending angle.

Two ways to find the tip in each frame:
  color  a colored marker on the tip, found by HSV color (recommended for new videos).
         Add --hsv2 for a second marker near the base of the bending segment to get
         the bending angle.
  roi    an OpenCV tracker started from a box around the tip (works on old videos
         with no marker; less reliable).

Typical use:
  python tracking/track_tip.py run.mov --pick                # click the marker to get its HSV range
  python tracking/track_tip.py run.mov --calibrate           # click two ruler marks to get px/mm
  python tracking/track_tip.py run.mov --hsv 40 80 80 80 255 255 --px-per-mm 4.2 --plot
  python tracking/track_tip.py old.mov --mode roi --px-per-mm 4.2 --plot

Output: <video>_track.csv (one row per frame) and, with --plot, <video>_track.png.
Positions are relative to where the tip starts; x is to the right and y is up.
"""

import argparse
import csv
import math
import pathlib
import sys

import cv2
import numpy as np


# ------------------------------------------------------------------ helpers

def open_video(path, fps_override=None):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        sys.exit(f"error: cannot open {path}")
    fps = fps_override or cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        sys.exit("error: the video has no frame rate; pass --fps")
    return cap, fps


def read_frame_at(cap, fps, seconds):
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(seconds * fps)))
    ok, frame = cap.read()
    if not ok:
        sys.exit("error: could not read a frame at the start time")
    return frame


def hsv_mask(hsv, lo, hi):
    """inRange that also handles red, where the hue range wraps past 180."""
    lo, hi = np.array(lo, np.uint8), np.array(hi, np.uint8)
    if lo[0] <= hi[0]:
        return cv2.inRange(hsv, lo, hi)
    a = cv2.inRange(hsv, np.array([0, lo[1], lo[2]], np.uint8), hi)
    b = cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]], np.uint8))
    return cv2.bitwise_or(a, b)


def find_marker(hsv, rng, min_area):
    mask = hsv_mask(hsv, rng[:3], rng[3:])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    m = cv2.moments(c)
    return (m["m10"] / m["m00"], m["m01"] / m["m00"])


def make_tracker():
    makers = [
        lambda: cv2.TrackerCSRT_create(),
        lambda: cv2.legacy.TrackerCSRT_create(),
        lambda: cv2.TrackerKCF_create(),
        lambda: cv2.TrackerMIL_create(),
    ]
    for make in makers:
        try:
            return make()
        except AttributeError:
            continue
    sys.exit("error: no OpenCV tracker available; pip install opencv-contrib-python")


def moving_average(values, window):
    """Centered moving average. Frames without a full window of found points
    (the start and end, and next to gaps) are left as NaN instead of biased."""
    values = np.asarray(values, float)
    if window <= 1:
        return values
    ok = ~np.isnan(values)
    kernel = np.ones(window)
    total = np.convolve(np.where(ok, values, 0.0), kernel, mode="same")
    count = np.convolve(ok.astype(float), kernel, mode="same")
    out = np.full_like(values, np.nan)
    full = count >= window
    out[full] = total[full] / window
    return out


def central_diff(values, times):
    values, times = np.asarray(values, float), np.asarray(times, float)
    out = np.full_like(values, np.nan)
    if len(values) >= 3:
        out[1:-1] = (values[2:] - values[:-2]) / (times[2:] - times[:-2])
    return out


# ------------------------------------------------------------------ interactive tools

def pick_colors(path, fps, start):
    cap, _ = open_video(path, fps)
    frame = read_frame_at(cap, fps, start)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    print("Click the marker (several spots is better). Press q when done.")
    picks = []

    def on_click(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            patch = hsv[max(0, y - 2):y + 3, max(0, x - 2):x + 3].reshape(-1, 3)
            picks.extend(patch.tolist())
            h, s, v = hsv[y, x]
            print(f"  ({x}, {y}): H={h} S={s} V={v}")

    cv2.namedWindow("pick", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("pick", on_click)
    while True:
        cv2.imshow("pick", frame)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()
    if not picks:
        return
    arr = np.array(picks)
    h_lo, h_hi = int(arr[:, 0].min()) - 8, int(arr[:, 0].max()) + 8
    s_lo, v_lo = max(0, int(arr[:, 1].min()) - 40), max(0, int(arr[:, 2].min()) - 40)
    print(f"\nSuggested: --hsv {h_lo % 180} {s_lo} {v_lo} {h_hi % 180} 255 255")


def calibrate(path, fps, start):
    cap, _ = open_video(path, fps)
    frame = read_frame_at(cap, fps, start)
    print("Click two points a known distance apart (e.g. two ruler marks).")
    points = []

    def on_click(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
            points.append((x, y))
            cv2.circle(frame, (x, y), 4, (0, 0, 255), -1)

    cv2.namedWindow("calibrate", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("calibrate", on_click)
    while len(points) < 2:
        cv2.imshow("calibrate", frame)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            return
    cv2.imshow("calibrate", frame)
    cv2.waitKey(300)
    cv2.destroyAllWindows()
    mm = float(input("Distance between the two points in mm: "))
    px = math.dist(points[0], points[1])
    print(f"\n{px:.1f} px over {mm} mm -> --px-per-mm {px / mm:.3f}")


# ------------------------------------------------------------------ tracking

def track(args, fps):
    cap, _ = open_video(args.video, fps)
    start_frame = int(round(args.start * fps))
    end_frame = int(round(args.end * fps)) if args.end else None
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    tracker = None
    writer = None
    rows = []
    frame_idx = start_frame
    while True:
        if end_frame is not None and frame_idx > end_frame:
            break
        ok, frame = cap.read()
        if not ok:
            break
        t = (frame_idx - start_frame) / fps
        tip = base = None

        if args.mode == "color":
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            tip = find_marker(hsv, args.hsv, args.min_area)
            if args.hsv2:
                base = find_marker(hsv, args.hsv2, args.min_area)
        else:
            if tracker is None:
                box = tuple(args.roi) if args.roi else cv2.selectROI("select the tip", frame, False)
                cv2.destroyAllWindows()
                if not box or box[2] == 0:
                    sys.exit("error: no box selected")
                tracker = make_tracker()
                tracker.init(frame, tuple(int(v) for v in box))
                tip = (box[0] + box[2] / 2, box[1] + box[3] / 2)
            else:
                found, box = tracker.update(frame)
                if found:
                    tip = (box[0] + box[2] / 2, box[1] + box[3] / 2)

        rows.append({"frame": frame_idx, "t_s": t, "tip": tip, "base": base})

        if args.annotate:
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(str(args.annotate), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            for p, color in ((tip, (0, 0, 255)), (base, (255, 0, 0))):
                if p is not None:
                    cv2.circle(frame, (int(p[0]), int(p[1])), 6, color, 2)
            if tip is not None and base is not None:
                cv2.line(frame, (int(base[0]), int(base[1])), (int(tip[0]), int(tip[1])), (0, 255, 255), 1)
            writer.write(frame)
        frame_idx += 1

    if writer is not None:
        writer.release()
    return rows


def analyze(rows, args):
    t = np.array([r["t_s"] for r in rows])
    x = np.array([r["tip"][0] if r["tip"] else np.nan for r in rows])
    y = np.array([r["tip"][1] if r["tip"] else np.nan for r in rows])
    found = ~np.isnan(x)
    if not found.any():
        sys.exit("error: the tip was never found. Check --hsv (try --pick) or the --roi box.")

    scale = args.px_per_mm or 1.0
    unit = "mm" if args.px_per_mm else "px"
    first = np.argmax(found)
    dx = (x - x[first]) / scale
    dy = -(y - y[first]) / scale  # image y points down; flip so up is positive

    # Smooth only for the derivative; positions and angles are reported raw.
    sx, sy = moving_average(dx, args.smooth), moving_average(dy, args.smooth)
    vx, vy = central_diff(sx, t), central_diff(sy, t)
    speed = np.hypot(vx, vy)

    angle = None
    if args.hsv2:
        bx = np.array([r["base"][0] if r["base"] else np.nan for r in rows])
        by = np.array([r["base"][1] if r["base"] else np.nan for r in rows])
        raw = np.degrees(np.arctan2(-(y - by), x - bx))
        ok = ~np.isnan(raw)
        if ok.any():
            unwrapped = np.full_like(raw, np.nan)
            unwrapped[ok] = np.degrees(np.unwrap(np.radians(raw[ok])))
            angle = unwrapped - unwrapped[ok][0]

    out = []
    for i, r in enumerate(rows):
        row = {
            "frame": r["frame"],
            "t_s": round(t[i], 4),
            "found": int(found[i]),
            "x_px": None if np.isnan(x[i]) else round(x[i], 2),
            "y_px": None if np.isnan(y[i]) else round(y[i], 2),
            f"x_{unit}": None if np.isnan(dx[i]) else round(dx[i], 3),
            f"y_{unit}": None if np.isnan(dy[i]) else round(dy[i], 3),
            f"speed_{unit}_s": None if np.isnan(speed[i]) else round(speed[i], 3),
        }
        if angle is not None:
            row["bend_deg"] = None if np.isnan(angle[i]) else round(angle[i], 2)
        out.append(row)

    last = len(found) - 1 - np.argmax(found[::-1])
    duration = t[last] - t[first]
    net = math.hypot(dx[last] - dx[first], dy[last] - dy[first])
    summary = {
        "frames": len(rows),
        "found_frames": int(found.sum()),
        "duration_s": round(duration, 3),
        f"net_displacement_{unit}": round(net, 2),
        f"mean_speed_{unit}_s": round(net / duration, 3) if duration > 0 else None,
        f"peak_speed_{unit}_s": round(float(np.nanmax(speed)), 3) if np.isfinite(speed).any() else None,
    }
    if angle is not None and np.isfinite(angle).any():
        summary["max_bend_deg"] = round(float(np.nanmax(angle)), 2)
        summary["min_bend_deg"] = round(float(np.nanmin(angle)), 2)
    return out, summary, unit


def save_plot(rows, unit, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = [r["t_s"] for r in rows]

    def col(name):
        return [np.nan if r.get(name) is None else r[name] for r in rows]

    has_angle = "bend_deg" in rows[0]
    fig, axes = plt.subplots(3 if has_angle else 2, 1, sharex=True, figsize=(8, 7 if has_angle else 5))
    axes[0].plot(t, col(f"x_{unit}"), label="x")
    axes[0].plot(t, col(f"y_{unit}"), label="y")
    axes[0].set_ylabel(f"position ({unit})")
    axes[0].legend()
    axes[1].plot(t, col(f"speed_{unit}_s"))
    axes[1].set_ylabel(f"speed ({unit}/s)")
    if has_angle:
        axes[2].plot(t, col("bend_deg"))
        axes[2].set_ylabel("bend (deg)")
    axes[-1].set_xlabel("time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", type=pathlib.Path)
    ap.add_argument("--mode", choices=["color", "roi"], default="color")
    ap.add_argument("--hsv", type=int, nargs=6, metavar=("H1", "S1", "V1", "H2", "S2", "V2"),
                    help="tip marker HSV range (OpenCV scale: H 0-179, S and V 0-255)")
    ap.add_argument("--hsv2", type=int, nargs=6, metavar=("H1", "S1", "V1", "H2", "S2", "V2"),
                    help="second marker at the base of the bending segment (for bend angle)")
    ap.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
                    help="roi mode: box around the tip in the first frame (default: draw it)")
    ap.add_argument("--px-per-mm", type=float, help="image scale; without it results are in pixels")
    ap.add_argument("--fps", type=float, help="override the frame rate (slow-motion phone videos)")
    ap.add_argument("--start", type=float, default=0.0, help="start time in seconds")
    ap.add_argument("--end", type=float, help="end time in seconds")
    ap.add_argument("--smooth", type=int, default=5, help="moving-average window in frames")
    ap.add_argument("--min-area", type=float, default=20.0, help="smallest marker blob in px^2")
    ap.add_argument("--out", type=pathlib.Path, help="CSV path (default: <video>_track.csv)")
    ap.add_argument("--plot", action="store_true", help="save a PNG plot next to the CSV")
    ap.add_argument("--annotate", type=pathlib.Path, help="write a video with the tracked points drawn")
    ap.add_argument("--pick", action="store_true", help="click the marker to get an HSV range")
    ap.add_argument("--calibrate", action="store_true", help="click two points to get px/mm")
    args = ap.parse_args()

    cap, fps = open_video(args.video, args.fps)
    cap.release()
    if args.pick:
        return pick_colors(args.video, fps, args.start)
    if args.calibrate:
        return calibrate(args.video, fps, args.start)
    if args.mode == "color" and not args.hsv:
        sys.exit("error: color mode needs --hsv (run with --pick to find it), or use --mode roi")

    rows = track(args, fps)
    out_rows, summary, unit = analyze(rows, args)

    out = args.out or args.video.with_name(args.video.stem + "_track.csv")
    with open(out, "w", newline="") as f:
        f.write(f"# video: {args.video.name}\n# fps: {fps}\n# mode: {args.mode}\n")
        f.write(f"# px_per_mm: {args.px_per_mm}\n")
        for key, value in summary.items():
            f.write(f"# {key}: {value}\n")
        writer = csv.DictWriter(f, fieldnames=list(out_rows[0].keys()))
        writer.writeheader()
        writer.writerows(out_rows)

    print(f"Tracked {summary['found_frames']}/{summary['frames']} frames at {fps:g} fps")
    for key, value in summary.items():
        if key not in ("frames", "found_frames"):
            print(f"  {key}: {value}")
    print(f"Saved {out}")
    if args.plot:
        png = out.with_suffix(".png")
        save_plot(out_rows, unit, png)
        print(f"Saved {png}")


if __name__ == "__main__":
    main()
