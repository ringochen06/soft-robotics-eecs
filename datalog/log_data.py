#!/usr/bin/env python3
"""Record test bench telemetry to CSV while you send commands.

While it records, type firmware commands (PUMP 120, STEER -80, TARE, STOP...).
A line starting with "!" is saved as a note on the next sample.
Type "quit" or press Ctrl+C to stop; the pump and actuators are turned off on exit.

Examples:
  python datalog/log_data.py --meta tip=V21 tubing=0.8mm test=thrust
  python datalog/log_data.py --plot force_g pump --duration 60
"""

import argparse
import sys
import threading
import time

from recorder import Recorder, default_path, parse_meta
from testbench import TestBench, TestBenchError


def read_stdin(bench, recorder, stop):
    for line in sys.stdin:
        text = line.strip()
        if not text:
            continue
        if text.lower() in ("quit", "exit", "q"):
            stop.set()
            return
        if text.startswith("!"):
            recorder.add_note(text[1:].strip())
            print(f"  note saved: {text[1:].strip()}")
            continue
        bench.write(text)
    # stdin closed (e.g. run from a script): keep recording until --duration or Ctrl+C


def print_device_output(bench):
    while not bench.replies.empty():
        print(f"  {bench.replies.get_nowait()}")
    while not bench.messages.empty():
        print(f"  {bench.messages.get_nowait()}")


def status_line(recorder, columns):
    last = recorder.last or {}
    values = " ".join(f"{c}={last.get(c)}" for c in columns if c in last)
    return f"[{time.time() - recorder.t0:6.1f} s] {recorder.count} samples  {values}"


def run_plain(bench, recorder, stop, duration, show):
    next_status = time.time() + 2.0
    while not stop.is_set():
        recorder.poll(timeout=0.1)
        print_device_output(bench)
        now = time.time()
        if duration and now - recorder.t0 >= duration:
            break
        if now >= next_status:
            print(status_line(recorder, show))
            next_status = now + 2.0


def run_plot(bench, recorder, stop, duration, columns):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, axes = plt.subplots(len(columns), 1, sharex=True, squeeze=False)
    lines = []
    for ax, col in zip(axes[:, 0], columns):
        (line,) = ax.plot([], [])
        ax.set_ylabel(col)
        ax.grid(True, alpha=0.3)
        lines.append(line)
    axes[-1, 0].set_xlabel("time (s)")
    fig.canvas.manager.set_window_title(str(recorder.path.name))

    def update(_frame):
        recorder.poll(timeout=0.01)
        print_device_output(bench)
        if stop.is_set() or (duration and time.time() - recorder.t0 >= duration):
            plt.close(fig)
            return lines
        rows = list(recorder.recent)
        if rows:
            t = [r["t_s"] for r in rows]
            for line, ax, col in zip(lines, axes[:, 0], columns):
                y = [r.get(col) for r in rows]
                line.set_data(t, [v if isinstance(v, (int, float)) else float("nan") for v in y])
                ax.relim()
                ax.autoscale_view()
        return lines

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    plt.show()
    stop.set()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first Arduino found)")
    ap.add_argument("--out", help="CSV path (default: data/<date>_<label>.csv)")
    ap.add_argument("--label", default="log", help="label used in the default file name")
    ap.add_argument("--meta", nargs="*", metavar="KEY=VALUE", help="metadata saved in the file header")
    ap.add_argument("--rate", type=int, help="telemetry rate in Hz (firmware default 20)")
    ap.add_argument("--duration", type=float, help="stop after this many seconds")
    ap.add_argument("--plot", nargs="+", metavar="COLUMN", help="live plot these columns")
    ap.add_argument("--show", nargs="*", default=["pump", "act_a", "force_g"],
                    help="columns printed in the status line")
    ap.add_argument("--boot-wait", type=float, default=2.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    meta = parse_meta(args.meta)
    try:
        bench = TestBench(args.port, boot_wait=args.boot_wait)
    except (TestBenchError, OSError) as e:
        sys.exit(f"error: {e}")

    with bench:
        if args.rate is not None:
            bench.command(f"RATE {args.rate}")
        path = args.out or default_path(args.label)
        recorder = Recorder(bench, path, meta)
        print(f"Connected to {bench.port} ({bench.info})")
        print(f"Recording to {recorder.path}")
        print('Type commands (e.g. PUMP 120, STOP), "!text" for a note, "quit" to stop.')

        stop = threading.Event()
        threading.Thread(target=read_stdin, args=(bench, recorder, stop), daemon=True).start()
        try:
            if args.plot:
                run_plot(bench, recorder, stop, args.duration, args.plot)
            else:
                run_plain(bench, recorder, stop, args.duration, args.show)
        except KeyboardInterrupt:
            pass
        finally:
            recorder.close()
            print(f"\nSaved {recorder.count} samples to {recorder.path}")


if __name__ == "__main__":
    main()
