"""Write test bench samples to a CSV file with a metadata header."""

import collections
import csv
import datetime
import pathlib
import queue
import time

REPO = pathlib.Path(__file__).resolve().parent.parent


def default_path(label):
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    return REPO / "data" / f"{stamp}_{label}.csv"


def parse_meta(items):
    """Turn ["tip=V21", "trial=1"] into {"tip": "V21", "trial": "1"}."""
    meta = {}
    for item in items or []:
        if "=" not in item:
            raise SystemExit(f"--meta expects key=value, got {item!r}")
        key, value = item.split("=", 1)
        meta[key.strip()] = value.strip()
    return meta


class Recorder:
    """Pulls samples off a TestBench and appends them to a CSV file.

    Metadata goes at the top as `# key: value` lines, so the file still loads
    with pandas.read_csv(path, comment="#").
    """

    def __init__(self, bench, path, meta=None, extra_columns=(), keep=2000):
        self.bench = bench
        self.path = pathlib.Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = open(self.path, "w", newline="")
        self.file.write(f"# started: {datetime.datetime.now().isoformat(timespec='seconds')}\n")
        self.file.write(f"# port: {bench.port}\n")
        self.file.write(f"# firmware: {bench.info}\n")
        for key, value in (meta or {}).items():
            self.file.write(f"# {key}: {value}\n")
        self.fields = ["t_s"] + list(bench.columns) + list(extra_columns) + ["note"]
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields, extrasaction="ignore")
        self.writer.writeheader()
        bench.drain()  # drop samples that arrived before recording started
        self.t0 = time.time()
        self.extra = {}
        self.pending_note = ""
        self.count = 0
        self.recent = collections.deque(maxlen=keep)
        self.last = None

    def add_note(self, text):
        self.pending_note = text

    def write(self, sample):
        row = dict(sample)
        row["t_s"] = round(sample["host_time"] - self.t0, 4)
        row.update(self.extra)
        if self.pending_note:
            row["note"] = self.pending_note
            self.pending_note = ""
        self.writer.writerow(row)
        self.count += 1
        self.recent.append(row)
        self.last = row
        return row

    def poll(self, timeout=0.1):
        """Write every queued sample. Returns the rows written."""
        rows = []
        try:
            rows.append(self.write(self.bench.samples.get(timeout=timeout)))
            while True:
                rows.append(self.write(self.bench.samples.get_nowait()))
        except queue.Empty:
            pass
        return rows

    def record_for(self, seconds):
        """Record for `seconds` and return the rows written in that window."""
        rows = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            rows.extend(self.poll(timeout=0.05))
        return rows

    def close(self):
        self.file.flush()
        self.file.close()
