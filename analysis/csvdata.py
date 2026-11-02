"""Read the CSV files written by datalog/ and tracking/ (with # metadata lines)."""

import csv
import pathlib

import numpy as np


def _convert(text):
    if text is None or text == "":
        return None
    for cast in (int, float):
        try:
            return cast(text)
        except ValueError:
            pass
    return text


def read_csv(path):
    """Return (meta, rows): meta from the "# key: value" lines, rows as dicts of numbers/strings."""
    meta, lines = {}, []
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#"):
                key, _, value = line[1:].partition(":")
                meta[key.strip()] = value.strip()
            else:
                lines.append(line)
    rows = [{k: _convert(v) for k, v in row.items()} for row in csv.DictReader(lines)]
    if not rows:
        raise SystemExit(f"error: {path} has no data rows")
    return meta, rows


def column(rows, name):
    """A column as a float array, with NaN for blanks and text."""
    return np.array([r.get(name) if isinstance(r.get(name), (int, float)) else np.nan for r in rows], float)


def write_csv(path, rows, meta=None):
    path = pathlib.Path(path)
    fields = list(rows[0].keys())
    for row in rows[1:]:
        fields += [k for k in row if k not in fields]
    with open(path, "w", newline="") as f:
        for key, value in (meta or {}).items():
            f.write(f"# {key}: {value}\n")
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path
