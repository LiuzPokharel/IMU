"""Record the board's CSV stream to data/<date>_<label>.csv.

Run from the project root (close the serial monitor and live plot first: one program per port):
  .venv/bin/python host/record.py /dev/cu.usbmodem101 --seconds 600 --label rest1
"""
import argparse
from datetime import date
from pathlib import Path
import time

import serial

from live_plot import parse

DATA = Path(__file__).resolve().parents[1] / "data"


def record(ser, out, seconds, every=30):
    """Copy data rows from the open port ser into the open file out for `seconds` (host clock).

    Only rows that parse are saved, so a half row from connecting mid-stream can't break the analysis.
    Anything else the board prints (e.g. "BNO085 not found") is shown on screen instead.
    """
    counts = {"g": 0, "a": 0, "q": 0}
    start = time.monotonic()
    next_report = start + every
    while (now := time.monotonic()) - start < seconds:
        # readline() gives up after the port's timeout, so the loop still ends on time if the board goes quiet
        line = ser.readline().decode(errors="replace").strip()
        row = parse(line)
        if row:
            out.write(line + "\n")
            counts[row[1]] += 1
        elif line:
            print(f"  board says: {line}")
        if now >= next_report:
            elapsed = now - start
            print(f"  {elapsed:4.0f} / {seconds:g} s   gyro {counts['g']} rows ({counts['g'] / elapsed:.1f} Hz)"
                  f"   accel {counts['a']} rows ({counts['a'] / elapsed:.1f} Hz)   orientation {counts['q']} rows")
            next_report += every
    return counts


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record gyro, accel and orientation rows from the board to a CSV file.")
    parser.add_argument("port", help="serial port, e.g. /dev/cu.usbmodem101 (list with: ls /dev/cu.usb*)")
    parser.add_argument("--seconds", type=float, default=600, help="how long to record (default 600 = 10 min)")
    parser.add_argument("--label", required=True, help="name for the run, e.g. rest1")
    args = parser.parse_args()

    DATA.mkdir(exist_ok=True)
    path = DATA / f"{date.today().isoformat()}_{args.label}.csv"
    ser = serial.Serial(args.port, 115200, timeout=1, exclusive=True)
    print(f"recording {args.seconds:g} s to {path}")
    print("  at-rest run: don't touch the board. Movement run: hold still for 3 s, then move it.")
    with open(path, "x") as out:   # "x": refuse to overwrite an earlier run with the same name
        out.write("t_us,sensor,x,y,z\n")   # q rows carry four values: w, x, y, z
        counts = record(ser, out, args.seconds)
    print(f"done: {counts['g']} gyro, {counts['a']} accel, {counts['q']} orientation rows -> {path}")
