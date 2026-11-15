"""Live-plot the board's CSV stream, or fake data when there's no hardware.

Run from the project root:
  .venv/bin/python host/live_plot.py --fake
  .venv/bin/python host/live_plot.py /dev/cu.usbmodem101
"""
import argparse
from collections import deque
from itertools import islice
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import serial

WINDOW = 1000                                # samples kept per sensor: 10 s at 100 Hz
COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]   # x, y, z


def fake_lines(dt=0.01):
    """Pretend to be the board sitting still: same CSV rows as the firmware, every dt seconds."""
    rng = np.random.default_rng(0)
    bias = np.array([0.01, -0.005, 0.02])    # rad/s, same as sim/gyro_drift.py
    gyro_sigma = 0.01                        # rad/s
    accel_sigma = 0.05                       # m/s^2
    gravity = np.array([0.0, 0.0, 9.81])     # at rest, all the accelerometer feels is gravity
    t_us = 0

    while True:
        gyro = bias + rng.normal(0.0, gyro_sigma, 3)   # true rate is 0, so: bias + noise
        accel = gravity + rng.normal(0.0, accel_sigma, 3)
        yield f"{t_us},g,{gyro[0]:.6f},{gyro[1]:.6f},{gyro[2]:.6f}"
        yield f"{t_us},a,{accel[0]:.6f},{accel[1]:.6f},{accel[2]:.6f}"
        t_us += 10000
        time.sleep(dt)


def serial_lines(ser):
    """Yield text rows from an open serial port. readline() returns b"" if it times out."""
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            yield line


def parse(line):
    """'timestamp_us,sensor,x,y,z' -> (t_us, sensor, [x, y, z]), or None if it isn't a data row.
    Sensors: g gyro, a accel (3 values each), q the BNO085's own orientation (4 values: w, x, y, z)."""
    parts = line.split(",")
    if len(parts) < 2 or len(parts) - 2 != {"g": 3, "a": 3, "q": 4}.get(parts[1]):
        return None
    try:
        return int(parts[0]), parts[1], [float(v) for v in parts[2:]]
    except ValueError:   # e.g. a row cut in half when we connected mid-stream
        return None


def read_stream(lines, data):
    """Sort rows into data["g"] and data["a"]. Runs in the background so a slow redraw can't lose rows."""
    for line in lines:
        row = parse(line)
        if row is None:
            print(line)   # not data, e.g. "BNO085 not found" from the firmware
            continue
        t_us, sensor, values = row
        if sensor in data:   # this plot shows gyro and accel only
            data[sensor].append((t_us, *values))


def draw(data, ax, plots, name, unit):
    """Redraw one sensor's panel. The title shows the mean (at rest, the gyro's mean is its bias) and the rate."""
    rows = np.array(list(data))   # list() first: a copy the background thread can't change mid-read
    if len(rows) < 2:
        return

    # ponytail: firmware timestamps wrap every ~71 min and this window doesn't handle it; fine for 5-min runs
    t = (rows[:, 0] - rows[-1, 0]) / 1e6   # seconds, with the newest sample at 0
    for i, plot in enumerate(plots):
        plot.set_data(t, rows[:, i + 1])
    ax.relim()
    ax.autoscale_view(scalex=False)

    mean = rows[:, 1:].mean(axis=0)
    rate = (len(rows) - 1) / (t[-1] - t[0])
    ax.set_title(f"{name} ({unit})   mean x {mean[0]:+.4f}, y {mean[1]:+.4f}, z {mean[2]:+.4f}   {rate:.0f} Hz",
                 loc="left", fontsize=10)


# --- Sanity checks -------------------------------------------------------
assert parse("1000,g,0.1,-0.2,0.3") == (1000, "g", [0.1, -0.2, 0.3])
assert parse("1000,q,1,0,0,0") == (1000, "q", [1.0, 0.0, 0.0, 0.0])
assert parse("1000,q,1,0,0") is None   # a quaternion needs 4 values
assert parse("BNO085 not found") is None
assert parse("1000,x,0.1,-0.2,0.3") is None   # unknown sensor
assert parse("1000,g,0.1,-0.,oops") is None   # damaged number

# The fake gyro's average should land on its bias, the same way a real gyro's will.
fake = [parse(line) for line in islice(fake_lines(dt=0), 2000)]
fake_gyro = np.array([row[2] for row in fake if row[1] == "g"])
assert np.allclose(fake_gyro.mean(axis=0), [0.01, -0.005, 0.02], atol=0.002)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live-plot gyro and accelerometer data.")
    parser.add_argument("port", nargs="?", help="serial port, e.g. /dev/cu.usbmodem101 (list with: ls /dev/cu.usb*)")
    parser.add_argument("--fake", action="store_true", help="use fake data instead of the board")
    args = parser.parse_args()
    # No silent fallback to fake data: on hardware day you must never mistake it for the real sensor.
    if args.fake == bool(args.port):
        parser.error("give a serial port or --fake, not both or neither")

    # exclusive=True: if another program (e.g. the serial monitor) has the port, fail now with a clear error
    lines = fake_lines() if args.fake else serial_lines(serial.Serial(args.port, 115200, timeout=1, exclusive=True))
    data = {"g": deque(maxlen=WINDOW), "a": deque(maxlen=WINDOW)}
    threading.Thread(target=read_stream, args=(lines, data), daemon=True).start()

    fig, (ax_g, ax_a) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.canvas.manager.set_window_title("BNO085 live" + (" (FAKE DATA)" if args.fake else ""))
    gyro_plots = [ax_g.plot([], [], color=c, lw=1.5, label=axis)[0] for c, axis in zip(COLORS, "xyz")]
    accel_plots = [ax_a.plot([], [], color=c, lw=1.5, label=axis)[0] for c, axis in zip(COLORS, "xyz")]
    for ax in (ax_g, ax_a):
        ax.set_xlim(-WINDOW * 0.01, 0)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="lower right", bbox_to_anchor=(1, 1), ncols=3, frameon=False)   # above the axes, clear of the data
    ax_a.set_xlabel("seconds before now")
    fig.tight_layout()

    while plt.fignum_exists(fig.number):   # until you close the window
        draw(data["g"], ax_g, gyro_plots, "Gyro", "rad/s")
        draw(data["a"], ax_a, accel_plots, "Accel", "m/s²")
        plt.pause(0.05)
