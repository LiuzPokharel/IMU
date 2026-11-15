"""Live 3D view: a board-shaped box that follows my Madgwick filter, with the BNO085's own answer as an outline.

Run from the project root (close the serial monitor first: one program per port):
  .venv/bin/python host/view3d.py /dev/cu.usbmodem101
  .venv/bin/python host/view3d.py --fake          # made-up motion, no board needed

Hold the board still for the first 2 s: that's when the gyro bias is measured.
"""
import argparse
from collections import deque
import threading
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import numpy as np
import serial

from compare import fake_recording, to_lines
from filters import accel_angles, angle_between, euler, madgwick_step, quat_from_euler, up_from_quat
from gyro_drift import conj, quat_mult   # importable because filters.py put sim/ on the path
from live_plot import parse, serial_lines

CALIBRATE_S = 2.0
# Half-sizes of the box, in the Feather's proportions (51 x 23 mm, drawn 6 mm thick so it reads as a slab).
CORNERS = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * [1.0, 0.45, 0.12]
FACES = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]   # -x +x -y +y -z +z
FACE_COLORS = ["#7f8c9b", "#eb6834", "#7f8c9b", "#1baf7a", "#3b4754", "#2a78d6"]   # +x orange, +y green, top blue
EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if np.sum(CORNERS[a] != CORNERS[b]) == 1]


def rot_matrix(q):
    """The 3x3 matrix that does the same rotation as q: world = R @ board."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def paced(lines):
    """Hand out recorded rows at the speed they were recorded, by their timestamps."""
    start, t0 = time.monotonic(), None
    for line in lines:
        t = int(line.split(",")[0]) / 1e6
        t0 = t if t0 is None else t0
        time.sleep(max(0, (t - t0) - (time.monotonic() - start)))
        yield line


class Tracker:
    """Feeds rows to the Madgwick filter and keeps the latest answers."""

    def __init__(self, beta):
        self.beta = beta
        self.q = None            # my orientation; None until calibration ends
        self.q_ref = None        # the BNO085's, turned so its heading starts where mine does
        self.bias = np.zeros(3)
        self.accel = None
        self.t_prev = None
        self.raw_ref = None
        self.unyaw = None
        self.still = []          # (t, gyro) collected while calibrating

    def feed(self, t_us, sensor, values):
        t, v = t_us / 1e6, np.array(values)
        if sensor == "a":
            self.accel = v
        elif sensor == "q":
            self.raw_ref = v
            if self.q is not None:
                if self.unyaw is None:
                    self.align()
                self.q_ref = quat_mult(self.unyaw, self.raw_ref * self.ref_sign)
        elif self.accel is not None:   # gyro, once there is an accel reading to start from
            if self.q is None:
                self.calibrate(t, v)
            else:
                # ponytail: the 71-minute micros() rollover gives one negative dt; the max() skips that step
                self.q = madgwick_step(self.q, v - self.bias, self.accel, max(0.0, t - self.t_prev), self.beta)
            self.t_prev = t

    def calibrate(self, t, w):
        """Collect gyro for CALIBRATE_S, then take its mean as the bias and start from the accelerometer's tilt."""
        self.still.append((t, w))
        if t - self.still[0][0] < CALIBRATE_S:
            return
        rates = np.array([w for _, w in self.still])
        if rates.std(axis=0).max() < 0.02:   # same stillness test as compare.py
            self.bias = rates.mean(axis=0)
        else:
            print("The board moved during calibration: gyro bias not removed. Restart with it held still.")
        self.q = quat_from_euler(*accel_angles(self.accel))

    def align(self):
        """Match the reference to my filter once: same quaternion direction, same starting heading."""
        up = up_from_quat(self.q)
        # If the BNO085's quaternion is world-to-board rather than board-to-world, its inverse fits "up" better.
        inverted = angle_between(up_from_quat(conj(self.raw_ref)), up) < angle_between(up_from_quat(self.raw_ref), up)
        self.ref_sign = np.array([1, -1, -1, -1]) if inverted else np.ones(4)
        yaw0 = euler(quat_mult(self.raw_ref * self.ref_sign, conj(self.q)))[2]
        self.unyaw = quat_from_euler(0, 0, -yaw0)

    def error_deg(self):
        return np.degrees(2 * np.arccos(np.clip(abs(np.dot(self.q, self.q_ref)), 0, 1)))


# --- Sanity checks -------------------------------------------------------
_q = quat_from_euler(0.3, -0.5, 1.2)
assert np.allclose(rot_matrix(_q)[2], up_from_quat(_q))          # bottom row is "up" in board axes
assert np.allclose(rot_matrix(_q) @ rot_matrix(_q).T, np.eye(3))  # a rotation matrix's inverse is its transpose
assert len(EDGES) == 12

# Fed 20 s of the made-up recording, the tracker ends within 2° of the true orientation.
_tracker = Tracker(beta=0.1)
for _line in to_lines(fake_recording(20)):
    _tracker.feed(*parse(_line))
assert _tracker.error_deg() < 2, _tracker.error_deg()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Live 3D view of the board's orientation.")
    parser.add_argument("port", nargs="?", help="serial port, e.g. /dev/cu.usbmodem101 (list with: ls /dev/cu.usb*)")
    parser.add_argument("--fake", action="store_true", help="use made-up motion instead of the board")
    parser.add_argument("--beta", type=float, default=0.1, help="Madgwick gain, rad/s")
    parser.add_argument("--seconds", type=float, help="close after this long (default: when you close the window)")
    parser.add_argument("--save", help="save the last frame to this image file")
    args = parser.parse_args()
    if args.fake == bool(args.port):
        parser.error("give a serial port or --fake, not both or neither")

    lines = paced(to_lines(fake_recording(120))) if args.fake else \
        serial_lines(serial.Serial(args.port, 115200, timeout=1, exclusive=True))
    rows = deque()   # filled by the background thread, emptied by the draw loop
    threading.Thread(target=lambda: rows.extend(r for r in map(parse, lines) if r), daemon=True).start()
    tracker = Tracker(args.beta)

    fig = plt.figure(figsize=(7, 7))
    fig.canvas.manager.set_window_title("BNO085 orientation" + (" (FAKE DATA)" if args.fake else ""))
    ax = fig.add_subplot(projection="3d")
    ax.set(xlim=(-1.2, 1.2), ylim=(-1.2, 1.2), zlim=(-1.2, 1.2), xlabel="world x", ylabel="world y", zlabel="up")
    ax.set_box_aspect((1, 1, 1))
    box = Poly3DCollection([CORNERS[f] for f in FACES], facecolors=FACE_COLORS, edgecolors="black", linewidths=0.5)
    # starts on the box's own edges: matplotlib can't add an empty 3D collection
    outline = Line3DCollection([[CORNERS[a], CORNERS[b]] for a, b in EDGES], colors="black", linewidths=1.2, linestyles="--")
    ax.add_collection3d(box)
    ax.add_collection3d(outline)

    start = time.monotonic()
    while plt.fignum_exists(fig.number) and (args.seconds is None or time.monotonic() - start < args.seconds):
        while rows:
            tracker.feed(*rows.popleft())
        if tracker.q is None:
            ax.set_title(f"Hold the board still: measuring gyro bias ({CALIBRATE_S:g} s)…", loc="left", fontsize=10)
        else:
            box.set_verts([(CORNERS @ rot_matrix(tracker.q).T)[f] for f in FACES])
            roll, pitch, yaw = np.degrees(euler(tracker.q))
            title = f"Madgwick (solid)   roll {roll:+6.1f}°  pitch {pitch:+6.1f}°  heading {yaw:+6.1f}°"
            if tracker.q_ref is not None:
                ref = CORNERS @ rot_matrix(tracker.q_ref).T * 1.03   # a hair larger so it shows around the box
                outline.set_segments([[ref[a], ref[b]] for a, b in EDGES])
                title += f"\nBNO085 fusion (dashed)   difference {tracker.error_deg():.1f}°"
            ax.set_title(title, loc="left", fontsize=10)
        plt.pause(0.03)
    if args.save:
        fig.savefig(args.save, dpi=120)
