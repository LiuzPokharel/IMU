"""Score the filters in filters.py against the BNO085's own orientation, on a recording.

Run from the project root:
  .venv/bin/python host/compare.py data/2026-11-16_move1.csv
  .venv/bin/python host/compare.py --fake                  # made-up motion with a known true orientation

The recording must start with the board held still for a few seconds: that's where the gyro bias is
measured. Prints an error table and a tuning sweep, and saves data/plots/<name>_compare.png.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

from analyze_rest import COLORS, ROOT, load
from filters import (G, accel_angles, comp_step, euler, madgwick_step, quat_from_euler, rotate_by,
                     tilt_step, up_from_angles)

sys.path.insert(0, str(ROOT / "sim"))
from gyro_drift import conj, integrate, quat_mult  # noqa: E402  (needs the path above)


def ups_from_quats(q):
    """up_from_quat() for an (N, 4) array at once."""
    w, x, y, z = q.T
    return np.column_stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def fake_recording(seconds=60.0, seed=0, bias=(0.004, -0.003, 0.006), gyro_sigma=0.003, accel_sigma=0.05):
    """A recording with a known answer, in the same shape load() returns.

    The board sits still, gets tilted and turned, gets shaken, then sits still again. Gyro and accel get
    the flaws measured on hardware day: bias and noise, accel reading 3% high, accel at 125 Hz against
    the gyro's 100. The "q" rows are the TRUE orientation, standing in for the BNO085's fusion.
    """
    rng = np.random.default_rng(seed)
    dt = 0.001                                   # truth is worked out at 1 kHz, then sampled
    t = np.arange(0, seconds, dt)
    s = t / seconds                              # 0..1 through the run
    ramp = lambda start, end: np.clip((s - start) / 0.02, 0, 1) * np.clip((end - s) / 0.02, 0, 1)  # noqa: E731

    # Still for the first 5% (at least 3 s, for the bias measurement), moving until 75%, still again to the end.
    omega = ramp(max(0.05, 3 / seconds), 0.75)[:, None] * np.column_stack([0.8 * np.sin(2 * np.pi * 0.23 * t),
                                                         0.6 * np.sin(2 * np.pi * 0.17 * t + 1),
                                                         0.5 * np.sin(2 * np.pi * 0.11 * t + 2)])
    q_true = integrate(omega, dt)
    # Shaking from 55% to 70%: real acceleration that the accelerometer can't tell apart from gravity.
    shake = ramp(0.55, 0.70)[:, None] * 6.0 * np.sin(2 * np.pi * 7 * t)[:, None] * np.array([1.0, 0.6, 0.3])

    gyro = omega + bias + rng.normal(0, gyro_sigma, omega.shape)
    accel = 1.03 * G * ups_from_quats(q_true) + shake + rng.normal(0, accel_sigma, omega.shape)
    # The BNO085's heading starts wherever it likes, so give the reference an arbitrary 40° yaw offset.
    yaw = quat_from_euler(0, 0, np.radians(40))
    q_ref = np.array([quat_mult(yaw, q) for q in q_true[::10]])

    t = t + 5.0                                  # the board had been on for 5 s
    return {"g": (t[::10], gyro[::10]), "a": (t[::8], accel[::8]), "q": (t[::10], q_ref)}


def to_lines(rec):
    """A recording as the firmware's CSV rows, in time order (for the live viewer's --fake mode)."""
    rows = [(t, f"{round(t * 1e6)},{sensor}," + ",".join(f"{v:.6f}" for v in values))
            for sensor, (ts, vals) in rec.items() for t, values in zip(ts, vals)]
    return [line for _, line in sorted(rows)]


def rest_bias(t, w, seconds):
    """Gyro bias = its mean over the first `seconds`, valid only if the board was still for them."""
    start = w[t - t[0] <= seconds]
    if seconds <= 0 or len(start) < 10:
        return np.zeros(3)
    if start.std(axis=0).max() > 0.02:   # rad/s: resting noise measured on hardware day was well under this
        print(f"  WARNING: the board was moving in the first {seconds:g} s, so the gyro bias is NOT removed. "
              "Re-record starting at rest.")
        return np.zeros(3)
    return start.mean(axis=0)


def run_filters(rec, tau=1.0, kp=1.0, beta=0.1, bias_seconds=2.0):
    """Run every method over a recording. Returns (t, {name: "up" vectors (N, 3)}, {name: quaternions (N, 4)}, bias)."""
    t, w = rec["g"]
    t_a, a = rec["a"]
    bias = rest_bias(t, w, bias_seconds)
    w = w - bias
    dt = np.diff(t, prepend=t[0])   # each gyro sample's own step from the timestamps
    # Gyro and accel arrive at different rates: each gyro sample uses the most recent accel reading.
    latest = np.clip(np.searchsorted(t_a, t, side="right") - 1, 0, None)

    N = len(t)
    angles = accel_angles(a[latest[0]])   # start where the accelerometer says, not at zero
    q0 = quat_from_euler(*angles)
    q_gyro, q_tilt, q_mad = q0, q0, q0
    ups = {name: np.zeros((N, 3)) for name in ("accel only", "complementary")}
    quats = {name: np.zeros((N, 4)) for name in ("gyro only", "gyro + accel correction", "Madgwick")}
    for n in range(N):
        an = a[latest[n]]
        angles = comp_step(angles, w[n], an, dt[n], tau)
        q_gyro = rotate_by(q_gyro, w[n], dt[n])
        q_tilt = tilt_step(q_tilt, w[n], an, dt[n], kp)
        q_mad = madgwick_step(q_mad, w[n], an, dt[n], beta)
        ups["accel only"][n] = up_from_angles(*accel_angles(an))
        ups["complementary"][n] = up_from_angles(*angles)
        quats["gyro only"][n], quats["gyro + accel correction"][n], quats["Madgwick"][n] = q_gyro, q_tilt, q_mad
    for name, q in quats.items():
        ups[name] = ups_from_quats(q)
    return t, ups, quats, bias


def reference(rec, t, q0):
    """The BNO085's orientation at each gyro time, turned about "up" so its heading starts where ours does.

    Heading has no absolute zero without a magnetometer: the BNO085 picks one and my filters pick another.
    Only the heading CHANGE can be compared, so the starting offset is removed.
    """
    t_q, q = rec["q"]
    q = q[np.clip(np.searchsorted(t_q, t, side="right") - 1, 0, None)]
    yaw0 = euler(quat_mult(q[0], conj(q0)))[2]
    unyaw = quat_from_euler(0, 0, -yaw0)
    return np.array([quat_mult(unyaw, qn) for qn in q])


def angle_deg(u, v):
    """Angle between matching rows of two (N, 3) arrays of unit vectors, in degrees."""
    return np.degrees(np.arccos(np.clip((u * v).sum(axis=1), -1, 1)))


def rotation_deg(q, r):
    """Angle of the rotation between matching rows of two (N, 4) quaternion arrays, in degrees."""
    return np.degrees(2 * np.arccos(np.clip(np.abs((q * r).sum(axis=1)), 0, 1)))


rms = lambda x: float(np.sqrt(np.mean(x ** 2)))  # noqa: E731


# --- Sanity checks -------------------------------------------------------
# Quaternions that differ only by heading have the same "up", and rotation_deg sees the heading.
_q = np.array([quat_from_euler(0.3, -0.5, 0.0), quat_from_euler(0.3, -0.5, 1.0)])
assert np.allclose(ups_from_quats(_q)[0], ups_from_quats(_q)[1])
assert np.isclose(rotation_deg(_q[:1], _q[1:])[0], np.degrees(1.0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Score the orientation filters against the BNO085's own fusion.")
    parser.add_argument("csv", nargs="?", help="recording from host/record.py that includes q rows")
    parser.add_argument("--fake", action="store_true", help="use made-up motion with a known true orientation")
    parser.add_argument("--seconds", type=float, default=60, help="length of the --fake run (default 60)")
    parser.add_argument("--tau", type=float, default=1.0, help="complementary filter time constant, s")
    parser.add_argument("--kp", type=float, default=1.0, help="accel correction gain, 1/s")
    parser.add_argument("--beta", type=float, default=0.1, help="Madgwick gain, rad/s")
    parser.add_argument("--bias-seconds", type=float, default=2.0,
                        help="seconds at the start, board still, used to measure gyro bias (0 = don't remove it)")
    args = parser.parse_args()
    if args.fake == bool(args.csv):
        parser.error("give a recording or --fake, not both or neither")

    rec = fake_recording(args.seconds) if args.fake else load(args.csv)
    name = "fake" if args.fake else Path(args.csv).stem
    if "q" not in rec:
        sys.exit("This recording has no q rows (the BNO085's own orientation). Upload the current imu "
                 "firmware and record again.")

    t, ups, quats, bias = run_filters(rec, args.tau, args.kp, args.beta, args.bias_seconds)
    q_ref = reference(rec, t, quats["gyro only"][0])
    # Hardware check: is the BNO085's quaternion board-to-world like ours, or the inverse? The accelerometer
    # knows which way is up, so whichever agrees with it over the first second is the right reading.
    first = t - t[0] <= 1.0
    if rms(angle_deg(ups_from_quats(q_ref * [1, -1, -1, -1])[first], ups["accel only"][first])) < \
            rms(angle_deg(ups_from_quats(q_ref)[first], ups["accel only"][first])):
        print("NOTE: the reference quaternion is world-to-board; inverting it.")
        rec["q"] = rec["q"][0], rec["q"][1] * [1, -1, -1, -1]
        q_ref = reference(rec, t, quats["gyro only"][0])
    up_ref = ups_from_quats(q_ref)
    t0 = t - t[0]

    print(f"{name}: {t0[-1]:.0f} s, {len(t)} gyro samples")
    print("  gyro bias removed (rad/s): " + ", ".join(f"{v:+.5f}" for v in bias))
    print(f"  tau = {args.tau:g} s, kp = {args.kp:g} /s, beta = {args.beta:g} rad/s\n")
    print("ERROR AGAINST THE REFERENCE (degrees)")
    print(f"  {'method':<26}{'tilt RMS':>10}{'tilt max':>10}{'total RMS':>11}{'total max':>11}{'total at end':>14}")
    tilt_err, total_err = {}, {}
    for method in ("accel only", "gyro only", "complementary", "gyro + accel correction", "Madgwick"):
        tilt_err[method] = angle_deg(ups[method], up_ref)
        row = f"  {method:<26}{rms(tilt_err[method]):>10.2f}{tilt_err[method].max():>10.2f}"
        if method in quats:   # heading too, so the whole rotation can be compared
            total_err[method] = rotation_deg(quats[method], q_ref)
            row += f"{rms(total_err[method]):>11.2f}{total_err[method].max():>11.2f}{total_err[method][-1]:>14.2f}"
        print(row)
    print("  tilt = roll and pitch together (the angle between the two \"up\" directions).")
    print("  total = tilt and heading. Nothing here corrects heading, so it drifts with whatever gyro bias is left.")

    # --- Tuning sweep: how does each filter's one knob trade noise against drift? ----------------
    print("\nTUNING SWEEP (tilt RMS, degrees)")
    for knob, values, method in (("tau", (0.1, 0.3, 1, 3, 10), "complementary"),
                                 ("kp", (0.1, 0.3, 1, 3, 10), "gyro + accel correction"),
                                 ("beta", (0.01, 0.03, 0.1, 0.3, 1), "Madgwick")):
        scores = [rms(angle_deg(run_filters(rec, **{**dict(tau=args.tau, kp=args.kp, beta=args.beta), knob: v},
                                            bias_seconds=args.bias_seconds)[1][method], up_ref)) for v in values]
        print(f"  {method:<26}" + "".join(f"  {knob} {v:<5g}{s:>6.2f}" for v, s in zip(values, scores)))

    # --- Plot ----------------------------------------------------------------
    import matplotlib.pyplot as plt

    fig, (ax_r, ax_p, ax_e) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ref_angles = np.degrees([euler(q)[:2] for q in q_ref])
    lines = [("accel only", "#b0b0b0", 0.8), ("gyro only", COLORS[1], 1.2),
             ("complementary", COLORS[0], 1.5), ("Madgwick", COLORS[2], 1.5)]
    for i, (ax, label) in enumerate(((ax_r, "roll (deg)"), (ax_p, "pitch (deg)"))):
        for method, color, lw in lines:
            if method in quats:
                ang = np.degrees([euler(q)[i] for q in quats[method]])
            else:   # recover the angle from the stored "up" vector
                ang = np.degrees([accel_angles(u)[i] for u in ups[method]])
            ax.plot(t0, ang, color=color, lw=lw, label=method)
        ax.plot(t0, ref_angles[:, i], color="black", lw=1, ls="--", label="reference")
        ax.set_ylabel(label)
    for method, color, lw in lines + [("gyro + accel correction", "#8a4fd3", 1.2)]:
        ax_e.semilogy(t0, np.maximum(tilt_err[method], 1e-3), color=color, lw=lw, label=method)
    ax_e.set(xlabel="seconds since recording start", ylabel="tilt error (deg, log scale)")
    for ax in (ax_r, ax_p, ax_e):
        ax.grid(True, which="both", alpha=0.3)
    ax_r.legend(ncols=5, frameon=False, loc="lower left", bbox_to_anchor=(0, 1))
    ax_e.legend(ncols=5, frameon=False, fontsize=8, loc="lower left", bbox_to_anchor=(0, 1))
    fig.suptitle(f"{name}: filters against the " + ("true orientation" if args.fake else "BNO085's own fusion"), y=0.995)
    fig.tight_layout()
    plots = ROOT / "data" / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    fig.savefig(plots / f"{name}_compare.png", dpi=120)
    print(f"\nsaved {plots / f'{name}_compare.png'}")
