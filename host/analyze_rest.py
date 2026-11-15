"""Analyze an at-rest recording made by host/record.py.

Run from the project root:
  .venv/bin/python host/analyze_rest.py data/2026-11-05_rest1.csv

Prints timing, gyro bias + noise, accel magnitude + tilt, and gyro-only drift.
Saves two plots to data/plots/: <name>_bias.png (warm-up drift) and <name>_drift.png (real vs. simulated).
"""
import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
from gyro_drift import drift_error, growth_exponent, integrate  # noqa: E402  (needs the path above)

COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]   # x, y, z: same as live_plot.py
WINDOW_S = 30                                # rolling-mean window for bias over time, seconds


def unwrap_us(t):
    """micros() is a 32-bit counter: it rolls over to 0 every ~71.6 min. Add 2^32 after each rollover."""
    return t + 2**32 * np.cumsum(np.diff(t, prepend=t[0]) < -2**31)


def load(path):
    """Read a recording into {"g": (t, xyz), "a": (t, xyz)}. t is seconds since board power-on, xyz is (N, 3).
    Recordings that include the BNO085's own orientation also get "q": (t, wxyz)."""
    with open(path) as f:
        next(f)   # skip the header
        rows = [line.rstrip().split(",") for line in f]
    data = {}
    for sensor in ("g", "a", "q"):
        arr = np.array([[r[0], *r[2:]] for r in rows if r[1] == sensor], dtype=float)
        if not len(arr):
            continue   # older recordings have no q rows
        data[sensor] = unwrap_us(arr[:, 0]) / 1e6, arr[:, 1:]
    return data


def timing(t):
    """Sample rate and gaps from timestamps in seconds. A step over 1.5x the typical step counts as a gap."""
    dt = np.diff(t)
    typical = np.median(dt)
    gaps = dt > 1.5 * typical
    return dict(
        rate=1 / typical,                              # from the typical step
        overall=(len(t) - 1) / (t[-1] - t[0]),         # over the whole run, gaps included
        gaps=int(gaps.sum()),
        missing=int(np.round(dt[gaps] / typical).sum() - gaps.sum()),   # a 3-step gap = 2 lost samples
        longest_ms=dt.max() * 1e3,
        backwards=int((dt <= 0).sum()),                # timestamps that didn't increase: should be 0
    )


def error_deg(q):
    """Angle of each orientation away from the start. At rest the truth never moves, so this IS the drift.
    Same formula as the sim: angle = 2·arccos(|w|)."""
    return np.degrees(2 * np.arccos(np.clip(np.abs(q[:, 0]), 0, 1)))


# --- Sanity checks -------------------------------------------------------
assert list(unwrap_us(np.array([2**32 - 20, 2**32 - 10, 0, 10]))) == [2**32 - 20, 2**32 - 10, 2**32, 2**32 + 10]
_tm = timing(np.array([0, 10, 20, 50, 60]) / 1e3)   # 30 and 40 ms are missing
assert _tm["gaps"] == 1 and _tm["missing"] == 2 and _tm["backwards"] == 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze an at-rest gyro + accel recording.")
    parser.add_argument("csv", help="recording from host/record.py, e.g. data/2026-11-05_rest1.csv")
    args = parser.parse_args()

    data = load(args.csv)
    name = Path(args.csv).stem
    plots = ROOT / "data" / "plots"
    plots.mkdir(parents=True, exist_ok=True)

    # --- Timing ----------------------------------------------------------
    print("TIMING")
    for sensor, label in (("g", "gyro"), ("a", "accel")):
        t = data[sensor][0]
        tm = timing(t)
        print(f"  {label:<6}{len(t):>7} samples over {t[-1] - t[0]:.1f} s   {tm['rate']:.1f} Hz typical, "
              f"{tm['overall']:.1f} Hz overall   gaps {tm['gaps']} (~{tm['missing']} samples lost), "
              f"longest step {tm['longest_ms']:.1f} ms, backwards {tm['backwards']}")

    # --- Gyro bias and noise -----------------------------------------------
    # At rest the true rate is 0, so the mean IS the bias and the spread around it IS the noise.
    t, w = data["g"]
    bias, sigma = w.mean(axis=0), w.std(axis=0)
    print("\nGYRO AT REST            x           y           z")
    print("  bias  (rad/s) " + "".join(f"{v:>+12.5f}" for v in bias))
    print("  bias  (deg/s) " + "".join(f"{v:>+12.4f}" for v in np.degrees(bias)))
    print("  noise (rad/s) " + "".join(f"{v:>12.5f}" for v in sigma))
    print("  noise (deg/s) " + "".join(f"{v:>12.4f}" for v in np.degrees(sigma)))

    # --- Bias over time (warm-up) ------------------------------------------
    # Rolling mean: average each 30 s window of samples. Noise averages out; a slow change in bias stays.
    n = round(WINDOW_S * timing(t)["rate"])
    fig, ax = plt.subplots(figsize=(10, 4))
    if len(t) > n:
        rolled = np.column_stack([np.convolve(w[:, i], np.ones(n) / n, mode="valid") for i in range(3)])
        t_mid = t[n // 2: n // 2 + len(rolled)]   # time at the middle of each window
        change = np.degrees(rolled[-1] - rolled[0])
        print(f"  bias change, first vs. last {WINDOW_S} s (deg/s) " + "".join(f"{v:>+10.4f}" for v in change))
        for i, axis in enumerate("xyz"):
            ax.plot(t_mid, np.degrees(rolled[:, i]), color=COLORS[i], lw=1.5, label=axis)
    else:
        print(f"  (recording shorter than {WINDOW_S} s: no bias-over-time plot)")
    ax.set(xlabel="seconds since board power-on", ylabel="gyro bias (deg/s)",
           title=f"{name}: gyro bias over time ({WINDOW_S} s rolling mean)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncols=3, frameon=False)
    fig.tight_layout()
    fig.savefig(plots / f"{name}_bias.png", dpi=120)

    # --- Accelerometer -----------------------------------------------------
    # At rest the accel feels only gravity, so |a| should be 9.81 and its direction says which way is down.
    a = data["a"][1]
    mag = np.linalg.norm(a, axis=1)
    m = a.mean(axis=0)
    tilt = np.degrees(np.arccos(m[2] / np.linalg.norm(m)))   # angle between gravity and the board's z axis
    print("\nACCEL AT REST")
    print(f"  |a| mean {mag.mean():.3f} m/s² (std {mag.std():.3f}), {100 * (mag.mean() / 9.81 - 1):+.1f}% vs. 9.81")
    print(f"  mean vector x {m[0]:+.3f}, y {m[1]:+.3f}, z {m[2]:+.3f} m/s²   tilt from level {tilt:.2f}°")

    # --- Gyro-only drift: real vs. simulated --------------------------------
    steps = np.diff(t, prepend=t[0])   # each sample's own dt from the timestamps (gaps included)
    t0 = t - t[0]
    err_real = error_deg(integrate(w, steps))
    err_debiased = error_deg(integrate(w - bias, steps))   # hindsight: subtracts the mean of this same run
    # The sim with this run's own bias and noise, board at rest. If bias were truly constant, it'd match.
    t_sim, err_sim = drift_error(bias, sigma, T=t0[-1], dt=np.median(np.diff(t)), rate=(0.0, 0.0, 0.0))

    angles = np.degrees((w * steps[:, None]).sum(axis=0))
    print(f"\nGYRO-ONLY DRIFT after {t0[-1]:.0f} s")
    print(f"  angle turned about x {angles[0]:+.1f}°, y {angles[1]:+.1f}°, z {angles[2]:+.1f}°"
          "   (z ≈ heading when the board lies flat)")
    curves = [("real, raw gyro", t0, err_real, COLORS[0], "-"),
              ("sim, measured bias + noise", t_sim, err_sim, COLORS[1], "--"),
              ("real, minus its own mean", t0, err_debiased, COLORS[2], "-")]
    fig, (ax_lin, ax_log) = plt.subplots(1, 2, figsize=(12, 4.5))
    for label, tt, err, color, style in curves:
        p = growth_exponent(tt, err)
        print(f"  {label:<28} {err[-1]:8.2f}° total   growth t^{p:.2f}")
        ax_lin.plot(tt, err, color=color, ls=style, lw=1.5, label=f"{label} (p = {p:.2f})")
        ax_log.loglog(tt[1:], err[1:], color=color, ls=style, lw=1.5, label=label)
    ax_lin.set(xlabel="seconds since recording start", ylabel="orientation error (deg)", title="Linear scale")
    ax_log.set(xlabel="seconds since recording start", ylabel="orientation error (deg)",
               title="Log-log scale (slope = growth exponent p)")
    for ax in (ax_lin, ax_log):
        ax.grid(True, which="both", alpha=0.3)
    ax_lin.legend(frameon=False)
    fig.suptitle(f"{name}: gyro-only drift at rest, real vs. simulated")
    fig.tight_layout()
    fig.savefig(plots / f"{name}_drift.png", dpi=120)

    print(f"\nsaved {plots / f'{name}_bias.png'}\n      {plots / f'{name}_drift.png'}")
