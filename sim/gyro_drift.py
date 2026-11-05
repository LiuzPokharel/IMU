"""Gyro-drift simulation.

Simulate a gyro spinning at a known rate, corrupt the reading with bias + noise,
and integrate it into a quaternion to see how fast the orientation estimate
drifts away from the truth.

  .venv/bin/python sim/gyro_drift.py                       # the original experiment
  .venv/bin/python sim/gyro_drift.py --bias 0.001 -0.002 0.0005 --sigma 0.003 --rate 0 0 0 --seconds 600
                                                           # plug in measured numbers, board at rest
host/analyze_rest.py imports drift_error() from here to put sim and real drift on one plot.
"""
import argparse

import numpy as np


def quat_mult(q, r):
    """Hamilton product q ⊗ r. Quaternions are [w, x, y, z] = a + b·i + c·j + d·k."""
    a1, b1, c1, d1 = q
    a2, b2, c2, d2 = r

    a = a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2
    b = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2
    c = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2
    d = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2

    return np.array([a, b, c, d])


def conj(q):
    """Conjugate [w, -x, -y, -z]. For a unit quaternion this is the inverse rotation."""
    return q * np.array([1, -1, -1, -1])


def integrate(omega, dt):
    """Turn a stream of body-frame angular rates (N, 3) into orientations (N, 4).

    Each step the board rotates by angle |ω|·dt about axis ω/|ω|. We build that
    small rotation as a quaternion dq and chain it onto the current orientation.
    dt is one number (fixed rate) or an array of N step lengths (real timestamps).
    """
    N = len(omega)
    dt = np.broadcast_to(dt, (N,))
    qs = np.zeros((N, 4))
    q = np.array([1.0, 0.0, 0.0, 0.0])   # identity: start with no rotation

    for n in range(N):
        w = omega[n]
        theta = np.linalg.norm(w) * dt[n]   # how far we turned this step (rad)

        if theta > 0:
            axis = w / np.linalg.norm(w)  # unit vector we turned about
            # Half-angle: q rotates a vector via q·v·q*, so the angle gets applied twice.
            dq = np.concatenate([[np.cos(theta / 2)], np.sin(theta / 2) * axis])
        else:
            dq = np.array([1.0, 0.0, 0.0, 0.0])  # no rotation (avoids dividing by 0)

        # dq goes on the RIGHT because ω is measured in the board's own (body) frame.
        q = quat_mult(q, dq)
        # Floating-point error slowly pushes |q| away from 1; re-normalize so it stays a pure rotation.
        q = q / np.linalg.norm(q)
        qs[n] = q

    return qs


def drift_error(bias, sigma, seed=0, T=60.0, dt=0.01, rate=(0.0, 0.0, 0.5)):
    """Simulate a gyro with this bias + noise, integrate it, and return (t, error angle in deg at every step).

    T    : total simulated time in seconds
    dt   : seconds between samples (0.01 -> 100 Hz, the real board's rate)
    rate : the TRUE angular velocity in rad/s about body x, y, z. Default: spinning about z at
           0.5 rad/s (~28.6 deg/s). Use (0, 0, 0) to match a board sitting still.

    A real gyro reports: measured = true + bias + noise
      bias  : constant offset per axis (rad/s). Integrates into error that grows linearly with time.
      noise : random every sample (white noise, std = sigma). Integrates into a random walk (~sqrt(t)).
    """
    t = np.arange(0, T, dt)   # time stamps: 0.00, 0.01, ...
    N = len(t)
    # np.tile repeats the row N times -> shape (N, 3), one row per timestep.
    omega_true = np.tile(rate, (N, 1))
    q_true = integrate(omega_true, dt)   # ground truth orientation

    rng = np.random.default_rng(seed)   # same seed -> same noise, so cases are compared fairly
    noise = rng.normal(0.0, sigma, size=(N, 3))
    omega_meas = omega_true + bias + noise   # what the gyro actually hands us
    q_est = integrate(omega_meas, dt)        # what we'd believe using only the gyro

    # q_err = conj(q_true) ⊗ q_est is the rotation that takes the TRUE orientation to the ESTIMATE.
    # If they matched, q_err = [1, 0, 0, 0]. Its w part is cos(angle/2), so angle = 2·arccos(|w|).
    #   abs  : q and -q are the same rotation; abs picks the short way round (0..180°).
    #   clip : rounding can give |w| = 1.0000000002, and arccos(>1) = nan.
    err_deg = np.zeros(N)
    for n in range(N):
        q_err = quat_mult(conj(q_true[n]), q_est[n])
        err_deg[n] = np.degrees(2 * np.arccos(np.clip(np.abs(q_err[0]), -1, 1)))
    return t, err_deg


def growth_exponent(t, err):
    """Fit err ≈ C·t^p and return p.  Taking logs: log(err) = log(C) + p·log(t), a straight line with slope p.
    p ≈ 1 means linear growth (bias), p ≈ 0.5 means sqrt(t) growth (random walk from noise)."""
    # An orientation error can't exceed 180°: past ~90° it bends over and folds back, so t^p stops fitting.
    # Fit only the part before the error first reaches 90°.
    end = np.argmax(err > 90) if (err > 90).any() else len(err)
    t, err = t[:end], err[:end]
    keep = (t > 0) & (err > 0)   # log(0) is -inf: skip t = 0 and any exact-zero error
    p, _ = np.polyfit(np.log(t[keep]), np.log(err[keep]), 1)
    return p


# --- Sanity checks -------------------------------------------------------
# Quaternion product rules: i·j = k, j·i = -k (order matters!)
i, j, k = np.array([0, 1, 0, 0]), np.array([0, 0, 1, 0]), np.array([0, 0, 0, 1])
assert np.allclose(quat_mult(i, j), k)
assert np.allclose(quat_mult(j, i), -k)

# 60 s at 0.5 rad/s about z = 30 rad total -> q = [cos(15), 0, 0, sin(15)] (half-angle again)
assert np.allclose(integrate(np.tile([0.0, 0.0, 0.5], (6000, 1)), 0.01)[-1],
                   [np.cos(15), 0, 0, np.sin(15)], atol=1e-6)

# A perfect gyro (no bias, no noise) should give ~zero error.
# (Not exactly 0: arccos is super sensitive near 1, so float rounding shows up as ~1e-6 deg.)
assert drift_error(np.zeros(3), 0.0)[1].max() < 1e-4


if __name__ == "__main__":
    # --- Experiment: which error source causes the drift? -------------------
    parser = argparse.ArgumentParser(description="Simulate gyro-only drift. Defaults are the original experiment.")
    parser.add_argument("--bias", type=float, nargs=3, default=[0.01, -0.005, 0.02], help="gyro bias x y z, rad/s")
    parser.add_argument("--sigma", type=float, default=0.01, help="gyro noise std per sample, rad/s")
    parser.add_argument("--rate", type=float, nargs=3, default=[0.0, 0.0, 0.5], help="true rotation x y z, rad/s (0 0 0 = at rest)")
    parser.add_argument("--seconds", type=float, default=60.0, help="how long to simulate")
    parser.add_argument("--dt", type=float, default=0.01, help="seconds between samples")
    args = parser.parse_args()

    bias = np.array(args.bias)   # rad/s
    sigma = args.sigma           # rad/s, noise standard deviation
    sim = dict(T=args.seconds, dt=args.dt, rate=args.rate)

    cases = {
        "bias + noise": drift_error(bias, sigma, **sim),
        "bias only":    drift_error(bias, 0.0, **sim),
        "noise only":   drift_error(np.zeros(3), sigma, **sim),
    }
    # One noise run is just ONE random path, so its growth wobbles. The sqrt(t) law is about the
    # typical size across many runs, so also take the RMS (root-mean-square) error over 20 seeds.
    t = cases["noise only"][0]
    runs = np.array([drift_error(np.zeros(3), sigma, seed=s, **sim)[1] for s in range(20)])
    cases["noise only, RMS of 20"] = (t, np.sqrt((runs ** 2).mean(axis=0)))

    print(f"{'case':<22}{f'error @ {args.seconds:g} s':>16}{'growth t^p':>12}")
    for name, (t, err) in cases.items():
        print(f"{name:<22}{err[-1]:>13.2f} deg{growth_exponent(t, err):>10.2f}")

    # Bias error grows ~linearly (p ≈ 1); noise error grows ~sqrt(t) on average (p ≈ 0.5).
    # Only checked for the original numbers: other settings (e.g. a tiny bias) can legitimately differ.
    if args == parser.parse_args([]):
        assert 0.9 < growth_exponent(*cases["bias only"]) < 1.1
        assert 0.4 < growth_exponent(*cases["noise only, RMS of 20"]) < 0.6

    # --- Plot ----------------------------------------------------------------
    import matplotlib.pyplot as plt

    fig, (ax_lin, ax_log) = plt.subplots(1, 2, figsize=(12, 4.5))
    for name, (t, err) in cases.items():
        ax_lin.plot(t, err, label=name)
        ax_log.loglog(t[1:], err[1:], label=name)

    # Left: normal axes. Shows bias dominates; noise-only hugs zero.
    ax_lin.set(xlabel="time (s)", ylabel="orientation error (deg)", title="Linear scale")
    # Right: log-log axes turn t^p into a straight line with slope p, so you can SEE linear vs sqrt growth.
    ax_log.set(xlabel="time (s)", ylabel="orientation error (deg)", title="Log-log scale (slope = growth exponent)")
    for ax in (ax_lin, ax_log):
        ax.grid(True, which="both", alpha=0.4)
        ax.legend()

    fig.suptitle(f"Gyro-only drift: bias = {bias} rad/s, σ = {sigma} rad/s, {1 / args.dt:g} Hz")
    fig.tight_layout()
    # Saved so it can go in the README. Only the original experiment overwrites that picture.
    out = "sim/gyro_drift.png" if args == parser.parse_args([]) else "sim/gyro_drift_custom.png"
    fig.savefig(out, dpi=120)
    print(f"saved {out}")
    plt.show()
