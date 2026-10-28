"""Gyro-drift simulation.

Simulate a gyro spinning at a known rate, corrupt the reading with bias + noise,
and (next step) integrate it into a quaternion to see how fast the orientation
estimate drifts away from the truth.
"""
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


def integrate(omega, dt):
    """Turn a stream of body-frame angular rates (N, 3) into orientations (N, 4).

    Each step the board rotates by angle |ω|·dt about axis ω/|ω|. We build that
    small rotation as a quaternion dq and chain it onto the current orientation.
    """
    N = len(omega)
    qs = np.zeros((N, 4))
    q = np.array([1.0, 0.0, 0.0, 0.0])   # identity: start with no rotation

    for n in range(N):
        w = omega[n]
        theta = np.linalg.norm(w) * dt   # how far we turned this step (rad)

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


# --- Timing --------------------------------------------------------------
dt = 0.01          # seconds between samples -> 100 Hz, same rate the real board will run at
T = 60.0           # total simulated time in seconds
t = np.arange(0, T, dt)   # time stamps: 0.00, 0.01, ..., 59.99
N = len(t)                # number of samples (6000)

# --- True motion ---------------------------------------------------------
# Angular velocity in rad/s about the body x, y, z axes.
# Here the board spins only about z at 0.5 rad/s (~28.6 deg/s).
# np.tile repeats the row N times -> shape (N, 3), one row per timestep.
omega_true = np.tile([0.0, 0.0, 0.5], (N, 1))

# --- Gyro error model ----------------------------------------------------
# A real gyro reports: measured = true + bias + noise
#   bias  : constant offset per axis (rad/s). Integrates into error that grows linearly with time.
#   noise : random every sample (white noise, std = sigma). Integrates into a random walk (~sqrt(t)).
bias = np.array([0.01, -0.005, 0.02])   # rad/s
sigma = 0.01                            # rad/s, noise standard deviation

rng = np.random.default_rng(0)          # fixed seed -> identical noise every run, so comparisons are fair
noise = rng.normal(0.0, sigma, size=(N, 3))

omega_meas = omega_true + bias + noise  # what the gyro actually hands us

# --- Integrate both into orientations -----------------------------------
q_true = integrate(omega_true, dt)   # ground truth
q_est = integrate(omega_meas, dt)    # what we'd believe using only the gyro

# --- Sanity checks -------------------------------------------------------
# Noise averages out to ~0 over 6000 samples, so the mean error should be ~= bias.
mean_err = (omega_meas - omega_true).mean(axis=0)
print("mean measured error:", mean_err)
print("true bias:          ", bias)
assert np.allclose(mean_err, bias, atol=1e-3), "mean error should match the bias"

# Quaternion product rules: i·j = k, j·i = -k (order matters!)
i, j, k = np.array([0, 1, 0, 0]), np.array([0, 0, 1, 0]), np.array([0, 0, 0, 1])
assert np.allclose(quat_mult(i, j), k)
assert np.allclose(quat_mult(j, i), -k)

# 60 s at 0.5 rad/s about z = 30 rad total -> q = [cos(15), 0, 0, sin(15)] (half-angle again)
assert np.allclose(q_true[-1], [np.cos(15), 0, 0, np.sin(15)], atol=1e-6)
print("final q_true:", q_true[-1])
print("final q_est: ", q_est[-1])
