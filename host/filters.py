"""Orientation filters: three ways to blend the gyro (smooth, drifts) with the accelerometer (noisy, never drifts).

  comp_step      complementary filter on roll and pitch (Euler angles)
  tilt_step      quaternion gyro integration, nudged toward the accelerometer's "up" (Mahony-style)
  madgwick_step  Madgwick's filter: the same idea, with the nudge found by gradient descent

Each step function takes the current state plus one gyro sample and returns the new state.
Conventions, same as sim/gyro_drift.py: q = [w, x, y, z] turns board-frame vectors into world-frame
ones (v_world = q ⊗ v ⊗ q*), world z points up, angles are radians, gyro rad/s, accel m/s².

  .venv/bin/python host/filters.py     # runs the checks at the bottom
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim"))
from gyro_drift import conj, quat_mult  # noqa: E402  (needs the path above)

G = 9.81


def unit_up(a):
    """The accelerometer reading as a unit "up" vector in board axes, or None if there is no reading.

    Tried and removed: ignoring samples whose length is far from 9.81 (a shaken board). It made the shake
    error about 3x WORSE. During a shake the length also passes through 9.81 when the reading is gravity
    mirrored about the shake axis, so the samples that got through were the misleading ones.
    """
    mag = np.linalg.norm(a)
    return a / mag if mag > 0 else None


# --- Angles and "up" vectors -------------------------------------------------

def accel_angles(a):
    """(roll, pitch) from gravity alone. At rest a = g·[-sin θ, sin φ cos θ, cos φ cos θ]; ratios cancel g."""
    return np.arctan2(a[1], a[2]), np.arctan2(-a[0], np.hypot(a[1], a[2]))


def up_from_angles(roll, pitch):
    """Where world "up" points in board axes for this roll and pitch. Yaw doesn't appear: gravity can't see it."""
    return np.array([-np.sin(pitch), np.sin(roll) * np.cos(pitch), np.cos(roll) * np.cos(pitch)])


def up_from_quat(q):
    """World "up" in board axes: q* ⊗ [0, 0, 1] ⊗ q, which is the bottom row of q's rotation matrix."""
    w, x, y, z = q
    return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def euler(q):
    """(roll, pitch, yaw) of a quaternion, ZYX order: the same roll and pitch accel_angles() gives."""
    w, x, y, z = q
    return (np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            np.arcsin(np.clip(2 * (w * y - x * z), -1, 1)),
            np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def quat_from_euler(roll, pitch, yaw=0.0):
    """Yaw about z, then pitch about y, then roll about x."""
    qz = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    qy = np.array([np.cos(pitch / 2), 0, np.sin(pitch / 2), 0])
    qx = np.array([np.cos(roll / 2), np.sin(roll / 2), 0, 0])
    return quat_mult(quat_mult(qz, qy), qx)


def angle_between(u, v):
    """Angle in radians between two vectors."""
    return np.arccos(np.clip(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v)), -1, 1))


def rotate_by(q, w, dt):
    """Turn orientation q by body rate w for dt seconds. Same small-rotation step as integrate() in the sim."""
    theta = np.linalg.norm(w) * dt
    if theta > 0:
        q = quat_mult(q, np.concatenate([[np.cos(theta / 2)], np.sin(theta / 2) * w / np.linalg.norm(w)]))
    return q / np.linalg.norm(q)


# --- The filters ---------------------------------------------------------------

def comp_step(angles, w, a, dt, tau=1.0):
    """Complementary filter. angles = (roll, pitch).

    Predict with the gyro, then pull a fraction (1 - alpha) of the way to the accelerometer's angles.
    alpha = tau / (tau + dt): the gyro is trusted for changes faster than tau, the accelerometer for slower.
    """
    roll, pitch = angles
    # Body rates are not Euler-angle rates once the board is tilted; this is the exact conversion.
    # tan(pitch) blows up at ±90°: gimbal lock, the reason the other two filters use quaternions.
    roll_rate = w[0] + np.sin(roll) * np.tan(pitch) * w[1] + np.cos(roll) * np.tan(pitch) * w[2]
    pitch_rate = np.cos(roll) * w[1] - np.sin(roll) * w[2]
    roll, pitch = roll + roll_rate * dt, pitch + pitch_rate * dt

    if unit_up(a) is not None:
        alpha = tau / (tau + dt)
        a_roll, a_pitch = accel_angles(a)
        # Blend along the short way round, so 179° and -179° average to 180°, not 0°.
        roll += (1 - alpha) * ((a_roll - roll + np.pi) % (2 * np.pi) - np.pi)
        pitch += (1 - alpha) * (a_pitch - pitch)
    return roll, pitch


def tilt_step(q, w, a, dt, kp=1.0):
    """Quaternion integration with accelerometer correction (the proportional part of Mahony's filter).

    The cross product of measured "up" and predicted "up" is a rotation axis whose length is the sine
    of the angle between them. Adding kp times it to the gyro rate turns the estimate toward the
    accelerometer at kp rad/s per radian of error, so 1/kp plays the role of tau above.
    """
    up = unit_up(a)
    if up is not None:
        w = w + kp * np.cross(up, up_from_quat(q))
    return rotate_by(q, w, dt)


def madgwick_step(q, w, a, dt, beta=0.1):
    """Madgwick's filter, gyro + accelerometer version.

    Error to shrink: f(q) = (where q says "up" is) - (where the accelerometer says it is).
    One gradient-descent step per sample: the gradient of ½|f|² is Jᵀf, with J the Jacobian of f.
    Only its direction is used, so the correction always turns q at beta rad/s. beta should be about
    the size of the gyro error you expect (rad/s): big enough to cancel it, no bigger.
    """
    qw, qx, qy, qz = q
    q_dot = 0.5 * quat_mult(q, np.concatenate([[0.0], w]))   # what the gyro says: q̇ = ½ q ⊗ [0, ω]
    up = unit_up(a)
    if up is not None:
        f = up_from_quat(q) - up
        J = np.array([[-2 * qy, 2 * qz, -2 * qw, 2 * qx],
                      [2 * qx, 2 * qw, 2 * qz, 2 * qy],
                      [0, -4 * qx, -4 * qy, 0]])
        grad = J.T @ f
        if np.linalg.norm(grad) > 0:
            q_dot = q_dot - beta * grad / np.linalg.norm(grad)
    q = q + q_dot * dt
    return q / np.linalg.norm(q)


# --- Sanity checks -------------------------------------------------------
_deg = np.degrees
assert np.allclose(_deg(accel_angles(np.array([0, 0, G]))), [0, 0])            # flat
assert np.allclose(_deg(accel_angles(np.array([0, G, 0]))), [90, 0])           # rolled 90°
assert np.allclose(_deg(accel_angles(np.array([-G, 0, 0]))), [0, 90])          # pitched 90°
assert np.allclose(accel_angles(np.array([1.0, 2, 3])), accel_angles(np.array([2.0, 4, 6])))   # length cancels

# Angles -> quaternion -> angles is a round trip, and both "up" formulas agree.
_q = quat_from_euler(0.3, -0.5, 1.2)
assert np.allclose(euler(_q), [0.3, -0.5, 1.2])
assert np.allclose(up_from_quat(_q), up_from_angles(0.3, -0.5))

# A perfect gyro with no accelerometer help: rolling at 0.5 rad/s for 1 s gives 0.5 rad of roll in all three.
_w, _none = np.array([0.5, 0.0, 0.0]), np.zeros(3)   # a zero accel reading is ignored by unit_up
_ang, _qt, _qm = (0.0, 0.0), np.array([1.0, 0, 0, 0]), np.array([1.0, 0, 0, 0])
for _ in range(100):
    _ang, _qt, _qm = comp_step(_ang, _w, _none, 0.01), tilt_step(_qt, _w, _none, 0.01), madgwick_step(_qm, _w, _none, 0.01)
assert np.allclose([_ang[0], euler(_qt)[0], euler(_qm)[0]], 0.5, atol=1e-4)

# Started 30° wrong and held still, each filter comes back to what the accelerometer shows.
_a = G * up_from_angles(0.2, -0.1)
_ang, _qt, _qm = (0.2 + 0.5, -0.1), quat_from_euler(0.2 + 0.5, -0.1), quat_from_euler(0.2 + 0.5, -0.1)
for _ in range(2000):   # 20 s
    _ang = comp_step(_ang, np.zeros(3), _a, 0.01)
    _qt, _qm = tilt_step(_qt, np.zeros(3), _a, 0.01), madgwick_step(_qm, np.zeros(3), _a, 0.01)
for _up in (up_from_angles(*_ang), up_from_quat(_qt), up_from_quat(_qm)):
    assert _deg(angle_between(_up, _a)) < 0.1

# The build log's prediction: at rest with gyro bias b, the complementary filter settles at error b·tau.
_ang = (0.0, 0.0)
for _ in range(3000):
    _ang = comp_step(_ang, np.array([0.002, 0, 0]), np.array([0, 0, G]), 0.01, tau=1.0)
assert np.isclose(_ang[0], 0.002 * 1.0, rtol=0.02)

if __name__ == "__main__":
    print("filters.py: all checks passed")
