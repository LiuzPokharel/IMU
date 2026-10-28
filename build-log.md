# Build Log

## 2026-10-27
- Set up the repo: README, build log, Python environment.
- Parts ordered and due ~Nov 3–6. Next: gyro-drift simulation in Python.

### Gyro-drift simulation (`sim/gyro_drift.py`)
- Gyro model: measured = true + bias + noise. Wrote the quaternion product (Hamilton) and checked it with i·j = k, j·i = −k.
- Integrated rates into a quaternion: dq = [cos(θ/2), sin(θ/2)·axis], q = q ⊗ dq, then normalize.
- Error angle = 2·arccos(|w|) of conj(q_true) ⊗ q_est.
- **Result: 69.3° of error after 60 s**, linear. Matches the z bias alone; the x/y bias cancels because the body frame is spinning.
- Hit a SyntaxError from putting `a = ...` assignments inside `np.array([...])`. A list holds values, not statements.

### Bias vs. noise experiment
- Ran the same sim three ways with identical noise: bias + noise, bias only, noise only. Fit err ∝ tᵖ on log-log axes to get the growth exponent.
- **Bias: 69.2° @ 60 s, p = 0.97 (linear). Noise: 0.73° RMS @ 60 s, p = 0.46 (~√t).** Bias is ~99% of the drift.
- Surprise: a single noise-only run gave p = 0.35, not 0.5. A random walk is one random path; √t describes the RMS over many runs. Averaging 20 seeds fixed it.
- Takeaway for week 1: measuring and subtracting the gyro bias at rest is the single biggest win.
- Also found: a perfect gyro gives ~1e-6° error, not 0, because arccos is very sensitive near 1.
