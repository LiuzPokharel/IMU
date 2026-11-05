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

**Next session:** set up PlatformIO (board: Adafruit Feather ESP32-S3, Arduino framework) and write the I2C scan, so it's ready to run when parts arrive (~Nov 3–6).

## 2026-11-01
- Created the PlatformIO project in `firmware/` (board `adafruit_feather_esp32s3`, Adafruit BNO08x library). An empty sketch compiles.
- USB CDC is already on: the board definition sets `ARDUINO_USB_CDC_ON_BOOT=1`, so no extra build flag is needed.
- Wrote the I2C scan (`firmware/src/scan/main.cpp`): it tries addresses 0x08–0x77 and counts the ones that ACK. It compiles, but it's untested until the parts arrive.
- Wrote the first BNO085 sketch (`firmware/src/imu/main.cpp`): uncalibrated gyro + accelerometer at 100 Hz, printed as CSV rows `timestamp_us,sensor,x,y,z`. Gyro and accel arrive as separate reports, so each row holds one report. It compiles, but it's untested until the parts arrive.
- Wrote the live plotter (`host/live_plot.py`): `--fake` streams fake rows in the firmware's CSV format, or pass a serial port for the real board. Titles show each sensor's mean and measured rate. In fake mode the gyro mean reads `+0.0100, -0.0049, +0.0202`, matching the bias put in.

### Hardware day checklist
1. **Cable:** push the STEMMA QT cable fully into both the Feather and the BNO085. It should seat with a click. The breakout's power LED should light when the Feather is powered.
2. **First upload:** if the upload can't connect, put the board in bootloader mode: hold BOOT, tap RESET, release BOOT, then upload. Press RESET afterward to run the new code. The serial port can disappear and come back during this.
3. **I2C scan:** the `scan` sketch uploads by default. Open the serial monitor and expect `0x4A` (`0x4B` if the address jumper is bridged).
4. **If the scan finds nothing:** check the cable first, then the Feather's STEMMA QT power pin, GPIO 7 (`PIN_I2C_POWER`). The board's startup code already drives it HIGH, so add this at the top of `setup()` only to rule it out:
   ```cpp
   pinMode(PIN_I2C_POWER, OUTPUT);
   digitalWrite(PIN_I2C_POWER, HIGH);
   delay(10);
   ```
5. **After the scan passes:** switch to the `imu` environment in VS Code's bottom bar (or set `default_envs = imu`) to upload the IMU sketch. The serial monitor should show CSV rows.
6. **Live plot:** close the serial monitor first, because only one program can hold the port. Find the port with `ls /dev/cu.usb*`, then run `.venv/bin/python host/live_plot.py <port>`.
7. **Sanity moves:** at rest the gyro should be near 0 and the accelerometer should show about 9.8 on one axis. Rotating the board should spike the gyro, and tilting it should move gravity between accelerometer axes and leave it there. Flipping it over should flip the sign of z.

### Lessons
- Several bugs compiled cleanly: a hard-coded address, checking the wrong value, a counter outside the `if`, a missing `!`, and an extra zero in the 100 Hz interval. The compiler only catches syntax, so the logic needs a known-answer test, either an `assert` or the expected hardware output.
- One mistake can cause a whole list of compiler errors. Fix the first one and recompile.
- Changing the fake bias to 0.05 made the gyro plot *look* cleaner while making the drift about 5× worse. The noise hadn't changed; the axis just zoomed out. Read the numbers in the plot titles, not the look of the plot.

**Next session (hardware day, ~Nov 4):** work through the checklist above, then add CSV saving to the plotter for the Week 1 at-rest drift measurement.

## 2026-11-04: hardware day
- **I2C scan:** worked on the first upload and found 2 devices: the BNO085 plus (most likely) the Feather's onboard battery monitor. Addresses: _to fill in_.
- **PlatformIO only shows its toolbar** when the open folder has `platformio.ini` at its root, so open `firmware/` in its own VS Code window.
- **Port lock (`Errno 35`):** a serial monitor left open from the scan blocked the next one. Only one program can hold the port.
- **Broken timestamps:** every row printed about 4294964000 (just below 2³²) and didn't increase, so the live plot showed `-908182 Hz`. Root cause: the Adafruit BNO08x library never records the host time when it reads a report (`t_us` is never set in `Adafruit_BNO08x.cpp`), so `value.timestamp` is only the sensor's correction, "sample time minus read time": a few ms, negative. Fix: `micros() + (int32_t)value.timestamp`. Plain `micros()` would be 2–5 ms late, by a different amount on every row.
- **Measured rates (from timestamps):** gyro **100.5 Hz** (requested 100), accelerometer **~123 Hz**. The BNO085 picks the nearest rate it supports.
- **Accelerometer magnitude:** about 9.57 m/s² in one orientation and 9.98 in another, against 9.81 true, so roughly a 2–4% error that depends on orientation. The simulation didn't model this.
- **Gyro resolution:** readings move in steps of 0.001953 rad/s (1/512). At rest they stay within about ±0.008 rad/s.
- The scan found 0x4A on the first try.
- The timestamp bug: value.timestamp printed as about 2³² minus a few thousand, and the fix was switching to micros().
- Gravity measured at about 9.99 m/s², the board’s resting tilt, and anything you noticed in the live plot.


## 2026-11-05: prep for the at-rest drift measurement (done by Claude while I was in class)

### What changed
1. **Firmware (`firmware/src/imu/main.cpp`)**
   - Timestamps were already right. `t_us = micros() + (int32_t)value.timestamp` calls `micros()` once per event, right after `getSensorEvent()` returns, so gyro and accel rows each get their own sample time.
   - The `while (!Serial)` wait is now `while (!Serial && millis() < 3000) delay(10);`. Before, the board froze at power-on until a computer opened the port. Now it starts streaming after at most 3 s, which a "from power-on" warm-up recording needs. It compiles. **It needs a re-upload** (env `imu`).
2. **`host/record.py`** saves the stream to `data/<date>_<label>.csv` with a `t_us,sensor,x,y,z` header.
   - Options: `port`, `--seconds` (default 600), `--label` (required).
   - Prints progress every 30 s with row counts and rates. It saves only rows that parse, so a half row from connecting mid-stream never reaches the file. Anything else the board prints shows on screen.
   - It refuses to overwrite an existing file with the same name.
   - Tested against a loopback serial port.
   - The CSVs are gitignored (`*.csv`). 10 minutes is about 6 MB.
3. **`host/analyze_rest.py`** reads one recording, prints a report, and saves `data/plots/<name>_bias.png` and `<name>_drift.png`. It reports:
   - **Timing:** the typical and overall rate per sensor, gaps (a step over 1.5× the typical one), samples lost, the longest step, and any timestamps that went backwards. It also handles the `micros()` rollover every ~71.6 min.
   - **Gyro bias** (mean) **and noise** (std) per axis, in rad/s and deg/s. At rest the true rate is 0, so the mean *is* the bias.
   - **Bias over time:** a 30 s rolling mean, plotted against seconds since power-on. Averaging 3000 samples cancels the noise but keeps a slow warm-up change. It also prints the change from the first to the last 30 s.
   - **Accelerometer:** |a| against 9.81, and the tilt from level (the angle between gravity and the board's z axis).
   - **Gyro-only drift:** it integrates the raw gyro with `integrate()` from the sim, using each sample's real dt from the timestamps. The error angle is 2·arccos(|w|), the same formula as the sim, because at rest the true orientation never changes. It plots three curves on one figure (linear and log-log), each with a fitted growth exponent p:
     - the real raw gyro
     - **the sim run with this recording's own bias and noise**
     - the real gyro minus its own mean (hindsight, the best case for bias removal)
4. **Recording: skipped.** The board wasn't plugged in (no `/dev/cu.usbmodem*`). Even when it is, my sandbox can't open serial ports, so you run this part. See below.
5. **`sim/gyro_drift.py`:**
   - It takes `--bias x y z`, `--sigma`, `--rate x y z` (`0 0 0` = at rest), `--seconds`, and `--dt`.
   - The experiment moved under `if __name__ == "__main__":`, so `analyze_rest.py` can import the functions without rerunning everything.
   - `drift_error()` now returns `(t, err)`, and `integrate()` accepts an array of dt values for real timestamps.
   - With no options it reproduces the original exactly (69.2°, p = 0.97 / 0.46) and leaves `gyro_drift.png` byte-identical. Custom runs save to `gyro_drift_custom.png`.
   - **Fit fix:** an orientation error can't exceed 180°. Past ~90° the curve bends over and folds back, which made the fit report nonsense like p = 0.12. `growth_exponent()` now fits only the part before the error first reaches 90°.

### How it was checked (no board)
I made 10 minutes of fake data from `live_plot.py`'s generator, deleted 5 gyro rows, and shifted the timestamps so `micros()` rolls over at 300 s. The analysis got back exactly what went in:

| | put in | measured |
|---|---|---|
| gyro bias (rad/s) | +0.010, −0.005, +0.020 | +0.01001, −0.00503, +0.02003 |
| gyro noise (rad/s) | 0.010 | 0.01002, 0.01000, 0.01004 |
| rate | 100 Hz | 100.0 Hz |
| gaps | 1 gap, 5 samples | 1 gap, ~5 samples lost, 0 backwards |
| drift growth p | bias → 1 | real 1.01, sim 1.00 |

I deleted the fake plots afterwards, so `data/plots/` only ever holds real runs.

### To do when you're back: run the recording
1. Upload the `imu` env (for the 3 s timeout). Close the serial monitor and any `live_plot` windows.
2. Put the board somewhere it won't get bumped: flat on the desk, cable not pulling on it.
3. **Unplug USB, plug it back in** (that's the power-on), then immediately run:
   ```
   .venv/bin/python host/record.py /dev/cu.usbmodem101 --seconds 600 --label rest1
   ```
   Don't touch the board or the desk for 10 minutes.
4. ```
   .venv/bin/python host/analyze_rest.py data/2026-11-05_rest1.csv
   ```
   Change the date to the day you record.
5. Questions to answer from the output:
   - Is the bias around 0.002 rad/s or smaller? Readings were within ±0.008 at rest on hardware day.
   - Does the rolling mean settle after a few minutes (warm-up)?
   - Does the real drift curve follow the sim's dashed line? If it bends away, the bias isn't constant. That would be the warm-up, which the sim doesn't model.
   - How small does "minus its own mean" stay? That shows how much a one-time bias calibration at power-on would save.
   - Plug the numbers into `sim/gyro_drift.py --bias … --sigma … --rate 0 0 0 --seconds 600` for the README.

Still to fill in from hardware day: the two scan addresses.

## Next: complementary filter (roll and pitch)

Goal: roll and pitch that **don't drift** (unlike the gyro) and **aren't noisy or shaken around** (unlike the accelerometer). This is the plan and the math. The code is yours to write.

### The two sensors, and why neither works alone
- **Gyro → angle by integrating.** It's smooth and reacts instantly, but the bias integrates into an error that grows forever (analyzed above: p ≈ 1).
- **Accelerometer → angle from gravity.** It never drifts, because gravity always points down. But it's noisy, and any real acceleration (shaking, moving the board) looks like gravity tilting.
- The fix: trust the gyro over short times and the accelerometer over long times.

### Math 1: tilt from the accelerometer
At rest the accelerometer reads gravity's reaction, a vector of length g pointing **up**, written in the board's own axes. For roll φ (about x), then pitch θ (about y):

  a = g · [ −sin θ,  sin φ cos θ,  cos φ cos θ ]

Take ratios so g cancels:
- a_y / a_z = tan φ  →  **φ = atan2(a_y, a_z)**
- a_x / √(a_y² + a_z²) = −sin θ / cos θ  →  **θ = atan2(−a_x, √(a_y² + a_z²))**

Because these are ratios, the 2–4% magnitude error measured on hardware day mostly cancels: only differences *between* axes matter. Use `atan2`, not `atan`, so all four quadrants come out right. Yaw (heading) never appears in this vector, because spinning about gravity doesn't change what gravity looks like. So the accelerometer can't fix yaw, and yaw keeps drifting. Fixing it needs the magnetometer (Week 3).

### Math 2: angle rates from the gyro
The gyro gives body rates ω = (ω_x, ω_y, ω_z), not Euler angle rates. The exact conversion (ZYX angles) is:
- φ̇ = ω_x + sin φ · tan θ · ω_y + cos φ · tan θ · ω_z
- θ̇ = cos φ · ω_y − sin φ · ω_z

For small angles this is φ̇ ≈ ω_x and θ̇ ≈ ω_y. Start with the simple version, then switch to the exact one and compare at 45° of tilt. The `tan θ` term blows up at θ = ±90°: that's gimbal lock, and it's why the later filters use quaternions.

### Math 3: the blend
Every gyro sample, with dt taken from the timestamps:

  φ ← α · (φ + φ̇ · dt) + (1 − α) · φ_accel,    α = τ / (τ + dt)

- τ is the crossover time constant: gyro below τ, accelerometer above it. With τ = 1 s at 100 Hz, α ≈ 0.990.
- It's called "complementary" because the gyro path is high-passed and the accelerometer path low-passed with the same τ, so the two weights add to exactly 1. A steady real angle passes through unchanged.
- **Prediction you can test:** at rest, with gyro bias b, the angle settles at a fixed error instead of growing. Setting φ = α(φ + b·dt) gives (1−α)φ = α·b·dt, so **error = b·τ**. For example, b = 0.002 rad/s with τ = 1 s gives 0.002 rad = 0.11°, compared with ~69° of gyro-only drift over 10 min.

### Steps
1. Create `host/comp_filter.py` and reuse `load()` from `analyze_rest.py` to read a recording.
2. Write `accel_angles(a)` → (roll, pitch). Test with known answers before using real data:
   - [0, 0, 9.81] → (0°, 0°)
   - [0, 9.81, 0] → roll 90°
   - [−9.81, 0, 0] → pitch 90°
   - 2× any vector → same angles
3. Gyro-only angles, small-angle version first. On `rest1` they should drift at the bias rate, matching the x/y angles that `analyze_rest.py` prints.
4. The blend. Gyro (100 Hz) and accel (~123 Hz) arrive separately, so on each gyro sample use the **most recent** accel reading.
5. **Rest test:** run on `rest1` and check that the steady error ≈ b·τ (step 3's prediction), and that its noise is far below accel-only.
6. **Motion test:** record `tilt1` (`record.py --label tilt1 --seconds 60`): flat, tilt to ~45° and hold, rock it, shake it, set it back. Plot accel-only, gyro-only, and complementary on one figure.
7. **Sweep τ** (0.1, 1, 10 s) and explain the trade-off with numbers:
   - small τ: noisy, and shaking bleeds in
   - large τ: bias error b·τ, and slow to recover
8. Later: port the same few lines into the firmware so the board outputs angles itself.

### Pitfalls
- **Units:** the gyro is in rad/s and `atan2` returns radians. Convert to degrees only for printing and plots.
- **dt:** use real timestamp differences, not 0.01 (the accelerometer isn't at 100 Hz, and gaps happen).
- **Start-up:** initialize φ and θ from the first accel reading, not from 0, or the filter spends ~τ seconds converging.
- **Sign conventions:** check the board's axis arrows printed on the BNO085. Tilt the right side down and confirm roll goes the way you expect.
