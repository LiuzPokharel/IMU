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

