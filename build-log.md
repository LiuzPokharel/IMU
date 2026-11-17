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
- Gravity measured at about 9.99 m/s², the board’s resting tilt.

## 2026-11-15: filters, comparison, 3D view (no board connected)
Written with Claude in one session, checked on simulated data only. Nothing here has run on the board yet.

- **Firmware (`imu`):** also streams the BNO085's own orientation (`SH2_GAME_ROTATION_VECTOR`, gyro + accel, no magnetometer) as `t,q,w,x,y,z` rows. It compiles; not uploaded.
- **`host/filters.py`:** complementary filter (roll, pitch), quaternion integration with accelerometer correction, and Madgwick. Checks at the bottom: known accelerometer angles, a perfect gyro gives the right angle, a 30° wrong start converges back, and the complementary filter at rest settles at error = bias × τ.
- **`host/compare.py`:** runs every method over a recording and scores it against the `q` rows. `--fake` makes 60 s of motion with a known true orientation.
- **`host/view3d.py`:** live 3D box following Madgwick, with the BNO085 as a dashed outline and the difference in degrees.
- `record.py`, `live_plot.py` and `analyze_rest.py` accept the new `q` rows.

### Results (simulated, 60 s, tilt error RMS)
accel only 8.51°, gyro only 0.44°, complementary 1.69°, gyro + accel correction 1.55°, Madgwick 0.31°.
- During the shake: accel only 22°, complementary 4.3°, Madgwick 0.5°.
- 5 minutes with the gyro bias left in: gyro only reaches 91° of tilt error, Madgwick 0.44° RMS. Heading drifts 56° in both, because nothing corrects it.

### Lesson: a "sensible" fix that made it worse
I first ignored accelerometer samples whose length was more than 15% from 9.81 (a shaken board). The shake error went from 4.3° to 12.7°. During a shake the length also passes through 9.81 when the reading is gravity mirrored about the shake axis, so the filter kept exactly the misleading samples. Removed it. Measure before keeping a fix.

### To do on the board
1. Upload `imu`. In the serial monitor, check that `q` rows appear.
2. Record 5 minutes: `record.py <port> --seconds 300 --label move1`. Keep it still for the first 3 s, then tilt, turn and shake it, and set it down for the last minute.
3. `compare.py data/<date>_move1.csv`. If it prints "inverting", the BNO085's quaternion is world-to-board; the script handles it.
4. `view3d.py <port>`: tilt the right edge down and check the box rolls the same way. If an axis is mirrored, the board's axes differ from what the code assumes.
5. Put the real table in the README, and record the demo video from `view3d.py`.
6. Still open from before: the 10-minute at-rest recording and the two I2C scan addresses.

### Not done
- **Magnetometer.** Without it heading drifts. Adding it means the 9-axis Madgwick and a magnetometer calibration.
- **Filter on the board.** The filters run in Python; the ESP32 only streams.
- **Understanding it.** I should re-derive the three filters by hand (the math is in the comments of `filters.py`) before explaining them in an interview.

## 2026-11-16: first real recordings
- New firmware uploaded; `q` rows arrive. `view3d.py` follows the board.
- **10 minutes at rest (`rest1`):**
  - Gyro bias: +0.0019, −0.0027, +0.0006 rad/s (0.11, −0.15, 0.04 deg/s). Noise 0.002–0.003 rad/s per axis.
  - The bias is steady: it changed by under 0.02 deg/s between the first and last 30 s, so there is no real warm-up drift. One small step in y and z about 28 s after power-on.
  - Gyro-only drift: **115.8° after 10 minutes**, growing as t^1.02. The simulation fed this run's own bias and noise gives 115.6°. Subtracting the mean bias leaves **0.43°**.
  - Accelerometer reads 9.44 m/s², 3.8% under 9.81. The board sat tilted 9.1° (the cable lifts one end).
- **Problem found: the gyro only arrived at 48 Hz, not 100.** Gyro 48 Hz, orientation 52 Hz, accel 126 Hz. Adding the third report filled the I2C bus at its default 100 kHz. Fix: `Wire.setClock(400000)` after `begin_I2C()`. Compiles; needs uploading and a check that `record.py` shows about 100 Hz for the gyro.
- `view3d.py` now shows how many rows have arrived and prints messages from the board.
- **I2C fix confirmed:** after uploading, the gyro arrives at 99.8 Hz, orientation 99.8 Hz, accel 126 Hz.
- **`move1` (5 min) turned out to be another at-rest run:** `record.py` printed "don't touch the board", so the board never moved (roll −2.5°, pitch 8.5° throughout). Message fixed. Still useful as a rest test against the BNO085's own fusion, gyro bias measured over the first 2 s and removed:
  - Tilt error RMS: complementary 0.03°, gyro + accel correction 0.03°, Madgwick 0.13°, accel only 0.17°, gyro only 2.77° (4.4° by the end).
  - Heading drifted 2.6° away from the BNO085's in 5 minutes.
  - A 2 s bias measurement is not enough for gyro-only: it still drifts about 1°/min. The filters remove that for tilt.
- Still needed: a real movement recording (`move2`).
- **Mount (`cad/`):** `mount.py` writes `mount.stl`, a plate 88.2 × 28.9 × 2.4 mm with eight 5 mm posts for M2.5 screws, holding the Feather and the BNO085 side by side. Hole positions follow Adafruit's standard layout (0.1 inch in from each edge); not yet measured against the boards or printed. The slicer makes the G-code, on the printer owner's machine.
- **Demo video (`demo/imu-demo.mov`):** the board in hand with `view3d.py` following it. iPhone videos carry the GPS location of where they were filmed, and trimming in QuickTime keeps it. The committed copy has the coordinates blanked; the originals are gitignored.

### Lessons
- **Adding a sensor report can slow the others.** Three reports at 100 Hz didn't fit through I2C at 100 kHz, and the gyro silently dropped to 48 Hz. Nothing errored; only the timestamps showed it. Check the measured rate after every firmware change.
- **A tool's own message can spoil a measurement.** `record.py` said "don't touch the board" on a run that was meant to have movement.
- **Check a phone video's metadata before publishing it.**

### Next
1. Record `move2` with real movement (still for 3 s, then tilt, turn and shake, then set down) and run `compare.py` on it.
2. Put the real results table in the README.
3. Measure the boards' hole spacing, then print the mount.
4. Fill in the two I2C scan addresses from hardware day.
