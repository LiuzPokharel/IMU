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
5. **After the scan passes:** switch to the `imu` environment in VS Code's bottom bar (or set `default_envs = imu`) to upload the IMU sketch.
