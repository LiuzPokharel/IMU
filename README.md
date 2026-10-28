# IMU Orientation Tracker

Estimating 3D orientation from a 9-DOF IMU with filters I wrote myself (complementary → quaternion integration → Madgwick), then measuring how my estimate compares to the BNO085's onboard sensor fusion.

## Hardware
- Adafruit ESP32-S3 Feather (4MB Flash / 2MB PSRAM)
- Adafruit BNO085 9-DOF IMU, connected over I2C via STEMMA QT

## Software
- **Firmware:** C++ / Arduino framework, built with PlatformIO
- **Host:** Python (numpy, matplotlib, pyserial) for logging, filtering, and plots

## Roadmap
- [ ] Gyro-drift simulation (Python): integrate a noisy, biased gyro into a quaternion and plot how the error grows
- [ ] Raw data: I2C scan, read gyro + accel at ~100 Hz, stream over serial, live plot
- [ ] Complementary filter for roll/pitch
- [ ] Quaternion gyro integration with accelerometer correction
- [ ] Madgwick filter + live 3D visualization
- [ ] Compare against the BNO085's onboard fusion (error in degrees, drift over 5 minutes)

## Results
_Coming soon._

## Setup
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Progress notes for each session are in [build-log.md](build-log.md).
