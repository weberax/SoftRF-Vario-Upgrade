# Vario Upgrade — Status Tracker

Living document. Update this whenever a phase lands, so it's always clear what's
done vs. what's next. See `notes.md` for build/flash/monitor commands.

## Goal

Port the variometer functionality of a long-used reference project
(`~/playground/Vario/DIY-Software-Mai-2019`, a GNUVario/prunkdump fork) into the
SoftRF T-Echo (nRF52) firmware: GPS + BMP280 baro + MPU9250 IMU, Kalman-filtered
sensor fusion, piezo buzzer with real climb/sink beep patterns. Reuse working
code/libraries from the reference project wherever possible rather than
reinventing it.

**Hardware status:** MPU9250 and the external piezo buzzer are not wired up yet.
GPS + onboard BMP280 are available today. Software is being built ahead of the
hardware so it's ready to test as soon as it lands.

## Status

### Done

- **Kalman engine** (`driver/Vario.cpp`, vendored `libraries/KalmanVario` —
  a direct port of the reference project's `kalmanvert`): 100 Hz loop fusing
  BMP280 altitude + MPU9250 vertical acceleration.
- **Kalman output wired to display and telemetry**: the EPD vario page and the
  LK8EX1 Bluetooth sentence both read `Vario_getVario()`/`Vario_getAlt()` now,
  not raw GPS/baro values.
- **Sample-rate fixes**: baro cache now actually refreshes at ~25 Hz (matching
  the BMP280 fast-mode config), and the MPU9250 is updated from the Kalman
  loop's own 10 ms cadence instead of a stray 500 ms G-load-page timer.
- **One-shot GPS-altitude calibration**: `Vario_calibrateAlt()` is now called
  automatically once a GPS fix has been valid for 5 s.
- **Buzzer pin ownership**: stock SoftRF alarms/jingles (`Sound_tone()`) are
  routed through `PiezoBeeper` instead of a second PWM peripheral on the same
  pin.
- **IMU calibration**: `Vario_calibrateIMU()` (PAD double-click) runs the
  vendored MPU9250 library's own `calibrateAccelGyro()` and persists the
  result to EEPROM (`settings->imu_accel_bias`/`imu_gyro_bias`), reapplied via
  `setAccBias()`/`setGyroBias()` on every boot.
- **Continuous accel-bias auto-correction**: every time the device is detected
  still for >2 s, `accel_bias_z` gets a small low-pass nudge (not just once at
  boot), so slow in-session drift keeps getting tracked.
- **Real beep patterns**: restored `libraries/GNUVarioBeeper` (the reference
  project's `beeper.cpp`, already retargeted to `PiezoBeeper_setFreq()`) —
  climb/sink/near-climb-blip cadence, climb-rate-dependent beep speed-up,
  thresholds/volume matching the reference project's tuned settings
  (sink −4.0 m/s, climb 0.1 m/s, near-climb sensitivity 3.5 m/s, volume 6/10).

### Next

- **On-device testing once MPU9250 + buzzer are wired up**: verify the beep
  pattern feels right, verify calibration (PAD double-click) actually
  improves things, verify no pin/PWM conflicts in practice.
- **EPD Up/Down secondary vario screen**: `EPD_vario_next()`/`_prev()` are
  still no-ops. Not blocking.

### Deferred

- **Total energy compensation (TEC)**: revisit once Kalman+beeper+calibration
  are flying and validated on real hardware.
- **Magnetometer**: current IMU path is accel+gyro only (vertical-accel/Kalman
  doesn't need yaw). Revisit later — worth checking whether adding mag
  (calibrated via the vendored MPU9250 library's own `calibrateMag()`)
  measurably improves Kalman tilt accuracy or is useful for TEC.

## Architecture

```
Baro (BMP280, ~25Hz cache)  ─┐
                              ├─► Vario.cpp: kalmanvert (100Hz) ─► kalman_alt, kalman_vs
MPU9250 (accel+quaternion)  ─┘                                        │
                                                                       ├─► ui/Vario_EPD.cpp (display)
                                                                       ├─► protocol/data/NMEA.cpp (LK8EX1 / BT)
                                                                       └─► GNUVarioBeeper (beeper.cpp) ─► PiezoBeeper (nRF52 PWM) ─► piezo
```

Calibration:
- **Continuous**: `update_accel_bias()` in `Vario.cpp`, runs automatically,
  no user action.
- **Full**: PAD double-click → `Vario_calibrateIMU()` → MPU9250 library's
  `calibrateAccelGyro()` → persisted to EEPROM.
- **Altitude**: one-shot GPS-altitude sync, automatic once a fix is stable.

## Key files

| File | Role |
|---|---|
| `driver/Vario.cpp`/`.h` | Kalman engine, calibration, beeper driving |
| `driver/PiezoBeeper.cpp`/`.h` | nRF52 differential-PWM buzzer HAL |
| `libraries/GNUVarioBeeper/beeper.cpp`/`.h` | Climb/sink/near-climb beep pattern state machine |
| `libraries/KalmanVario/kalmanvert.cpp`/`.h` | 2-state (position/velocity) Kalman filter |
| `libraries/MPU9250/MPU9250.h` | IMU driver + Madgwick fusion + calibration |
| `driver/Baro.cpp`/`.h` | BMP280 read/cache |
| `driver/EEPROM.h`/`.cpp` | Settings persistence, incl. IMU calibration bias |
| `ui/Vario_EPD.cpp` | EPD vario page |
| `protocol/data/NMEA.cpp` | LK8EX1 Bluetooth vario sentence |

## Testing checklist

- [x] Firmware compiles without errors
- [ ] Firmware flashes to T-Echo without errors
- [ ] T-Echo boots and displays VARIO page with Kalman-fused VAR/CLB values
- [ ] LK8EX1 sentence over Bluetooth shows plausible values in XCTrack
- [ ] GPS-altitude calibration kicks in after a stable fix (serial log)
- [ ] MPU9250 wired up: vertical-accel input visibly speeds up vario response
- [ ] PAD double-click runs full IMU calibration (serial log + saved on reboot)
- [ ] Piezo buzzer wired up: climb/sink/near-climb beep patterns sound right
- [ ] Long-press PAD mutes/unmutes the beeper
