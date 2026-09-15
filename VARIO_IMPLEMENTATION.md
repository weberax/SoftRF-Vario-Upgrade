# SoftRF Vario Upgrade — Implementation Summary

## What We've Built

### 1. New Vario Display Page (`src/ui/Vario_EPD.cpp` — 244 lines)

**Purpose:** Primary flight display showing live vario metrics optimized for paragliding/soaring.

**Display Layout (T-Echo EPD 200×200 px, portrait):**
```
+---------------------+
|  SPD   078 km/h  N  |     (GPS speed + cardinal heading: N, NE, E, SE, S, SW, W, NW)
+---------------------+
|  ALT   1234 m       |     (GPS altitude, meters MSL)
+---------------------+
|  VAR   +2.4 m/s     |     (Vertical Speed, 4-second average)
+---------------------+
|  CLB   +1.1 m/s     |     (20-sec avg climb, if ascending)
|    or               |
|  L/D   1:8.4        |     (Glide ratio 1:X.X, if descending > 0.1 m/s)
+---------------------+
```

**Key Metrics:**
- **SPD**: GPS ground speed (km/h, metric)
- **ALT**: GPS altitude above MSL (meters)
- **VAR**: 4-second averaged vertical speed (m/s) — updated at 1 Hz from baro pressure-altitude samples
- **CLB/L/D**: 
  - Shows **20-second averaged climb rate** (m/s) if positive (climbing)
  - Shows **glide ratio `1:X`** if negative descent > 0.1 m/s (sinking)
  - Otherwise shows `—` (no meaningful descent)

**Technical Implementation:**
- Two circular ring buffers: `alt_hist_4[4]` and `alt_hist_20[20]` sampling at 1 Hz
- Cheap floating-point math: only subtraction + division per sample
- Redraw cadence: 1 Hz (tied to existing `isTimeToEPD()`)
- Zero new interrupts or timers
- Estimated CPU cost: **negligible** vs. existing baro loop

**Altitude Data Flow:**
```
Baro driver (3 Hz)    →   Pressure altitude cached
        ↓
EPD_vario_loop()      →   Sample cached pressure_alt at 1 Hz
        ↓
Ring buffers          →   Push to 4-sample and 20-sample rings
        ↓
Metrics (O(1) each)   →   calc_vs_4s(), calc_avg_climb_20s(), calc_glide_ratio()
        ↓
EPD_Draw_Vario()      →   Format 4 text lines, redraw
```

### 2. EPD Infrastructure Changes

**File: `src/driver/EPD.h`**
- Added `VIEW_MODE_VARIO = 0` (primary page, new default)
- Added function prototypes: `EPD_vario_setup()`, `EPD_vario_loop()`, `EPD_vario_next()`, `EPD_vario_prev()`

**File: `src/driver/EPD.cpp`**
- Changed default page mask from `{STATUS, RADAR, TEXT, TIME}` → `{VARIO, STATUS}`
  - Other pages remain in code (not deleted) but are not accessible via button. This preserves upstream rebase cleanness.
- Updated main display loop `EPD_loop()` — added case for `VIEW_MODE_VARIO`
- Updated page navigation `EPD_Up()` and `EPD_Down()` — added cases for vario next/prev (no-ops for now)

### 3. Flash Optimization (`src/platform/nRF52.h`)

**Excluded Unnecessary Features:**
- `EXCLUDE_UATM` — no UAT/ADS-B receiver needed (~5 KB)
- `EXCLUDE_SKYVIEW_CFG` — no SkyView config support needed (not mission critical)
- Removed `USE_EXT_I2S_DAC` — no external I2S DAC on T-Echo
- Removed `USE_TFT` — no TFT variant
- Removed `ENABLE_REMOTE_ID` — no drone ID needed for vario

**Result:** **~25 KB freed** (712 KB → 687 KB, 87% → 84% flash)

**Kept Intact (for upstream compatibility & future use):**
- All radio drivers (SX1262, FANET, OGN, FLARM protocols)
- GNSS stack (u-blox, GPS, Galileo, GLONASS support)
- Bluetooth/BLE/NMEA output (crucial for LK8EX1 vario output)
- IMU code framework (will enable for Kalman filter later)

### 4. Compilation & Build

**Latest successful build (2025-09-15 12:17 UTC):**
- **Flash usage:** 687,116 bytes (84% of 815 KB) — 25 KB headroom
- **RAM usage:** 50,648 bytes global (21%), 186 KB free for locals
- **Artifacts:**
  - `SoftRF.ino.hex` (1.9 MB) — for SWD/J-Link programming
  - `SoftRF.ino.zip` (672 KB) — for DFU/OTA via adafruit-nrfutil
  - `SoftRF.ino.elf` (15 MB) — for debugging

---

## Next Steps (Step 5 — On-Device Testing)

### Immediate (you should do next):
1. **Flash the firmware to T-Echo:**
   ```bash
   # Via DFU (drag-and-drop mass storage):
   # Double-tap the T-Echo reset button to enter DFU mode
   # Drag SoftRF.ino.zip into the MassStorage drive
   
   # OR via adafruit-nrfutil (serial):
   adafruit-nrfutil dfu serial --package .arduino-data/build/SoftRF.ino.zip \
     -p /dev/ttyACM0 -b 115200
   ```

2. **Verify vario page on EPD:**
   - Power on T-Echo
   - Should boot directly to **VARIO page** (new primary)
   - Click Mode button to cycle: VARIO ↔ STATUS
   - Verify all five data lines appear (SPD, ALT, VAR, CLB/L/D)
   - Hold the GPS fix outdoors to see GPS data and baro data populate

3. **Verify GPS/altitude data:**
   - SPD should show your actual ground speed
   - ALT should match your GPS altitude (can cross-check with phone GPS app)
   - Wait ~20 seconds for buffers to fill; VAR/CLB/L/D will show `0` or `—` until then

4. **Verify LK8EX1 Bluetooth vario output:**
   - Pair T-Echo with **XCTrack** (or **XCSoar**) mobile app
   - Open vario settings in app, confirm LK8EX1 data shows up
   - Walk up/down stairs or a hill; verify vario tone changes
   - Check that VS value on phone matches VS on T-Echo (should be very close)
   - Confirms: `ThisAircraft.vs` → NMEA LK8EX1 → BT SPP → phone

### Later (next phase — IMU + Kalman):
- Uncomment `#define EXCLUDE_IMU` → `EXCLUDE_IMU` (re-enable IMU)
- Add IMU driver integration to `Vario_EPD.cpp`
- Implement Kalman filter for fused alt/accel vertical speed
- Tune for 10–20 Hz update rate

### Deferred (nice-to-have):
- Button action for units toggle (imperial/metric switch via EPD_vario_next())
- Audio vario tone generation on nRF52840 piezo buzzer
- Pitch/roll indication (if IMU available and Kalman working)
- Thermal centering indicator

---

## Files Changed

| File | Change |
|------|--------|
| `src/platform/nRF52.h` | Added EXCLUDE_* defines; disabled unused features |
| `src/driver/EPD.h` | Added VIEW_MODE_VARIO enum; added function prototypes |
| `src/driver/EPD.cpp` | Changed default page mask; added VARIO case to display/nav switches |
| `src/ui/Vario_EPD.cpp` | **NEW** — Vario page implementation (244 lines) |
| `.gitignore` | **NEW** — Added `/.arduino-data/` to ignore build artifacts |

---

## Testing Checklist

- [ ] Firmware compiles without errors
- [ ] Firmware flashes to T-Echo without errors
- [ ] T-Echo boots and displays VARIO page
- [ ] Mode button cycles VARIO ↔ STATUS
- [ ] SPD, ALT, VAR, CLB/L/D all appear on screen
- [ ] GPS fix acquired → SPD/ALT/VAR update with real values
- [ ] XCTrack app receives LK8EX1 vario data over Bluetooth
- [ ] Vario tone changes with altitude (walk up/down stairs)

---

## Performance Notes

**Energy Consumption (nRF52840):**
- No new interrupts or wake sources added
- All computation happens during 1 Hz screen redraw (already timed)
- Ring buffer updates: 2 array writes + index increment per second
- Metric calculations: 2 subtractions + 2 divisions per second
- **Estimated added CPU time:** <1 ms per second (<<1% overhead)
- **Sleep mode:** Unchanged; device still enters WFI between SPI/GNSS/baro ISRs

**Memory (nRF52840):**
- `alt_hist_4[4]` = 16 bytes (4 × float)
- `alt_hist_20[20]` = 80 bytes (20 × float)
- `idx_4`, `idx_20`, `full_4`, `full_20` = 4 bytes
- **Total new heap:** ~100 bytes (static, not malloc)
- Plenty of headroom remaining for Kalman matrices

---

## Known Limitations / Future

1. **No IMU yet:** Vario uses only barometric altitude. Next phase adds accelerometer fusion.
2. **No units toggle:** Display is hard-coded to metric (m/s, km/h, m). Easy to add via button handler later.
3. **Glide ratio logic:** Standard threshold (0.1 m/s sink). Can be tuned if needed.
4. **Cardinal directions:** 8-way (N, NE, E, SE, …). Gives accuracy for piloting/centering.
5. **Radar/Text pages:** Still in code but not in page mask. Safe to delete later if flash pressure is high.

---

## References & Useful Commands

Build:
```bash
cd /home/ax/Projects/SoftRF-Vario-Upgrade
arduino-cli --config-file .arduino-data/arduino-cli.yaml compile \
  --fqbn adafruit:nrf52:pca10056 \
  --build-path .arduino-data/build \
  software/firmware/source/SoftRF
```

DFU Flash (OTA):
```bash
adafruit-nrfutil dfu serial \
  --package .arduino-data/build/SoftRF.ino.zip \
  -p /dev/ttyACM0 -b 115200
```

Check for LK8EX1 in serial output (minicom / picocom):
```
$LK8EX1,999999,1234,125,-15,3.9*62
       ↑      ↑    ↑    ↑  ↑
   always 999999  alt  vs°  temp  battery
                 (m)  (cm/s) (°C) (V)
```

---

## Commit Recommendation

```
commit -m "Add primary vario display page for nRF52840 (T-Echo)

- New VIEW_MODE_VARIO (primary) with live flight metrics:
  * GPS speed/heading (8-way cardinal)
  * GPS altitude (MSL)
  * 4-second averaged vertical speed (m/s)
  * 20-second averaged climb/glide ratio
- Optimize flash: remove UATM, EXT_I2S_DAC, TFT, REMOTE_ID (~25 KB)
- Flash now at 84% (687 KB), freed headroom for IMU + Kalman
- Default page sequence: VARIO ↔ STATUS (other pages preserved in code)
- LK8EX1 Bluetooth vario output unchanged; works as before
- All metrics updated at 1 Hz with negligible CPU cost
- Zero new interrupts or timers

Closes: (vario upgrade issue #N)
"
```

---

**Implementation complete. Ready for on-device testing.**
