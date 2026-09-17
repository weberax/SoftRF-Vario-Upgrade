/*
 * Vario.cpp
 * Copyright (C) 2025 Vario Upgrade Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../system/SoC.h"

#if !defined(EXCLUDE_VARIO)

#include "Vario.h"
#include "Baro.h"
#include "GNSS.h"
#include "EEPROM.h"
#include "PiezoBeeper.h"
#include <kalmanvert.h>
#include <beeper.h>
#include <math.h>
#include <stdio.h>

/* ==================== Configuration ==================== */

#define VARIO_KALMAN_SIGMA_POSITION   0.1     /* m          */
#define VARIO_KALMAN_SIGMA_ACCEL      0.3     /* m/s^2      */

/* Beep thresholds/volume, ported from the reference project's tuned
   VarioSettings.h rather than the GNUVarioBeeper library defaults. */
#define VARIO_BEEP_SINKING_THRESHOLD          -4.0   /* m/s */
#define VARIO_BEEP_CLIMBING_THRESHOLD          0.1   /* m/s */
#define VARIO_BEEP_NEAR_CLIMBING_SENSITIVITY   3.5   /* m/s */
#define VARIO_BEEP_VOLUME                        6   /* of 10 */

/* Update rate for Kalman (milliseconds between updates) */
#define VARIO_UPDATE_INTERVAL         10       /* 100 Hz      */

/* Baro update target (milliseconds between fast samples) */
#define VARIO_BARO_INTERVAL           40       /* 25 Hz       */

/* ==================== State ==================== */

static kalmanvert kalman;
static unsigned long vario_time_marker = 0;
static unsigned long baro_time_marker = 0;

/* Climb/sink/near-climb beep pattern state machine, ported from the
   reference project (GNUVario) - thresholds set from VARIO_BEEP_* in
   Vario_setup(). */
static beeper vario_beeper;

/* Cached Kalman outputs */
static float kalman_alt = 0;
static float kalman_vs = 0;
static float baro_alt_prev = 0;
static float baro_vs_raw = 0;

/* IMU / Madgwick state */
static float accel_bias_z = 0;  /* Estimated bias on accel Z (earth frame, m/s^2) */
static bool accel_bias_valid = false;
static unsigned long still_duration = 0;
static bool still_bias_applied = false;  /* nudged bias already for this still period? */

/* Continuous auto-correction: how strongly each *new* stillness period
   nudges accel_bias_z (0..1, low-pass coefficient) - applied once per still
   period (not every tick while still), so slow thermal/temperature drift
   gets tracked across a session without a single noisy sample yanking the
   bias around. */
#define VARIO_ACCEL_BIAS_LOWPASS   0.15f

/* Beeper state */
static bool beeper_muted = false;

/* One-shot GPS-altitude calibration state */
#define VARIO_GPS_CAL_FIX_HOLD_MS   5000   /* require a continuously valid fix this long first */
/* Sanity bound vs. the running baro-relative altitude: real QNH-vs-standard-
   pressure differences are at most on the order of ~150-200m even in
   extreme weather. A bigger gap means the GPS vertical solution itself is
   bad (common indoors/poor sky view - horizontal fix can look fine while
   altitude is garbage) - skip calibrating against it rather than locking
   in a bad value, and keep retrying each tick until it becomes plausible. */
#define VARIO_GPS_CAL_MAX_BARO_DELTA_M   200.0f
static bool alt_calibrated = false;
static unsigned long gps_fix_stable_since = 0;

/* Debug telemetry: 1Hz baro/GPS/Kalman status line + immediate fix
   transition prints, for bring-up monitoring over serial. */
#define VARIO_DEBUG_INTERVAL   1000  /* ms */
static unsigned long debug_time_marker = 0;
static bool debug_prev_fix_valid = false;

/* ==================== Forward declarations ==================== */

extern float Baro_altitude(void);

#if !defined(EXCLUDE_IMU)
#include <MPU9250.h>
extern MPU9250 imu_1;
#endif

/* ==================== Kalman + IMU helpers ==================== */

/**
 * Project accelerometer reading from body frame to earth frame using Madgwick quaternion.
 * Returns vertical acceleration (earth Z, positive up), in m/s^2.
 * Returns 0 if IMU not available.
 */
static float compute_vertical_accel(void) {

#if !defined(EXCLUDE_IMU)
  extern MPU9250 imu_1;
  
  if (hw_info.imu != IMU_MPU9250) {
    return 0;
  }

  /* Get quaternion (body to earth frame).
     Note: hideakitai MPU9250 uses scalar-first convention: q = [w, x, y, z]
     We rotate accel from body frame to earth frame:
     a_earth = R^T * a_body, where R is the rotation matrix from q.
  */
  
  float ax_body = imu_1.getAccX();  /* m/s^2, body frame */
  float ay_body = imu_1.getAccY();
  float az_body = imu_1.getAccZ();

  float q0 = imu_1.getQuaternionW();
  float q1 = imu_1.getQuaternionX();
  float q2 = imu_1.getQuaternionY();
  float q3 = imu_1.getQuaternionZ();

  /* Rotate: a_earth_z = (q0^2 - q1^2 - q2^2 + q3^2)*az + 2*(q0*q1 + q2*q3)*ax + 2*(q0*q2 - q1*q3)*ay */
  /* This is the Z component of R^T * a_body */
  float az_earth = (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az_body
                 + 2.0f * (q0*q1 + q2*q3) * ax_body
                 + 2.0f * (q0*q2 - q1*q3) * ay_body;

  /* Remove gravity (already subtracted by Madgwick but add here as reference) */
  /* The Madgwick filter outputs a quaternion to level frame, so az_earth ≈ g initially */
  /* Subtract 1g offset to get actual vertical acceleration */
  float vert_accel = az_earth - 9.80665f;

  return vert_accel;
#else
  return 0;
#endif
}

/**
 * Check if the device has been motionless for >2 seconds (accel near 1g, low gyro).
 * If so, slowly nudge accel_bias_z towards the current reading. Runs every
 * time stillness is (re-)detected, not just once, so it keeps tracking slow
 * drift (temperature, long-term sensor aging) through a whole session
 * instead of freezing after the first still period.
 */
static void update_accel_bias(void) {
#if !defined(EXCLUDE_IMU)
  extern MPU9250 imu_1;

  float ax = imu_1.getAccX();
  float ay = imu_1.getAccY();
  float az = imu_1.getAccZ();
  float gx = imu_1.getGyroX();
  float gy = imu_1.getGyroY();
  float gz = imu_1.getGyroZ();

  float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
  float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);

  /* Check if device is still (accel magnitude near 1g, gyro low) */
  if (fabsf(accel_mag - 9.80665f) < 0.5f && gyro_mag < 1.0f) {
    still_duration += VARIO_UPDATE_INTERVAL;

    /* Nudge once, 2s into this still period - not on every tick for as
       long as stillness continues, so a single still period contributes
       one gentle correction rather than fully re-locking the bias. */
    if (still_duration > 2000 && !still_bias_applied) {
      float sample = compute_vertical_accel();
      if (!accel_bias_valid) {
        /* First estimate of the session: take it outright, no need to
           slow-walk from an arbitrary zero starting point. */
        accel_bias_z = sample;
        accel_bias_valid = true;
      } else {
        accel_bias_z += VARIO_ACCEL_BIAS_LOWPASS * (sample - accel_bias_z);
      }
      still_bias_applied = true;
    }
  } else {
    still_duration = 0;
    still_bias_applied = false;
  }
#endif
}

/* ==================== Public API ==================== */

void Vario_setup(void) {

  /* Read initial baro altitude */
  float initial_alt = Baro_altitude();

  /* Initialize Kalman filter with zero initial vertical speed.
     We'll assume the device starts in a still, level position.
     The accel bias starts at 0 and gets refined during first 2s of stillness.
  */
  kalman.init(initial_alt,               /* initial altitude (m) */
              0,                         /* initial accel (m/s^2) */
              VARIO_KALMAN_SIGMA_POSITION,
              VARIO_KALMAN_SIGMA_ACCEL,
              millis());

  kalman_alt = initial_alt;
  kalman_vs = 0;
  baro_alt_prev = initial_alt;
  baro_vs_raw = 0;

  vario_time_marker = millis();
  baro_time_marker = millis();
  beeper_muted = false;

  vario_beeper.setThresholds(VARIO_BEEP_SINKING_THRESHOLD,
                              VARIO_BEEP_CLIMBING_THRESHOLD,
                              VARIO_BEEP_NEAR_CLIMBING_SENSITIVITY);
  vario_beeper.setVolume(VARIO_BEEP_VOLUME);
  vario_beeper.setGlidingBeepState(true);   /* near-climb "blip", ref. project has this on */
  vario_beeper.setGlidingAlarmState(false); /* ref. project has the near-climb alarm off */

  alt_calibrated = false;
  gps_fix_stable_since = 0;
  accel_bias_z = 0;
  accel_bias_valid = false;
  still_duration = 0;
  still_bias_applied = false;

#if !defined(EXCLUDE_IMU)
  /* Apply saved full-calibration bias (Vario_calibrateIMU()), if any. The
     continuous auto-correction above still applies on top of this as a
     slow trim, it just no longer has to start from zero every boot. */
  if (hw_info.imu == IMU_MPU9250 && settings->imu_calibrated) {
    imu_1.setAccBias(settings->imu_accel_bias[0],
                      settings->imu_accel_bias[1],
                      settings->imu_accel_bias[2]);
    imu_1.setGyroBias(settings->imu_gyro_bias[0],
                       settings->imu_gyro_bias[1],
                       settings->imu_gyro_bias[2]);
    Serial.println(F("[Vario] Applied saved MPU9250 calibration"));
  }
#endif

  char buf[80];
  snprintf(buf, sizeof(buf), "[Vario] Initialized: alt=%.1f m, sigma_p=%.2f m, sigma_a=%.2f m/s^2",
           initial_alt, VARIO_KALMAN_SIGMA_POSITION, VARIO_KALMAN_SIGMA_ACCEL);
  Serial.println(buf);
}

void Vario_loop(void) {

  unsigned long now = millis();
  
  /* ===== Kalman update @ 100 Hz ===== */
  if ((now - vario_time_marker) >= VARIO_UPDATE_INTERVAL) {

#if !defined(EXCLUDE_IMU)
    /* Refresh accel/quaternion at the Kalman loop's own rate - the generic
       G-load handling in nRF52.cpp only samples the IMU every 500ms, which
       is far too coarse for accel-assisted vario response. */
    if (hw_info.imu == IMU_MPU9250) {
      imu_1.update();
    }
#endif

    /* Read current acceleration (earth frame, accounting for bias) */
    float vert_accel = compute_vertical_accel() - accel_bias_z;
    
    /* Check for new baro data */
    float baro_alt = Baro_altitude();
    bool baro_updated = false;
    
    if ((now - baro_time_marker) >= VARIO_BARO_INTERVAL) {
      baro_updated = true;
      baro_time_marker = now;
      
      /* Compute raw baro-only vertical speed for LK8EX1 (still at slow 1 Hz rate) */
      /* This is updated at 25 Hz but reported slowly for protocol compatibility */
      if (baro_alt != baro_alt_prev) {
        float dt = (now - vario_time_marker) / 1000.0f;
        baro_vs_raw = (baro_alt - baro_alt_prev) / (VARIO_BARO_INTERVAL / 1000.0f);
        baro_alt_prev = baro_alt;
      }
    }

    /* Update Kalman: always feed accel, only feed baro when available */
    if (baro_updated) {
      kalman.update(baro_alt, vert_accel, now);
    } else {
      /* Prediction-only step: feed NaN for measurement to skip correction */
      /* For now, we do a full update with stale baro. Alternative: modify kalmanvert to accept NaN */
      kalman.update(baro_alt, vert_accel, now);
    }

    /* getCalibratedPosition(), not getPosition() - the latter is the raw
       internal state and never reflects Vario_calibrateAlt()'s GPS offset. */
    kalman_alt = kalman.getCalibratedPosition();
    kalman_vs = kalman.getVelocity();

    /* Immediate (non-throttled) fix-acquired/lost transition log */
    bool fix_valid_now = isValidGNSSFix();
    if (fix_valid_now != debug_prev_fix_valid) {
      Serial.println(fix_valid_now ? F("[Vario] GPS fix ACQUIRED") : F("[Vario] GPS fix LOST"));
      debug_prev_fix_valid = fix_valid_now;
    }

    /* One-shot GPS-altitude calibration: once a fix has stayed valid for a
       few seconds, nudge the Kalman altitude to match GPS. Corrects for the
       initial baro-only altitude being relative to whatever pressure was
       read at boot, rather than true MSL. */
    if (!alt_calibrated) {
      if (isValidGNSSFix()) {
        if (gps_fix_stable_since == 0) {
          gps_fix_stable_since = now;
        } else if ((now - gps_fix_stable_since) >= VARIO_GPS_CAL_FIX_HOLD_MS) {
          float delta = ThisAircraft.altitude - kalman.getPosition();
          if (fabsf(delta) <= VARIO_GPS_CAL_MAX_BARO_DELTA_M) {
            Vario_calibrateAlt(ThisAircraft.altitude);
            kalman_alt = kalman.getCalibratedPosition();
            alt_calibrated = true;
          } else {
            char wbuf[100];
            snprintf(wbuf, sizeof(wbuf),
              "[Vario] WARNING: GPS alt %.1fm vs baro %.1fm differ by %.1fm - "
              "skipping calibration, GPS vertical fix looks bad",
              ThisAircraft.altitude, kalman.getPosition(), delta);
            Serial.println(wbuf);
            gps_fix_stable_since = now;  /* retry in another VARIO_GPS_CAL_FIX_HOLD_MS, not every tick */
          }
        }
      } else {
        gps_fix_stable_since = 0;
      }
    }

    /* Auto-refine accel bias while still */
    update_accel_bias();

    /* Drive the climb/sink/near-climb beep pattern from Kalman VS (unless muted) */
    if (!beeper_muted) {
      vario_beeper.setVelocity(kalman_vs);
      vario_beeper.update();
    } else {
      PiezoBeeper_setFreq(0);  /* muted */
    }

    /* Throttled (1Hz) baro/GPS/Kalman status line for bring-up monitoring */
    if ((now - debug_time_marker) >= VARIO_DEBUG_INTERVAL) {
      char dbuf[160];
      snprintf(dbuf, sizeof(dbuf),
        "[Vario] baro=%.2fm vacc=%.3f bias=%.3f%s | kalman alt=%.2fm vs=%.2fm/s | "
        "GPS fix=%d alt=%.1fm spd=%.1fkmh sats=%u hdop=%s",
        baro_alt, vert_accel, accel_bias_z, accel_bias_valid ? "" : "(warming up)",
        kalman_alt, kalman_vs,
        fix_valid_now ? 1 : 0, ThisAircraft.altitude, gnss.speed.kmph(),
        gnss.satellites.value(),
        gnss.hdop.isValid() ? String(gnss.hdop.hdop(), 2).c_str() : "-");
      Serial.println(dbuf);
      debug_time_marker = now;
    }

    vario_time_marker = now;
  }
}

float Vario_getVario(void) {
  return kalman_vs;
}

float Vario_getAlt(void) {
  return kalman_alt;
}

float Vario_getRawVario(void) {
  return baro_vs_raw;
}

void Vario_calibrateAlt(float gps_altitude) {
  kalman.calibratePosition(gps_altitude);
  kalman_alt = gps_altitude;
  char buf[60];
  snprintf(buf, sizeof(buf), "[Vario] Calibrated altitude to GPS: %.1f m", gps_altitude);
  Serial.println(buf);
}

void Vario_calibrateIMU(void) {
#if !defined(EXCLUDE_IMU)
  if (hw_info.imu != IMU_MPU9250) {
    Serial.println(F("[Vario] IMU calibration skipped: no MPU9250 detected"));
    return;
  }

  Serial.println(F("[Vario] Full IMU calibration starting - keep the device still and level..."));
  PiezoBeeper_setFreq(600);  /* audible "calibrating, hold still" cue */

  imu_1.calibrateAccelGyro();

  settings->imu_accel_bias[0] = imu_1.getAccBiasX();
  settings->imu_accel_bias[1] = imu_1.getAccBiasY();
  settings->imu_accel_bias[2] = imu_1.getAccBiasZ();
  settings->imu_gyro_bias[0]  = imu_1.getGyroBiasX();
  settings->imu_gyro_bias[1]  = imu_1.getGyroBiasY();
  settings->imu_gyro_bias[2]  = imu_1.getGyroBiasZ();
  settings->imu_calibrated    = true;

  EEPROM_store();

  /* The continuous vertical-accel bias tracked above is relative to
     whatever the IMU was reporting before this fresh calibration -
     restart it so it doesn't fight the new hardware-level bias. */
  accel_bias_z      = 0;
  accel_bias_valid  = false;
  still_duration     = 0;
  still_bias_applied = false;

  PiezoBeeper_setFreq(0);
  Serial.println(F("[Vario] Full IMU calibration complete and saved."));
#endif
}

void Vario_toggleMute(void) {
  beeper_muted = !beeper_muted;
  char buf[40];
  snprintf(buf, sizeof(buf), "[Vario] Beeper muted: %d", beeper_muted ? 1 : 0);
  Serial.println(buf);
}

bool Vario_isMuted(void) {
  return beeper_muted;
}

#else

/* Stub implementations when EXCLUDE_VARIO is defined */
void Vario_setup(void) {}
void Vario_loop(void) {}
float Vario_getVario(void) { return 0; }
float Vario_getAlt(void) { return 0; }
float Vario_getRawVario(void) { return 0; }
void Vario_calibrateAlt(float gps_altitude) { (void)gps_altitude; }
void Vario_calibrateIMU(void) {}
void Vario_toggleMute(void) {}
bool Vario_isMuted(void) { return false; }

#endif /* EXCLUDE_VARIO */
