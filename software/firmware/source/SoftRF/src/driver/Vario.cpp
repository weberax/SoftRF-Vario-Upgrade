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
#include "PiezoBeeper.h"
#include <kalmanvert.h>
#include <math.h>
#include <stdio.h>

/* ==================== Configuration ==================== */

#define VARIO_KALMAN_SIGMA_POSITION   0.1     /* m          */
#define VARIO_KALMAN_SIGMA_ACCEL      0.3     /* m/s^2      */

#define VARIO_SINKING_THRESHOLD       -2.0    /* m/s        */
#define VARIO_CLIMBING_THRESHOLD       0.2    /* m/s        */

#define VARIO_BEEP_BASE_FREQ          500      /* Hz at 0 m/s */
#define VARIO_BEEP_FREQ_COEFF         150      /* Hz per m/s  */

/* Update rate for Kalman (milliseconds between updates) */
#define VARIO_UPDATE_INTERVAL         10       /* 100 Hz      */

/* Baro update target (milliseconds between fast samples) */
#define VARIO_BARO_INTERVAL           40       /* 25 Hz       */

/* ==================== State ==================== */

static kalmanvert kalman;
static unsigned long vario_time_marker = 0;
static unsigned long baro_time_marker = 0;

/* Cached Kalman outputs */
static float kalman_alt = 0;
static float kalman_vs = 0;
static float baro_alt_prev = 0;
static float baro_vs_raw = 0;

/* IMU / Madgwick state */
static float accel_bias_z = 0;  /* Estimated bias on accel Z (earth frame, m/s^2) */
static bool accel_bias_valid = false;
static unsigned long still_duration = 0;

/* Beeper state */
static bool beeper_muted = false;

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
 * If so, refine accel_bias_z estimate.
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
    
    /* After 2 seconds, apply low-pass refine of accel_bias */
    if (still_duration > 2000 && !accel_bias_valid) {
      accel_bias_z = compute_vertical_accel();
      accel_bias_valid = true;
    }
  } else {
    still_duration = 0;
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

  char buf[80];
  snprintf(buf, sizeof(buf), "[Vario] Initialized: alt=%.1f m, sigma_p=%.2f m, sigma_a=%.2f m/s^2",
           initial_alt, VARIO_KALMAN_SIGMA_POSITION, VARIO_KALMAN_SIGMA_ACCEL);
  Serial.println(buf);
}

void Vario_loop(void) {

  unsigned long now = millis();
  
  /* ===== Kalman update @ 100 Hz ===== */
  if ((now - vario_time_marker) >= VARIO_UPDATE_INTERVAL) {
    
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

    kalman_alt = kalman.getPosition();
    kalman_vs = kalman.getVelocity();

    /* Auto-refine accel bias while still */
    update_accel_bias();

    /* Update beeper frequency based on Kalman VS (unless muted) */
    if (!beeper_muted) {
      uint16_t beep_freq = 0;
      
      if (kalman_vs > VARIO_CLIMBING_THRESHOLD) {
        /* Climbing: frequency increases with VS */
        beep_freq = VARIO_BEEP_BASE_FREQ + (uint16_t)(VARIO_BEEP_FREQ_COEFF * kalman_vs);
      } else if (kalman_vs < VARIO_SINKING_THRESHOLD) {
        /* Sinking: lower frequency, single note */
        beep_freq = 300;  /* fixed sink tone */
      }
      /* else: silent zone between -2 and +0.2 m/s */
      
      PiezoBeeper_setFreq(beep_freq);
    } else {
      PiezoBeeper_setFreq(0);  /* muted */
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
void Vario_toggleMute(void) {}
bool Vario_isMuted(void) { return false; }

#endif /* EXCLUDE_VARIO */
