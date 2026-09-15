/*
 * Vario.h
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

#ifndef DRIVER_VARIO_H
#define DRIVER_VARIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize Vario subsystem (Kalman filter + IMU).
 * Reads initial baro altitude and accel bias.
 */
void Vario_setup(void);

/**
 * Main vario update loop. Called from main loop at ~100+ Hz.
 * Updates Kalman filter, reads IMU, drives beeper frequency modulation.
 */
void Vario_loop(void);

/**
 * Get Kalman-filtered vertical speed (m/s).
 * Positive = climbing, negative = sinking.
 */
float Vario_getVario(void);

/**
 * Get Kalman-filtered altitude (m).
 */
float Vario_getAlt(void);

/**
 * Get raw baro-only vertical speed (m/s).
 * Used for LK8EX1 protocol compatibility (unchanged slow rate).
 */
float Vario_getRawVario(void);

/**
 * Calibrate Kalman altitude to GPS position (typically called on startup with good GNSS lock).
 */
void Vario_calibrateAlt(float gps_altitude);

/**
 * Toggle vario beeper mute state.
 */
void Vario_toggleMute(void);

/**
 * Query mute state.
 */
bool Vario_isMuted(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_VARIO_H */
