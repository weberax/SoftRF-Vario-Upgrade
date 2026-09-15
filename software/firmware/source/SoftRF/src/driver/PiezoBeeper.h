/*
 * PiezoBeeper.h
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

#ifndef DRIVER_PIEZO_BEEPER_H
#define DRIVER_PIEZO_BEEPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize piezo beeper on NRF_PWM0 (pins P0.06 and P0.08, complementary drive).
 * Must be called once during setup.
 */
void PiezoBeeper_setup(void);

/**
 * Set beeper frequency (Hz). 0 = silence.
 * Frequency range: 100-5000 Hz recommended.
 * Complementary drive provides ~6.6V peak-to-peak (vs 3.3V single-pin).
 */
void PiezoBeeper_setFreq(uint16_t freq_hz);

/**
 * Get current beeper frequency.
 */
uint16_t PiezoBeeper_getFreq(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_PIEZO_BEEPER_H */
