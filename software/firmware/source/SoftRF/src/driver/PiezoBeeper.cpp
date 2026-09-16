/*
 * PiezoBeeper.cpp
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

#include "PiezoBeeper.h"

#if defined(NRF52840_XXAA) || defined(NRF52832_XXAA)

#include <Arduino.h>
#include <nrf.h>
#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== Pin definitions ==================== */

#define PIEZO_PIN_POS       6   /* P0.06 (positive) */
#define PIEZO_PIN_NEG       8   /* P0.08 (negative / inverted) */

/* NRF_PWM0 peripheral (0 = PWM0, 1 = PWM1, 2 = PWM2 (used by tone()), 3 = PWM3) */
#define PIEZO_PWM_INSTANCE  0

/* Channel assignments for complementary output */
#define PIEZO_CH_POS        0   /* Channel 0 → P0.06, normal polarity */
#define PIEZO_CH_NEG        1   /* Channel 1 → P0.08, inverted polarity */

/* PWM clock source: 16 MHz / 16 = 1 MHz tick rate (gives us finer resolution) */
#define PIEZO_PWM_CLOCK     PWM_PRESCALER_PRESCALER_DIV_16

/* ==================== State ==================== */

static uint16_t current_freq_hz = 0;
static volatile bool pwm_enabled = false;

/* ==================== Helpers ==================== */

/**
 * Convert frequency (Hz) and duty cycle (50% for complementary drive) to PWM period (ticks).
 * Returns the PWM counter_top value in ticks (clock is 1 MHz with DIV16 prescaler).
 */
static uint16_t freq_to_pwm_top(uint16_t freq_hz) {
  if (freq_hz == 0) return 0;
  
  /* PWM clock: 16 MHz / 16 = 1 MHz
     Period in ticks = 1e6 / freq_hz
     Clamp to valid range for 16-bit counter: 3 to 32767
  */
  uint32_t top = 1000000UL / freq_hz;
  if (top < 3) top = 3;
  if (top > 32767) top = 32767;
  
  return (uint16_t)top;
}

/* ==================== Public API ==================== */

void PiezoBeeper_setup(void) {

  NRF_PWM_Type *pwm = NRF_PWM0;

  /* Configure pins as PWM outputs */
  /* P0.06 and P0.08 are set by the PSEL registers below */

  /* Stop PWM if already running */
  pwm->TASKS_STOP = 1;
  __NOP();

  /* Set clock prescaler */
  pwm->PRESCALER = PIEZO_PWM_CLOCK;

  /* Configure output pins using PSEL array
     PSEL[0] → PIEZO_CH_POS (P0.06)
     PSEL[1] → PIEZO_CH_NEG (P0.08)
     PSEL[2] and PSEL[3] left disconnected
  */
  pwm->PSEL.OUT[PIEZO_CH_POS] = (PIEZO_PIN_POS | (0u << 31));  /* P0.06, normal */
  pwm->PSEL.OUT[PIEZO_CH_NEG] = (PIEZO_PIN_NEG | (0u << 31));  /* P0.08, normal (we invert via duty) */
  pwm->PSEL.OUT[2] = 0xFFFFFFFFUL;  /* Unconnected */
  pwm->PSEL.OUT[3] = 0xFFFFFFFFUL;  /* Unconnected */

  /* Set mode: individual (each channel has its own duty) */
  pwm->MODE = PWM_MODE_UPDOWN_Up;

  /* Set counter top (period). Start with a safe default (1 kHz = 1000 ticks @ 1 MHz) */
  pwm->COUNTERTOP = 1000;

  /* Set duty cycle for both channels: 50% (top/2) for complementary drive */
  /* For inverted (P0.08), we use (top | 0x8000) to invert polarity */
  pwm->SEQ[0].PTR = 0;  /* We'll set values directly in registers, not via sequence */
  pwm->SEQ[0].CNT = 0;

  /* Configure as individual (continuous update mode) */
  pwm->LOOP = 0;  /* Single sequence, no loop */

  /* Configure duty registers directly.
     PWM uses the SEQ[0].PTR to DMA, but we can also write directly to CCx registers.
     However, CCx are only available in certain configs. Instead, we use inline duty setting.
     For simplicity with individual mode, we'll use a small 4-entry duty table.
  */

  /* Allocate RAM for duty cycle values (must be in RAM for DMA) */
  static uint16_t pwm_duty[4];
  
  pwm->SEQ[0].PTR = (uint32_t)&pwm_duty[0];
  pwm->SEQ[0].CNT = 2;  /* 2 channels, 2 shorts = 4 bytes */

  /* Start disabled; we'll enable when first frequency is set */
  pwm_enabled = false;

  Serial.println(F("[PiezoBeeper] Setup complete on NRF_PWM0, pins P0.06 (pos) + P0.08 (neg)"));
}

void PiezoBeeper_setFreq(uint16_t freq_hz) {

  NRF_PWM_Type *pwm = NRF_PWM0;

  if (freq_hz == current_freq_hz) {
    return;  /* No change */
  }

  current_freq_hz = freq_hz;

  if (freq_hz == 0) {
    /* Stop PWM */
    if (pwm_enabled) {
      pwm->TASKS_STOP = 1;
      __NOP();
      pwm_enabled = false;
    }
    return;
  }

  /* Calculate PWM period for the requested frequency */
  uint16_t top = freq_to_pwm_top(freq_hz);

  /* Stop PWM before reconfiguring */
  if (pwm_enabled) {
    pwm->TASKS_STOP = 1;
    __NOP();
  }

  /* Update counter top (period) */
  pwm->COUNTERTOP = top;

  /* Set duty cycle for both channels: 50% (top/2)
     Channel 0 (P0.06): normal polarity = top/2
     Channel 1 (P0.08): inverted polarity = (top/2) | 0x8000 bit (polarity inversion flag)
  */
  uint16_t duty = top / 2;

  /* Update duty via memory (if we have a SEQ buffer set up) or direct registers.
     For now, we'll access via the PWM_CC registers if available, or set up a minimal SEQ buffer.
     
     To keep it simple and portable, we set a 2-entry buffer and restart the sequence.
  */
  
  /* Get pointer to SEQ buffer we allocated in setup() */
  uint16_t *duty_ptr = (uint16_t *)pwm->SEQ[0].PTR;
  
  if (duty_ptr != NULL) {
    duty_ptr[0] = duty;                   /* Channel 0: normal */
    duty_ptr[1] = duty | 0x8000;          /* Channel 1: inverted */
  }

  /* Ensure DMA updated (manual sync point) */
  __ISB();

  /* Start PWM */
  pwm->TASKS_SEQSTART[0] = 1;
  __NOP();
  pwm_enabled = true;
}

uint16_t PiezoBeeper_getFreq(void) {
  return current_freq_hz;
}

#else
  /* Stub for non-nRF52 platforms */
  void PiezoBeeper_setup(void) {}
  void PiezoBeeper_setFreq(uint16_t freq_hz) { (void)freq_hz; }
  uint16_t PiezoBeeper_getFreq(void) { return 0; }
#endif

#ifdef __cplusplus
}
#endif
