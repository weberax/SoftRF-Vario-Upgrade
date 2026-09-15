/*
 * Vario_EPD.cpp
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

#if defined(USE_EPAPER)

#include "../driver/EPD.h"
#include "../driver/Baro.h"

#include <TinyGPS++.h>

/* Ring buffers for averaging */
#define VARIO_HIST_4   4
#define VARIO_HIST_20  20

static float alt_hist_4[VARIO_HIST_4];
static float alt_hist_20[VARIO_HIST_20];
static uint8_t idx_4 = 0;
static uint8_t idx_20 = 0;
static bool full_4 = false;
static bool full_20 = false;

/* Cardinal directions (8-way) */
static const char* cardinal_8[8] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };

/* Constants for conversion */
#define M_TO_FT  3.28084
#define M_S_TO_FPM  196.85  /* m/s to feet/min */

/**
 * Get 8-way cardinal direction from course (degrees, 0-360)
 */
static const char* get_cardinal_8(float course) {
  if (course < 0 || course >= 360) course = fmod(course, 360);
  int idx = (int)((course + 22.5) / 45.0) & 7;
  return cardinal_8[idx];
}

/**
 * Calculate 4-second vertical speed average (m/s).
 * Returns the rate of change of pressure altitude over the last ~4 seconds.
 */
static float calc_vs_4s() {
  if (!full_4) return 0;
  
  float alt_newest = alt_hist_4[idx_4];
  float alt_oldest = alt_hist_4[(idx_4 + 1) % VARIO_HIST_4];
  
  return (alt_newest - alt_oldest) / VARIO_HIST_4;  /* m/s */
}

/**
 * Calculate 20-second climb/sink average (m/s).
 * Returns the rate of change of pressure altitude over the last ~20 seconds.
 */
static float calc_avg_climb_20s() {
  if (!full_20) return 0;
  
  float alt_newest = alt_hist_20[idx_20];
  float alt_oldest = alt_hist_20[(idx_20 + 1) % VARIO_HIST_20];
  
  return (alt_newest - alt_oldest) / VARIO_HIST_20;  /* m/s */
}

/**
 * Calculate glide ratio from current ground speed and average sink rate.
 * Returns glide ratio (horizontal distance / vertical distance).
 * Input: speed in m/s, sink_rate in m/s (should be negative for descent).
 */
static float calc_glide_ratio(float speed_m_s, float sink_rate_m_s) {
  /* Standard glide ratio threshold: only show if sink rate > 0.1 m/s */
  if (sink_rate_m_s < 0.1f) {
    return -1.0f;  /* Invalid/no descent marker */
  }
  
  /* Avoid division by zero */
  if (speed_m_s < 0.01f) {
    return -1.0f;
  }
  
  return speed_m_s / sink_rate_m_s;
}

void EPD_vario_setup()
{
  /* Initialize ring buffers */
  for (int i = 0; i < VARIO_HIST_4; i++) {
    alt_hist_4[i] = 0;
  }
  for (int i = 0; i < VARIO_HIST_20; i++) {
    alt_hist_20[i] = 0;
  }
  idx_4 = 0;
  idx_20 = 0;
  full_4 = false;
  full_20 = false;
}

static void EPD_Draw_Vario()
{
  char buf[32];
  int16_t  tbx, tby;
  uint16_t tbw, tbh;
  uint16_t display_width  = display->width();
  uint16_t display_height = display->height();

#if defined(USE_EPD_TASK)
  if (EPD_update_in_progress == EPD_UPDATE_NONE) {
#else
  {
#endif
    display->fillScreen(GxEPD_WHITE);

    display->setFont(&FreeMonoBold18pt7b);
    display->setTextColor(GxEPD_BLACK);

    uint16_t line_height = 36;
    uint16_t y_start = 20;

    /* ===== Line 1: GPS Speed + Heading ===== */
    {
      float speed_kn = ThisAircraft.speed;
      float speed_kmh = speed_kn * 1.852;  /* knots to km/h */
      const char *cardinal = get_cardinal_8(ThisAircraft.course);

      snprintf(buf, sizeof(buf), "SPD %03.0f %s %s",
               speed_kmh, "km/h", cardinal);
      
      display->getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);
      display->setCursor(8, y_start);
      display->print(buf);
    }

    /* ===== Line 2: GPS Altitude (MSL) ===== */
    {
      float alt_m = ThisAircraft.altitude;
      float alt_ft = alt_m * M_TO_FT;

      snprintf(buf, sizeof(buf), "ALT  %4.0f m",
               alt_m);
      
      display->getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);
      display->setCursor(8, y_start + line_height);
      display->print(buf);
    }

    /* ===== Line 3: 4-second Vertical Speed ===== */
    {
      float vs_4s = calc_vs_4s();
      float vs_fpm = vs_4s * M_S_TO_FPM;

      snprintf(buf, sizeof(buf), "VAR %+5.2f m/s",
               vs_4s);
      
      display->getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);
      display->setCursor(8, y_start + 2 * line_height);
      display->print(buf);
    }

    /* ===== Line 4-5: 20-second Climb OR Glide Ratio ===== */
    {
      float avg_climb_20s = calc_avg_climb_20s();
      
      if (avg_climb_20s > 0) {
        /* Climbing: show average climb rate */
        snprintf(buf, sizeof(buf), "CLB %+5.2f m/s",
                 avg_climb_20s);
      } else {
        /* Descending: show glide ratio */
        float speed_m_s = ThisAircraft.speed * 0.5144;  /* knots to m/s */
        float sink_rate = -avg_climb_20s;  /* Make positive */
        float glide = calc_glide_ratio(speed_m_s, sink_rate);
        
        if (glide > 0 && glide < 100) {
          snprintf(buf, sizeof(buf), "L/D  1:%.1f",
                   glide);
        } else {
          /* No valid glide or glide > 99 */
          snprintf(buf, sizeof(buf), "L/D    ---");
        }
      }
      
      display->getTextBounds(buf, 0, 0, &tbx, &tby, &tbw, &tbh);
      display->setCursor(8, y_start + 3 * line_height);
      display->print(buf);
    }

#if defined(USE_EPD_TASK)
    EPD_update_in_progress = EPD_UPDATE_FAST;
#else
    display->display(true);
#endif
  }
}

void EPD_vario_loop()
{
  if (isTimeToEPD()) {
    /* Update ring buffers with current pressure altitude at 1 Hz */
    float pressure_alt = Baro_altitude();

    /* Update 4-sample ring */
    alt_hist_4[idx_4] = pressure_alt;
    idx_4 = (idx_4 + 1) % VARIO_HIST_4;
    if (idx_4 == 0) full_4 = true;

    /* Update 20-sample ring */
    alt_hist_20[idx_20] = pressure_alt;
    idx_20 = (idx_20 + 1) % VARIO_HIST_20;
    if (idx_20 == 0) full_20 = true;

    /* Redraw the vario page */
    EPD_Draw_Vario();

    EPDTimeMarker = millis();
  }
}

void EPD_vario_next()
{
  /* Up button: no-op for now (can add units toggle later) */
}

void EPD_vario_prev()
{
  /* Down button: no-op for now */
}

#endif /* USE_EPAPER */
