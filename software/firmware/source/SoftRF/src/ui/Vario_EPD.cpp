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
#include "../driver/Vario.h"

#include <TinyGPS++.h>

/* Ring buffers for averaging */
#define VARIO_HIST_20  20
#define VARIO_HIST_2S  20  /* 2-second smoothing at ~10 Hz = 20 samples */

static float alt_hist_20[VARIO_HIST_20];
static float vario_hist_2s[VARIO_HIST_2S];
static uint8_t idx_20 = 0;
static uint8_t idx_2s = 0;
static bool full_20 = false;
static bool full_2s = false;

/* Navbox structure for 5-box layout (4 boxes top/middle, 1 wide box bottom) */
static navbox_t navbox1;  /* GS km/h (top-left) */
static navbox_t navbox2;  /* HDG (top-right) */
static navbox_t navbox3;  /* GPS m (middle-left) */
static navbox_t navbox4;  /* vV m/s (middle-right) */
static navbox_t navbox5;  /* GLIDE/CLIMB wide box (bottom, full width) */

/**
 * Get 2-second smoothed vertical speed from ring buffer (m/s)
 */
static float get_vs_smoothed_2s() {
  if (!full_2s) return 0;
  
  float sum = 0;
  for (int i = 0; i < VARIO_HIST_2S; i++) {
    sum += vario_hist_2s[i];
  }
  return sum / VARIO_HIST_2S;
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
 * Input: speed in m/s, sink_rate in m/s (should be positive for descent).
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
  uint16_t display_width  = display->width();
  uint16_t display_height = display->height();
  int16_t dy = 0;

#if defined(EPD_ASPECT_RATIO_2C1)
  if (display->epd2.panel == GxEPD2::DEPG0213BN) {
    if (display_width  == 128) display_width  = 122;
    if (display_height == 128) {
      display_height = 122;
      if (display->getRotation() == ROTATE_90 ) { dy = 6; }
    }
  }
#endif /* EPD_ASPECT_RATIO_2C1 */

  /* Initialize ring buffers */
  for (int i = 0; i < VARIO_HIST_20; i++) {
    alt_hist_20[i] = 0;
  }
  for (int i = 0; i < VARIO_HIST_2S; i++) {
    vario_hist_2s[i] = 0;
  }
  idx_20 = 0;
  idx_2s = 0;
  full_20 = false;
  full_2s = false;

  /* Setup navboxes: 4 boxes in top/middle rows, 1 wide box at bottom */
  
  /* Box 1: GS km/h (top-left) */
  memcpy(navbox1.title, "GS km/h", 7);
  navbox1.x = 0;
  navbox1.y = 0 + dy;
  navbox1.width = display_width / 2;
  navbox1.height = display_height / 3;
  navbox1.value = 0;
  navbox1.timestamp = millis();

  /* Box 2: HDG (top-right) */
  memcpy(navbox2.title, "HDG", 3);
  navbox2.x = navbox1.width;
  navbox2.y = navbox1.y;
  navbox2.width = display_width / 2;
  navbox2.height = display_height / 3;
  navbox2.value = 0;
  navbox2.timestamp = millis();

  /* Box 3: GPS m (middle-left) */
  memcpy(navbox3.title, "GPS m", 5);
  navbox3.x = navbox1.x;
  navbox3.y = navbox1.y + navbox1.height;
  navbox3.width = display_width / 2;
  navbox3.height = display_height / 3;
  navbox3.value = 0;
  navbox3.timestamp = millis();

  /* Box 4: vV m/s (middle-right) */
  memcpy(navbox4.title, "vV m/s", 6);
  navbox4.x = navbox3.width;
  navbox4.y = navbox3.y;
  navbox4.width = display_width / 2;
  navbox4.height = display_height / 3;
  navbox4.value = 0;
  navbox4.timestamp = millis();

  /* Box 5: GLIDE/CLIMB wide (bottom, full width) */
  memcpy(navbox5.title, "GLIDE", 5);
  navbox5.x = navbox1.x;
  navbox5.y = navbox3.y + navbox3.height;
  navbox5.width = display_width;
  navbox5.height = display_height / 3;
  navbox5.value = 0;
  navbox5.timestamp = millis();
}

static void EPD_Draw_NavBoxes()
{
  char buf[32];
  int16_t  tbx, tby;
  uint16_t tbw, tbh;

#if defined(USE_EPD_TASK)
  if (EPD_update_in_progress == EPD_UPDATE_NONE) {
#else
  {
#endif
    display->fillScreen(GxEPD_WHITE);

    /* Draw boxes 1 & 2 (top row) */
    display->drawRoundRect( navbox1.x + 1, navbox1.y + 1,
                            navbox1.width - 2, navbox1.height - 2,
                            4, GxEPD_BLACK);
    display->drawRoundRect( navbox2.x + 1, navbox2.y + 1,
                            navbox2.width - 2, navbox2.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    /* Box 1 title */
    display->getTextBounds(navbox1.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox1.x + 5, navbox1.y + 5 + tbh);
    display->print(navbox1.title);

    /* Box 2 title */
    display->getTextBounds(navbox2.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox2.x + 5, navbox2.y + 5 + tbh);
    display->print(navbox2.title);

    display->setFont(&FreeMonoBold18pt7b);

    /* Box 1 value (GS km/h) */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox1.x + 25, navbox1.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox1.x + 75, navbox1.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    snprintf(buf, sizeof(buf), "%.0f", navbox1.value);
    display->print(buf);

    /* Box 2 value (HDG) */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox2.x + 15, navbox2.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox2.x + 55, navbox2.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    snprintf(buf, sizeof(buf), "%.0f", navbox2.value);
    display->print(buf);

    /* Draw boxes 3 & 4 (middle row) */
    display->drawRoundRect( navbox3.x + 1, navbox3.y + 1,
                            navbox3.width - 2, navbox3.height - 2,
                            4, GxEPD_BLACK);
    display->drawRoundRect( navbox4.x + 1, navbox4.y + 1,
                            navbox4.width - 2, navbox4.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    /* Box 3 title */
    display->getTextBounds(navbox3.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox3.x + 5, navbox3.y + 5 + tbh);
    display->print(navbox3.title);

    /* Box 4 title */
    display->getTextBounds(navbox4.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox4.x + 5, navbox4.y + 5 + tbh);
    display->print(navbox4.title);

    display->setFont(&FreeMonoBold18pt7b);

    /* Box 3 value (GPS m) */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox3.x + 25, navbox3.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox3.x + 75, navbox3.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    snprintf(buf, sizeof(buf), "%.0f", navbox3.value);
    display->print(buf);

    /* Box 4 value (vV m/s with sign) */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox4.x + 15, navbox4.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox4.x + 55, navbox4.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    snprintf(buf, sizeof(buf), "%+.2f", navbox4.value);
    display->print(buf);

    /* Draw box 5 (bottom, full width) */
    display->drawRoundRect( navbox5.x + 1, navbox5.y + 1,
                            navbox5.width - 2, navbox5.height - 2,
                            4, GxEPD_BLACK);

    display->setFont(&FreeMono9pt7b);

    /* Box 5 title */
    display->getTextBounds(navbox5.title, 0, 0, &tbx, &tby, &tbw, &tbh);
    display->setCursor(navbox5.x + 5, navbox5.y + 5 + tbh);
    display->print(navbox5.title);

    display->setFont(&FreeMonoBold18pt7b);

    /* Box 5 value (GLIDE ratio or CLIMB average) */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox5.x + 25, navbox5.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox5.x + 75, navbox5.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    
    /* navbox5.value encodes: >0 = glide ratio, <0 = climb (show as climb m/s) */
    if (navbox5.value > 0 && navbox5.value < 100) {
      snprintf(buf, sizeof(buf), "1:%.1f", navbox5.value);
    } else if (navbox5.value < 0) {
      snprintf(buf, sizeof(buf), "%+.2f", -navbox5.value);
    } else {
      snprintf(buf, sizeof(buf), "---");
    }
    display->print(buf);

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
    /* Update altitude ring buffer at 1 Hz */
    float pressure_alt = Baro_altitude();
    alt_hist_20[idx_20] = pressure_alt;
    idx_20 = (idx_20 + 1) % VARIO_HIST_20;
    if (idx_20 == 0) full_20 = true;

    /* Update 2-second smoothing ring buffer with Kalman vario at ~10 Hz */
    float vs_raw = Vario_getVario();
    vario_hist_2s[idx_2s] = vs_raw;
    idx_2s = (idx_2s + 1) % VARIO_HIST_2S;
    if (idx_2s == 0) full_2s = true;

    /* Prepare navbox values */
    float speed_kn = ThisAircraft.speed;
    float speed_kmh = speed_kn * 1.852;
    float alt_m = ThisAircraft.altitude;
    float vs_smoothed = get_vs_smoothed_2s();
    float avg_climb_20s = calc_avg_climb_20s();

    navbox1.value = speed_kmh;
    navbox2.value = ThisAircraft.course;
    navbox3.value = alt_m;
    navbox4.value = vs_smoothed;

    /* Calculate glide ratio or show climb average */
    if (avg_climb_20s > 0) {
      /* Climbing: show as negative (encoded) to switch to CLIMB display */
      navbox5.value = -avg_climb_20s;
      strcpy(navbox5.title, "CLIMB");
    } else {
      /* Descending: show glide ratio */
      float speed_m_s = speed_kn * 0.5144;
      float sink_rate = -avg_climb_20s;
      float glide = calc_glide_ratio(speed_m_s, sink_rate);
      navbox5.value = glide;
      strcpy(navbox5.title, "GLIDE");
    }

    EPD_Draw_NavBoxes();
    EPDTimeMarker = millis();
  }
}

void EPD_vario_next()
{
  /* Up button: no-op for now */
}

void EPD_vario_prev()
{
  /* Down button: no-op for now */
}

#endif /* USE_EPAPER */
