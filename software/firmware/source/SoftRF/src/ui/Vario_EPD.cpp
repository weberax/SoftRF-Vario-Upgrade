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

/* Helper function to convert heading degrees to 2-letter compass direction */
static const char* heading_to_compass(float heading) {
  heading = fmod(heading + 360.0, 360.0);  /* Normalize to 0-360 */
  
  if (heading < 11.25) return "N ";
  if (heading < 33.75) return "NE";
  if (heading < 56.25) return "E ";
  if (heading < 78.75) return "SE";
  if (heading < 101.25) return "S ";
  if (heading < 123.75) return "SW";
  if (heading < 146.25) return "W ";
  if (heading < 168.75) return "NW";
  if (heading < 191.25) return "N ";
  if (heading < 213.75) return "NE";
  if (heading < 236.25) return "E ";
  if (heading < 258.75) return "SE";
  if (heading < 281.25) return "S ";
  if (heading < 303.75) return "SW";
  if (heading < 326.25) return "W ";
  if (heading < 348.75) return "NW";
  return "N ";
}

/* Navbox structure for 5-box layout (4 boxes top/middle, 1 wide box bottom) */
static navbox_t navbox1;  /* GS km/h (top-left) */
static navbox_t navbox2;  /* HDG (top-right) */
static navbox_t navbox3;  /* GPS m (middle-left) */
static navbox_t navbox4;  /* vV m/s (middle-right) */
static navbox_t navbox5;  /* GLIDE/CLIMB wide box (bottom, full width) */

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

    /* Box 1 value (GS km/h) - FreeMonoBold18pt7b */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox1.x + 25, navbox1.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox1.x + 75, navbox1.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    snprintf(buf, sizeof(buf), "%.0f", navbox1.value);
    display->print(buf);

    /* Box 2 value (HDG) - FreeMonoBold18pt7b */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox2.x + 15, navbox2.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox2.x + 55, navbox2.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    display->print(heading_to_compass(navbox2.value));

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

    /* Box 3 value (GPS m) - FreeSerifBold12pt7b (SMALLER FONT!) */
    display->setFont(&FreeSerifBold12pt7b);
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox3.x + 5, navbox3.y + 50);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox3.x + 38, navbox3.y + 30);
#endif /* EPD_ASPECT_RATIO_2C1 */
    snprintf(buf, sizeof(buf), "%.0f", navbox3.value);
    display->print(buf);

    /* Box 4 value (vV m/s with sign) - FreeMonoBold18pt7b */
    display->setFont(&FreeMonoBold18pt7b);
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox4.x + 15, navbox4.y + 50);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox4.x + 55, navbox4.y + 30);
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

    /* Box 5 value (GLIDE ratio or CLIMB average) - FreeMonoBold18pt7b */
#if defined(EPD_ASPECT_RATIO_1C1)
    display->setCursor(navbox5.x + 25, navbox5.y + 52);
#endif /* EPD_ASPECT_RATIO_1C1 */
#if defined(EPD_ASPECT_RATIO_2C1)
    display->setCursor(navbox5.x + 55, navbox5.y + 32);
#endif /* EPD_ASPECT_RATIO_2C1 */
    
    /* Display the formatted value */
    snprintf(buf, sizeof(buf), "%.1f", navbox5.value);
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
    /* Read data directly from ThisAircraft (already populated by Baro_loop and GPS) */
    float speed_kmh = ThisAircraft.speed * 1.852;  /* knots to km/h */
    float vs_m_s = ThisAircraft.vs / (_GPS_FEET_PER_METER * 60.0);  /* ft/min to m/s */

    navbox1.value = speed_kmh;
    navbox2.value = ThisAircraft.course;
    navbox3.value = ThisAircraft.pressure_altitude;
    navbox4.value = vs_m_s;
    
    /* Calculate glide ratio or show climb */
    if (vs_m_s > 0) {
      /* Climbing */
      navbox5.value = vs_m_s;
      strcpy(navbox5.title, "CLIMB");
    } else {
      /* Descending or level - calculate glide ratio */
      float speed_m_s = ThisAircraft.speed * 0.5144;  /* knots to m/s */
      float sink_rate = -vs_m_s;
      
      if (sink_rate > 0.1f && speed_m_s > 0.01f) {
        navbox5.value = speed_m_s / sink_rate;  /* L/D ratio */
        strcpy(navbox5.title, "GLIDE");
      } else {
        navbox5.value = 0;
        strcpy(navbox5.title, "GLIDE");
      }
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
