/******************************************************************************
* File Name:   display_screen.c
*
* Description: This file contains declaration code that writes to the display
* screen.
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "feature_config.h"

#include "display_screen.h"
#include "img.h"
#include "GUI_definitions.h"
#include "cy_time_misc.h"
#include "cy_sntp_time.h"

#include "cy_debug.h"
#include "cy_string.h"
#include "cy_wcm.h"
#include "cy_wcm_error.h"
#include "wifi_task.h"
#include "ppp_task.h"
#include "ppp_config.h"
#include "kiosk_config.h"
#include "gps_config.h"
#include "cy_modem.h"
#include "flash_eeprom.h"
#include "cy_status_config.h"
#include "cy_status.h"
#include "url_config.h"
#include "qrcode.h"


/*-- Local Definitions -------------------------------------------------*/

#define TICK_X_COORD_BOTTOM_RIGHT(bmp)  (SCREEN_WIDTH - bmp.XSize - 1)
#define TICK_Y_COORD_BOTTOM_RIGHT(bmp)  (SCREEN_HEIGHT - bmp.YSize - 1)

#define SCAN_QRCODE_MSG   "Scan QR to visit Kiosk page"


/*-- Local Functions -------------------------------------------------*/

static void get_uptime(int *day_p, int *hour_p, int *min_p, int *sec_p)
{
  uint32_t uptime_sec = current_time() / 1000;

  VoidAssert(day_p != NULL);
  VoidAssert(hour_p != NULL);
  VoidAssert(min_p != NULL);
  VoidAssert(sec_p != NULL);

  *day_p = uptime_sec / (3600 * 24);
  uptime_sec %= (3600 * 24);

  *hour_p = uptime_sec / 3600;
  uptime_sec %= 3600;

  *min_p = uptime_sec / 60;
  *sec_p = uptime_sec % 60;
}

static void draw_text_in_fill_rect( const char *text_p,
                    const GUI_FONT *font_p,
                    GUI_COLOR textColor,
                    GUI_COLOR boxFillColor,
                    GUI_RECT *boxRect_p)
{
  VoidAssert(text_p != NULL);
  VoidAssert(font_p != NULL);
  VoidAssert(boxRect_p != NULL);

  GUI_SetColor(boxFillColor);
  GUI_FillRect(boxRect_p->x0, boxRect_p->y0, boxRect_p->x1, boxRect_p->y1);

  GUI_SetBkColor(boxFillColor);
  GUI_SetColor(textColor);
  GUI_SetFont(font_p);

  GUI_DispStringInRect(text_p, boxRect_p, GUI_TA_HCENTER | GUI_TA_VCENTER);
}

#if (FEATURE_WIFI == ENABLE_FEATURE)
static void read_wifi_ip_address( char* buf,
                                  size_t buf_size)
{
  const cy_wcm_ip_address_t* wifi_ip_addr = get_wifi_ip_address();
  VoidAssert(wifi_ip_addr != NULL);

  SNPRINTF( buf,
            buf_size,
            "%d.%d.%d.%d",
            (uint8_t)(wifi_ip_addr->ip.v4),
            (uint8_t)(wifi_ip_addr->ip.v4 >> 8),
            (uint8_t)(wifi_ip_addr->ip.v4 >> 16),
            (uint8_t)(wifi_ip_addr->ip.v4 >> 24));
}
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
static void read_ppp_ip_address(char* buf,
                                size_t buf_size)
{
  const cy_wcm_ip_address_t* ppp_ip_addr = get_ppp_ip_address();
  VoidAssert(ppp_ip_addr != NULL);

  SNPRINTF( (char*)buf,
            buf_size,
            "%d.%d.%d.%d",
            (uint8_t)(ppp_ip_addr->ip.v4),
            (uint8_t)(ppp_ip_addr->ip.v4 >> 8),
            (uint8_t)(ppp_ip_addr->ip.v4 >> 16),
            (uint8_t)(ppp_ip_addr->ip.v4 >> 24));
}
#endif


/*-- Public Functions -------------------------------------------------*/

void draw_clock_face(bool forced_redraw)
{
  GUI_SetBkColor(GUI_BLACK);
  if (forced_redraw) {
    GUI_Clear();
  }

  GUI_SetColor(GUI_WHITE);

  time_t currentTime;
  get_current_time(&currentTime);
  char ctime[9];
  struct tm *myTime = localtime(&currentTime);
  SNPRINTF(ctime, sizeof(ctime), "%02d:%02d:%02d", myTime->tm_hour, myTime->tm_min, myTime->tm_sec);
  GUI_SetFont(&GUI_FontD48); //GUI_SetFont(&GUI_FontD80);
  GUI_DispStringHCenterAt(ctime, SCREEN_CENTER_X_COORD, 70);

  char cdate[20];
  cy_sntp_get_current_date_short( cdate,
                                  sizeof(cdate));

  GUI_SetFont(&GUI_Font24B_1);
  GUI_DispStringHCenterAt(cdate, SCREEN_CENTER_X_COORD, 150); //190);
}


void draw_status(bool forced_redraw)
{
  static char prev_status_str[CY_STATUS_STR_MAX_LEN] = "";
  static int prev_day = 0;

  bool string_has_changed;
  bool day_has_changed;
  char status_str[CY_STATUS_STR_MAX_LEN];
  GUI_COLOR bgColor = GUI_DARKBLUE;
  GUI_COLOR txtColor = GUI_WHITE;

  int day, hour, min, sec;
  day = hour = min = sec = 0;

  GUI_SetBkColor(bgColor);
  GUI_SetColor(txtColor);

  if (forced_redraw) {
    GUI_Clear();
  }

  get_uptime(&day, &hour, &min, &sec);

  day_has_changed = prev_day != day;

  if ((day > 0) && (day_has_changed || forced_redraw)) {
    char day_str[20];
    if (day > 1) {
      SNPRINTF(day_str, sizeof(day_str), "%d days", day);
    }
    else {
      SNPRINTF(day_str, sizeof(day_str), "%d day", day);
    }

    GUI_RECT boxRect = {SCREEN_X_COORD_0, 20, SCREEN_X_COORD_319, 50};

    draw_text_in_fill_rect( day_str,
                            &GUI_Font24B_1,
                            txtColor,
                            bgColor,
                            &boxRect);
    prev_day = day;
  }

  char ctime[9];
  SNPRINTF(ctime, sizeof(ctime), "%02d:%02d:%02d", hour, min, sec);

  GUI_SetFont(&GUI_FontD48);
  GUI_DispStringHCenterAt(ctime, SCREEN_CENTER_X_COORD, 70);

  /* make a copy of the current string so it doesn't change under our feet */
  SNPRINTF(status_str, sizeof(status_str), "%s", get_status_str());
  string_has_changed = strcmp(prev_status_str, status_str);

  if (string_has_changed || forced_redraw) {
    GUI_RECT boxRect = {SCREEN_X_COORD_0, 150, SCREEN_X_COORD_319, 200};

    draw_text_in_fill_rect( status_str,
                            &GUI_Font24B_1,
                            txtColor,
                            bgColor,
                            &boxRect);
    strcpy(prev_status_str, status_str);
  }
}


void draw_kiosk_id(int x_coord, int y_coord)
{
  char string[40];
  int16_t id = DEFAULT_KIOSK_ID;

#if (HAVE_FLASH_EEPROM == 1)
  if (!flash_eeprom_get_kiosk_id(&id, sizeof(id))) {
    DEBUG_PRINT(("flash_eeprom_get_kiosk_id failed\n"));
  }
#endif

  SNPRINTF(string, sizeof(string), "KIOSK: %d", id);
  GUI_DispStringAt(string, x_coord, y_coord);
}

void read_one_ip_address(char* buf,
                         size_t buf_size)
{
#if (FEATURE_PPP == ENABLE_FEATURE)
    read_ppp_ip_address(buf,
                        buf_size);

#elif (FEATURE_WIFI == ENABLE_FEATURE)
    read_wifi_ip_address(buf,
                         buf_size);

#else
    buf[0] = '\0';
#endif
}

void draw_ip_address(int x_coord, int y_coord)
{
    char string[60];

#if (FEATURE_WIFI == ENABLE_FEATURE)
    char ip_address_wifi[IPV4_ADDRESS_STR_LEN] = "0.0.0.0";
    read_wifi_ip_address( ip_address_wifi,
                          sizeof(ip_address_wifi));
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
    char ip_address_ppp[IPV4_ADDRESS_STR_LEN] = "0.0.0.0";
    read_ppp_ip_address(  ip_address_ppp,
                          sizeof(ip_address_ppp));
#endif

#if (FEATURE_WIFI == ENABLE_FEATURE) && (FEATURE_PPP == ENABLE_FEATURE)
    SNPRINTF(string, sizeof(string), "IP: %s / %s",
        ip_address_wifi, ip_address_ppp);

#elif (FEATURE_WIFI == ENABLE_FEATURE)
    SNPRINTF(string, sizeof(string), "IP: %s", ip_address_wifi);

#elif (FEATURE_PPP == ENABLE_FEATURE)
    SNPRINTF(string, sizeof(string), "IP: %s", ip_address_ppp);

#endif

    GUI_DispStringAt(string, x_coord, y_coord);
}

// PPP_APN
void draw_apn(int x_coord, int y_coord)
{
    char string[40];
    SNPRINTF(string, sizeof(string), "APN: %s", PPP_APN);
    GUI_DispStringAt(string, x_coord, y_coord);
}

void draw_gps(int x_coord, int y_coord)
{
    char string[80];
    char location[GPS_INFO_MAX_LEN + 1] = "";

#if (HAVE_FLASH_EEPROM == 1)
    if (!flash_eeprom_get_gps_location( location,
                                        sizeof(location))) {
      DEBUG_PRINT(("flash_eeprom_get_gps_location failed\n"));
    }
#endif

    if (strlen(location) > 0) {
      SNPRINTF(string, sizeof(string), "GPS: %s", location);
    }
    else {
      SNPRINTF(string, sizeof(string), "GPS: %s", DEFAULT_MODEM_GPS_LOCATION);
    }

    GUI_DispStringAt(string, x_coord, y_coord);
}

void draw_operator(int x_coord, int y_coord)
{
    char string[80];
    const char* ptr = cy_modem_get_operator_name();

    if (strlen(ptr) > 0) {
      SNPRINTF(string, sizeof(string), "OPR: %s", ptr);
    }
    else {
      SNPRINTF(string, sizeof(string), "OPR: %s", DEFAULT_MODEM_OPERATOR_NAME);
    }

    GUI_DispStringAt(string, x_coord, y_coord);
}

void draw_ue_system_info(int x_coord, int y_coord)
{
    char string[80];
    const char* ptr = cy_modem_get_ue_system_info();

    if (strlen(ptr) > 0) {
      SNPRINTF(string, sizeof(string), "SYS: %s", ptr);
    }
    else {
      SNPRINTF(string, sizeof(string), "SYS: %s", DEFAULT_MODEM_UE_SYSTEM_INFO);
    }

    GUI_DispStringAt(string, x_coord, y_coord);
}

void draw_iccid(int x_coord, int y_coord)
{
    char string[80];
    const char* ptr = cy_modem_get_iccid();

    if (strlen(ptr) > 0) {
      SNPRINTF(string, sizeof(string), "ICCID: %s", ptr);
    }
    else {
      SNPRINTF(string, sizeof(string), "ICCID: %s", DEFAULT_MODEM_ICCID);
    }

    GUI_DispStringAt(string, x_coord, y_coord);
}

void draw_thick_hort_line(int y_coord, int x1, int x2, int thickness)
{
  GUI_FillRect(x1, y_coord, x2, y_coord + thickness);
}

int num_rssi_bars(int rssi)
{
  if (rssi > SIGNAL_QUALITY_RSSI_BEST_MAX) {
    // invalid
    return 0;
  }
  else if (rssi >= SIGNAL_QUALITY_RSSI_BEST_MIN) {
    return 4;
  }
  else if (rssi >= SIGNAL_QUALITY_RSSI_GOOD_MIN) {
    return 3;
  }
  else if (rssi >= SIGNAL_QUALITY_RSSI_NORM_MIN) {
    return 2;
  }
  return 1;
}

void draw_rssi_icon(void)
{
  int border = 5;
  int barWidth = 10;
  int gapWidth = 2;
  int barHeightIncrement = 5;
  int iconHeight = 4 * barHeightIncrement;
  int iconWidth = (4 * barWidth) + (3 * gapWidth);
  int xStart = SCREEN_X_COORD_319 - border - iconWidth;

  // poor
  GUI_RECT bar1 = { xStart,
                    border + iconHeight - barHeightIncrement,
                    xStart + barWidth,
                    border + iconHeight};

  // norm
  GUI_RECT bar2 = { bar1.x1 + gapWidth,
                    border + iconHeight - (2 * barHeightIncrement),
                    bar1.x1 + gapWidth + barWidth,
                    border + iconHeight};

  // good
  GUI_RECT bar3 = { bar2.x1 + gapWidth,
                    border + iconHeight - (3 * barHeightIncrement),
                    bar2.x1 + gapWidth + barWidth,
                    border + iconHeight};

  // best
  GUI_RECT bar4 = { bar3.x1 + gapWidth,
                    border,
                    bar3.x1 + gapWidth + barWidth,
                    border + iconHeight};

  // 0-31, 99
  int rssi = cy_modem_get_rssi();
  int nBars = num_rssi_bars(rssi);

  DEBUG_PRINT(("rssi = %d (%d)\n", rssi, nBars));

  if (nBars > 3) {
    GUI_FillRect(bar1.x0, bar1.y0, bar1.x1, bar1.y1);
    GUI_FillRect(bar2.x0, bar2.y0, bar2.x1, bar2.y1);
    GUI_FillRect(bar3.x0, bar3.y0, bar3.x1, bar3.y1);
    GUI_FillRect(bar4.x0, bar4.y0, bar4.x1, bar4.y1);
  }
  else if (nBars > 2) {
    GUI_FillRect(bar1.x0, bar1.y0, bar1.x1, bar1.y1);
    GUI_FillRect(bar2.x0, bar2.y0, bar2.x1, bar2.y1);
    GUI_FillRect(bar3.x0, bar3.y0, bar3.x1, bar3.y1);
    GUI_DrawRect(bar4.x0, bar4.y0, bar4.x1, bar4.y1);
  }
  else if (nBars > 1) {
    GUI_FillRect(bar1.x0, bar1.y0, bar1.x1, bar1.y1);
    GUI_FillRect(bar2.x0, bar2.y0, bar2.x1, bar2.y1);
    GUI_DrawRect(bar3.x0, bar3.y0, bar3.x1, bar3.y1);
    GUI_DrawRect(bar4.x0, bar4.y0, bar4.x1, bar4.y1);
  }
  else if (nBars > 0) {
    GUI_FillRect(bar1.x0, bar1.y0, bar1.x1, bar1.y1);
    GUI_DrawRect(bar2.x0, bar2.y0, bar2.x1, bar2.y1);
    GUI_DrawRect(bar3.x0, bar3.y0, bar3.x1, bar3.y1);
    GUI_DrawRect(bar4.x0, bar4.y0, bar4.x1, bar4.y1);
  }
  else {
    GUI_DrawRect(bar1.x0, bar1.y0, bar1.x1, bar1.y1);
    GUI_DrawRect(bar2.x0, bar2.y0, bar2.x1, bar2.y1);
    GUI_DrawRect(bar3.x0, bar3.y0, bar3.x1, bar3.y1);
    GUI_DrawRect(bar4.x0, bar4.y0, bar4.x1, bar4.y1);
  }
}

void network_info_screen(void)
{
  int line_spacing = 30;
  int text_x = 5;
  int y_coord = 20;

  GUI_SetBkColor(GUI_DARKGREEN); //GUI_LIGHTGREEN); //GUI_WHITE);
  GUI_Clear();

  GUI_SetFont(&GUI_Font20B_1); //&GUI_Font24B_1); //&GUI_Font24_1);
  GUI_SetColor(GUI_WHITE); //GUI_BLUE);

  draw_rssi_icon();

  draw_kiosk_id(text_x, y_coord);
  y_coord += line_spacing;

  draw_ip_address(text_x, y_coord);
  y_coord += line_spacing;

#if (FEATURE_PPP == ENABLE_FEATURE)
  draw_apn(text_x, y_coord);
  y_coord += line_spacing;

  draw_iccid(text_x, y_coord);
  y_coord += line_spacing;

  draw_operator(text_x, y_coord);
  y_coord += line_spacing;

  draw_ue_system_info(text_x, y_coord);
  y_coord += line_spacing;

  draw_gps(text_x, y_coord);
#endif
}


void green_tick_on(void)
{
  GUI_DrawBitmap( &bmS_green_tick,
                  TICK_X_COORD_BOTTOM_RIGHT(bmS_green_tick),
                  TICK_Y_COORD_BOTTOM_RIGHT(bmS_green_tick));
}

void green_tick_off(void)
{
  GUI_DrawBitmap( &bmS_no_tick,
                  TICK_X_COORD_BOTTOM_RIGHT(bmS_no_tick),
                  TICK_Y_COORD_BOTTOM_RIGHT(bmS_no_tick));
}

void mask_home_screen(void)
{
  GUI_DrawBitmap(&bmmask_kiosk, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
}

void marketing_screen_1(void)
{
#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
  GUI_DrawBitmap(&bmchip2, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
#endif
}

void marketing_screen_2(void)
{
#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
  GUI_DrawBitmap(&bmmanword2, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
#endif
}

void black_mask(void)
{
  GUI_DrawBitmap(&bmdark_grey_mask_3d, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
}

void green_mask(void)
{
  GUI_DrawBitmap(&bmdark_green_mask_3d, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
}

void blue_mask(void)
{
  GUI_DrawBitmap(&bmdark_blue_mask_3d, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
}

void red_mask(void)
{
  GUI_DrawBitmap(&bmdark_red_mask_3d, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
}

void pink_mask(void)
{
  GUI_DrawBitmap(&bmdark_pink_mask_3d, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
}

void violet_mask(void)
{
  GUI_DrawBitmap(&bmdark_violet_mask_3d, SCREEN_X_COORD_0, SCREEN_Y_COORD_0);
}

void show_text_mask_dispensed(void)
{
  GUI_RECT boxRect = {SCREEN_X_COORD_0, 89, SCREEN_X_COORD_319, 149};

  draw_text_in_fill_rect( "Mask Dispensed",
                          &GUI_Font32B_1,
                          GUI_WHITE,
                          GUI_BLUE, //bgColor,
                          &boxRect);
}

void show_text_out_of_stock(void)
{
  GUI_RECT boxRect = {SCREEN_X_COORD_0, 89, SCREEN_X_COORD_319, 149};

  draw_text_in_fill_rect( "Out Of Stock",
                          &GUI_Font32B_1,
                          GUI_WHITE,
                          GUI_RED,
                          &boxRect);
}

void show_qrcode(bool forced_redraw)
{
  char device_url[URL_INFO_MAX_LEN + 10];
  char url_pattern[URL_INFO_MAX_LEN + 1] = "";
  int16_t id = DEFAULT_KIOSK_ID;

  GUI_COLOR txtColor = GUI_BLACK;
  GUI_COLOR bgColor = GUI_WHITE;
  GUI_COLOR fgColor = GUI_BLACK;

#if (HAVE_FLASH_EEPROM == 1)
  if (!flash_eeprom_get_url_pattern(url_pattern,
                                    sizeof(url_pattern))) {
    DEBUG_PRINT(("flash_eeprom_get_url_pattern failed\n"));
  }

  if (!flash_eeprom_get_kiosk_id(&id, sizeof(id))) {
    DEBUG_PRINT(("flash_eeprom_get_kiosk_id failed\n"));
  }
#endif

  if (strlen(url_pattern) == 0) {
    SNPRINTF(url_pattern, sizeof(url_pattern), "%s", DEFAULT_DEVICE_URL_PATTERN);
  }

  /* append kiosk id to the url pattern */
  SNPRINTF(device_url, sizeof(device_url), "%s%d", url_pattern, id);
  DEBUG_PRINT(("device url: %s\n", device_url));

  // Create the QR code
  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(3)];
  qrcode_initText(&qrcode, qrcodeData, 3, 0, device_url);


  GUI_SetBkColor(bgColor);
  if (forced_redraw) {
    GUI_Clear();
  }


  // Top quiet zone
  //DEBUG_PRINT(("\n\n\n\n"));

  int width = 4;

  /* place the qrcode at the center of the screen */
  int xStart = (SCREEN_WIDTH - qrcode.size * width) / 2;
  int yStart = (SCREEN_HEIGHT - qrcode.size * width) / 2;

  for (uint8_t y = 0; y < qrcode.size; y++) {

    // Left quiet zone
    //DEBUG_PRINT(("        "));

    // Each horizontal module
    for (uint8_t x = 0; x < qrcode.size; x++) {
      // Print each module (UTF-8 \u2588 is a solid block)
      if (qrcode_getModule(&qrcode, x, y)) {
        //DEBUG_PRINT(("\u2588\u2588"));
        GUI_SetColor(fgColor);
      }
      else {
        //DEBUG_PRINT(("  "));
        GUI_SetColor(bgColor);
      }
      GUI_FillRect( x * width + xStart,
                    y * width + yStart,
                    x * width + xStart + width,
                    y * width + yStart + width);
    }
    //DEBUG_PRINT(("\n"));
  }

  // Bottom quiet zone
  //DEBUG_PRINT(("\n\n\n\n"));

  int y1 = yStart * 1 / 4;
  int y2 = yStart * 3 / 4;

  GUI_RECT boxRect1 = {SCREEN_X_COORD_0, y1, SCREEN_X_COORD_319, y2};

  draw_text_in_fill_rect( SCAN_QRCODE_MSG,
                          &GUI_Font24B_1,
                          txtColor,
                          bgColor,
                          &boxRect1);

  y1 = SCREEN_HEIGHT - (yStart * 3 / 4);
  y2 = SCREEN_HEIGHT - (yStart * 1 / 4);

  GUI_RECT boxRect2 = {SCREEN_X_COORD_0, y1, SCREEN_X_COORD_319, y2};

  draw_text_in_fill_rect( device_url,
                          &GUI_Font16B_1,
                          txtColor,
                          bgColor,
                          &boxRect2);
}
