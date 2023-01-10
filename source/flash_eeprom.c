/******************************************************************************
* File Name:   flash_eeprom.c
*
* Description: This file contains code that saves settings to the flash eeprom
*
*
*******************************************************************************
* Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include <string.h>
#include "flash_eeprom.h"  /* for HAVE_FLASH_EEPROM */

#if (HAVE_FLASH_EEPROM == 1)
#include "cy_em_eeprom.h"

#include "cy_debug.h"
#include "cy_misc_types.h"
#include "wifi_config.h"
#include "kiosk_config.h"
#include "time_config.h"
#include "gps_config.h"
#include "mask_config.h"
#include "url_config.h"


/*-- Local Definitions -------------------------------------------------*/

#define STORE_WIFI_SETTINGS             1 /* 1:enable, 0:disable */
#define STORE_CUSTOM_ACTIVATION_CODES   0 /* 1:enable, 0:disable */
#define STORE_KIOSK_ID                  1 /* 1:enable, 0:disable */
#define STORE_TIME_INFO                 1 /* 1:enable, 0:disable */
#define STORE_GPS_INFO                  1 /* 1:enable, 0:disable */
#define STORE_MASK_QTY                  1 /* 1:enable, 0:disable */
#define STORE_URL_INFO                  1 /* 1:enable, 0:disable */

/* Logical Start of Emulated EEPROM in bytes. */
#define LOGICAL_EEPROM_START                    (0u)

#define WIFI_DATA_START                         LOGICAL_EEPROM_START
#if (STORE_WIFI_SETTINGS)
#define WIFI_DATA_SIZE                          sizeof(wifi_details_t)
#else
#define WIFI_DATA_SIZE                          0
#endif

#define CUSTOM_ACTIVATION_CODES_DATA_START      (WIFI_DATA_START + WIFI_DATA_SIZE)
#if (STORE_CUSTOM_ACTIVATION_CODES)
#define CUSTOM_ACTIVATION_CODES_DATA_SIZE       sizeof(customActivationCodeList_t)
#else
#define CUSTOM_ACTIVATION_CODES_DATA_SIZE       0
#endif

#define KIOSK_ID_DATA_START                     (CUSTOM_ACTIVATION_CODES_DATA_START + CUSTOM_ACTIVATION_CODES_DATA_SIZE)
#if (STORE_KIOSK_ID)
#define KIOSK_ID_DATA_SIZE                      sizeof(int16_t)
#else
#define KIOSK_ID_DATA_SIZE                      0
#endif

#define TIME_INFO_DATA_START                    (KIOSK_ID_DATA_START + KIOSK_ID_DATA_SIZE)
#if (STORE_TIME_INFO)
#define TIME_INFO_DATA_SIZE                     sizeof(cy_time_info_t)
#else
#define TIME_INFO_DATA_SIZE                     0
#endif

#define GPS_INFO_DATA_START                     (TIME_INFO_DATA_START + TIME_INFO_DATA_SIZE)
#if (STORE_GPS_INFO)
#define GPS_INFO_DATA_SIZE                      sizeof(ifx_gps_info_t)
#else
#define GPS_INFO_DATA_SIZE                      0
#endif

#if (STORE_MASK_QTY)
#define MASK_QTY_DATA_SIZE                      sizeof(int16_t)
#else
#define MASK_QTY_DATA_SIZE                      0
#endif

#define MASK_V_QTY_DATA_START                   (GPS_INFO_DATA_START + GPS_INFO_DATA_SIZE)
#define MASK_P_QTY_DATA_START                   (MASK_V_QTY_DATA_START + MASK_QTY_DATA_SIZE)
#define MASK_R_QTY_DATA_START                   (MASK_P_QTY_DATA_START + MASK_QTY_DATA_SIZE)
#define MASK_B_QTY_DATA_START                   (MASK_R_QTY_DATA_START + MASK_QTY_DATA_SIZE)
#define MASK_K_QTY_DATA_START                   (MASK_B_QTY_DATA_START + MASK_QTY_DATA_SIZE)

#define URL_INFO_DATA_START                     (MASK_K_QTY_DATA_START + MASK_QTY_DATA_SIZE)
#if (STORE_URL_INFO)
#define URL_INFO_DATA_SIZE                      sizeof(ifx_url_info_t)
#else
#define URL_INFO_DATA_SIZE                      0
#endif


/* EEPROM Configuration details. */
#define SIMPLE_MODE             (0u)
#define EEPROM_SIZE             (1024u)  //was (512u)
#define BLOCKING_WRITE          (1u)
#define REDUNDANT_COPY          (1u)
#define WEAR_LEVELLING_FACTOR   (2u)

/* Set the macro FLASH_REGION_TO_USE to either USER_FLASH or
 * EMULATED_EEPROM_FLASH to specify the region of the flash used for
 * emulated EEPROM.
 */
#define USER_FLASH              (0u)
#define EMULATED_EEPROM_FLASH   (1u)
#define FLASH_REGION_TO_USE     EMULATED_EEPROM_FLASH

#if (FLASH_REGION_TO_USE)
CY_SECTION(".cy_em_eeprom")
#endif /* #if(FLASH_REGION_TO_USE) */
CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)


/*-- Local Data -------------------------------------------------*/

const static uint8_t s_EepromStorage[CY_EM_EEPROM_GET_PHYSICAL_SIZE(EEPROM_SIZE,
                                                                    SIMPLE_MODE,
                                                                    WEAR_LEVELLING_FACTOR,
                                                                    REDUNDANT_COPY)] = {0u};

/* EEPROM configuration and context structure. */
static cy_stc_eeprom_config_t s_Em_EEPROM_config =
{
  .eepromSize = EEPROM_SIZE,
  .blockingWrite = BLOCKING_WRITE,
  .redundantCopy = REDUNDANT_COPY,
  .wearLevelingFactor = WEAR_LEVELLING_FACTOR,
  .userFlashStartAddr = (uint32_t)s_EepromStorage,
};

static cy_stc_eeprom_context_t s_Em_EEPROM_context;

/* Structure used for finding mask data start location */
typedef struct
{
  int color;
  uint32_t data_start;
} flash_eeprom_mask_data_t;

static flash_eeprom_mask_data_t s_mask_data_start[] = {
  {MASK_COLOR_VIOLET, MASK_V_QTY_DATA_START},
  {MASK_COLOR_PINK,   MASK_P_QTY_DATA_START},
  {MASK_COLOR_RED,    MASK_R_QTY_DATA_START},
  {MASK_COLOR_BLUE,   MASK_B_QTY_DATA_START},
  {MASK_COLOR_BLACK,  MASK_K_QTY_DATA_START},
};


/*-- Local Functions -------------------------------------------------*/

// return true if the start location is found
static bool get_mask_start_location(int mask_color, uint32_t *start_location_p)
{
  int i;
  ReturnAssert(start_location_p != NULL, false);

  for (i = 0; i < sizeof(s_mask_data_start)/sizeof(s_mask_data_start[0]); i++) {
    if (s_mask_data_start[i].color == mask_color) {
      *start_location_p = s_mask_data_start[i].data_start;
      return true;
    }
  }

  return false;
}

static bool verify_wifi_settings(void)
{
#if (STORE_WIFI_SETTINGS)
  bool result = false;

  wifi_details_t wifi_details;
  memset(&wifi_details, 0, sizeof(wifi_details));

  if (flash_eeprom_get_wifi_settings( &wifi_details,
                                      sizeof(wifi_details))) {
    if ((wifi_details.security != WIFI_SECURITY) ||
        (wifi_details.ssid_len == 0) ||
        (wifi_details.ssid_len > sizeof(wifi_details.wifi_ssid)) ||
        (wifi_details.password_len > sizeof(wifi_details.wifi_password)) ||
        ((wifi_details.password_len == 0) && (wifi_details.wifi_password[0] != '\0'))) {

      // invalid wifi details, reset to default
      result = flash_eeprom_reset_wifi_settings();
      if (result) {
        DEBUG_PRINT(("Default WIFI settings saved to eeprom\n"));
      }
      else {
        DEBUG_PRINT(("Error: flash_eeprom_reset_wifi_settings failed\n"));
      }
    }
    else {
      result = true;
      DEBUG_PRINT(("WIFI Settings read from eeprom\n"));
    }
  }
  else {
    DEBUG_PRINT(("Error: flash_eeprom_get_wifi_settings failed\n"));
  }

  return result;

#else
  return true;
#endif
}

static bool verify_custom_activation_codes(void)
{
#if (STORE_CUSTOM_ACTIVATION_CODES)
  bool result = false;

  customActivationCodeList_t codes;
  memset(&codes, 0, sizeof(codes));

  if (flash_eeprom_get_custom_activation_codes( &codes,
                                                sizeof(codes))) {
    result = true;
    if (codes.num_items <= ARRAY_SIZE(codes.array)) {
      int i;
      int found = 0;

      for (i = 0; i < codes.num_items; i++) {
        if ((codes.array[i].len > sizeof(codes.array[i].buf)) ||
            ((codes.array[i].len == 0) && (codes.array[i].buf[0] != '\0'))) {
          result = false;
          break;
        }
        else if (codes.array[i].len > 0){
          found++;
        }
      }

      if (found != codes.num_items) {
        result = false;
      }
    }
    else {
      result = false;
    }

    if (result) {
      DEBUG_PRINT(("Custom activation codes read from eeprom\n"));
    }
    else {
      // invalid data, reset to default
      result = flash_eeprom_reset_custom_activation_codes();
      if (result) {
        DEBUG_PRINT(("Default custom activation codes saved to eeprom\n"));
      }
      else {
        DEBUG_PRINT(("Error: flash_eeprom_reset_custom_activation_codes failed\n"));
      }
    }
  }
  else {
    DEBUG_PRINT(("Error: flash_eeprom_get_custom_activation_codes failed\n"));
  }

  return result;

#else
  return true;
#endif
}

static bool verify_kiosk_id(void)
{
#if (STORE_KIOSK_ID)
  bool result = false;

  int16_t id = 0;

  if (flash_eeprom_get_kiosk_id(&id,
                                sizeof(id))) {
    if ((id < MIN_KIOSK_ID) ||
        (id > MAX_KIOSK_ID)) {

      // invalid id, reset to default
      result = flash_eeprom_reset_kiosk_id();
      if (result) {
        DEBUG_PRINT(("Default Kiosk ID saved to eeprom\n"));
      }
      else {
        DEBUG_PRINT(("Error: flash_eeprom_reset_kiosk_id failed\n"));
      }
    }
    else {
      result = true;
      DEBUG_PRINT(("Kiosk ID read from eeprom\n"));
    }
  }
  else {
    DEBUG_PRINT(("Error: flash_eeprom_get_kiosk_id failed\n"));
  }

  return result;

#else
  return true;
#endif
}

static bool verify_time_info(void)
{
#if (STORE_TIME_INFO)
  bool result = false;

  cy_time_info_t time_info;
  memset(&time_info, 0, sizeof(time_info));

  if (flash_eeprom_get_time_info( &time_info,
                                  sizeof(time_info))) {

    if ((time_info.timestamp < MIN_TIMESTAMP_SEC) ||
        (time_info.timestamp > MAX_TIMESTAMP_SEC) ||
        (time_info.timezone_diff < MIN_TIMEZONE_DIFF) ||
        (time_info.timezone_diff > MAX_TIMEZONE_DIFF)) {

      // invalid time_info, reset to default
      result = flash_eeprom_reset_time_info();
      if (result) {
        DEBUG_PRINT(("Default time_info saved to eeprom\n"));
      }
      else {
        DEBUG_PRINT(("Error: flash_eeprom_reset_time_info failed\n"));
      }
    }
    else {
      result = true;
      DEBUG_PRINT(("time_info read from eeprom\n"));
    }
  }
  else {
    DEBUG_PRINT(("Error: flash_eeprom_get_time_info failed\n"));
  }

  return result;

#else
  return true;
#endif
}

static bool verify_gps_info(void)
{
#if (STORE_GPS_INFO)
  bool result = false;

  ifx_gps_info_t gps_info;
  memset(&gps_info, 0, sizeof(gps_info));

  if (flash_eeprom_get_gps_info(&gps_info,
                                sizeof(gps_info))) {

    char temp[sizeof(gps_info.buf) + 1];
    int temp_len;

    // As gps_info.buf may not be null-terminated,
    // we should make a temp copy which is
    memset(temp, 0, sizeof(temp));
    memcpy(temp, gps_info.buf, sizeof(gps_info.buf));
    temp_len = strlen(temp);

    if ((gps_info.len == 0) ||
        (gps_info.len != temp_len)) {

      // invalid gps_info, reset to default
      result = flash_eeprom_reset_gps_info();
      if (result) {
        DEBUG_PRINT(("Default gps_info saved to eeprom\n"));
      }
      else {
        DEBUG_PRINT(("Error: flash_eeprom_reset_gps_info failed\n"));
      }
    }
    else {
      result = true;
      DEBUG_PRINT(("gps_info read from eeprom\n"));
    }
  }
  else {
    DEBUG_PRINT(("Error: flash_eeprom_get_gps_info failed\n"));
  }

  return result;

#else
  return true;
#endif
}

static bool verify_mask_qty(int mask_color)
{
#if (STORE_MASK_QTY)
  bool result = false;

  int16_t qty = 0;

  if (flash_eeprom_get_mask_qty(mask_color,
                                &qty,
                                sizeof(qty))) {
    if ((qty < MIN_MASK_QTY) ||
        (qty > MAX_MASK_QTY)) {

      // invalid qty, reset to default
      result = flash_eeprom_reset_mask_qty(mask_color);
      if (result) {
        DEBUG_PRINT(("Default Mask %c qty saved to eeprom\n", mask_color));
      }
      else {
        DEBUG_PRINT(("Error: flash_eeprom_reset_mask_qty failed\n"));
      }
    }
    else {
      result = true;
      DEBUG_PRINT(("Mask %c qty read from eeprom\n", mask_color));
    }
  }
  else {
    DEBUG_PRINT(("Error: flash_eeprom_get_mask_qty failed\n"));
  }

  return result;

#else
  return true;
#endif
}

static bool verify_all_mask_qty(void)
{
#if (STORE_MASK_QTY)
  int i;

  for (i = 0; i < sizeof(s_mask_data_start)/sizeof(s_mask_data_start[0]); i++) {
    if (!verify_mask_qty(s_mask_data_start[i].color)) {
      return false;
    }
  }
  return true;

#else
  return true;
#endif
}

static bool verify_url_info(void)
{
#if (STORE_URL_INFO)
  bool result = false;

  ifx_url_info_t url_info;
  memset(&url_info, 0, sizeof(url_info));

  if (flash_eeprom_get_url_info(&url_info,
                                sizeof(url_info))) {

    char temp[sizeof(url_info.buf) + 1];
    int temp_len;

    // As url_info.buf may not be null-terminated,
    // we should make a temp copy which is
    memset(temp, 0, sizeof(temp));
    memcpy(temp, url_info.buf, sizeof(url_info.buf));
    temp_len = strlen(temp);

    if ((url_info.len == 0) ||
        (url_info.len != temp_len)) {

      // invalid url_info, reset to default
      result = flash_eeprom_reset_url_info();
      if (result) {
        DEBUG_PRINT(("Default url_info saved to eeprom\n"));
      }
      else {
        DEBUG_PRINT(("Error: flash_eeprom_reset_url_info failed\n"));
      }
    }
    else {
      result = true;
      DEBUG_PRINT(("url_info read from eeprom\n"));
    }
  }
  else {
    DEBUG_PRINT(("Error: flash_eeprom_get_url_info failed\n"));
  }

  return result;

#else
  return true;
#endif
}

/*-- Public Functions -------------------------------------------------*/

bool flash_eeprom_init(void)
{
  bool result = false;

  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  /* Initialize the EMEEPROM. */
  status = Cy_Em_EEPROM_Init( &s_Em_EEPROM_config,
                              &s_Em_EEPROM_context);

  if (status == CY_EM_EEPROM_SUCCESS) {
    result =  verify_wifi_settings() &&
              verify_custom_activation_codes() &&
              verify_kiosk_id() &&
              verify_time_info() &&
              verify_gps_info() &&
              verify_all_mask_qty() &&
              verify_url_info();
  }

  return result;
}

// wifi settings
bool flash_eeprom_get_wifi_settings(void *buf_p,
                                    size_t buf_size)
{
#if (STORE_WIFI_SETTINGS)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == sizeof(wifi_details_t), false);

  status = Cy_Em_EEPROM_Read( WIFI_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_set_wifi_settings(const void *buf_p,
                                    size_t buf_size,
                                    bool ignore_checksum_error)
{
#if (STORE_WIFI_SETTINGS)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == sizeof(wifi_details_t), false);

  /* Write data to EEPROM. */
  status = Cy_Em_EEPROM_Write(WIFI_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  if (status != CY_EM_EEPROM_SUCCESS) {
      DEBUG_PRINT(("Error: Cy_Em_EEPROM_Write returned status = 0x%08x\n", status));

      if (ignore_checksum_error && (status == CY_EM_EEPROM_BAD_CHECKSUM)) {
          status = CY_EM_EEPROM_SUCCESS;
      }
  }

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_reset_wifi_settings(void)
{
#if (STORE_WIFI_SETTINGS)
  wifi_details_t wifi_details;
  memset(&wifi_details, 0, sizeof(wifi_details));

  wifi_details.security = WIFI_SECURITY;

  ReturnAssert(sizeof(WIFI_SSID) <= sizeof(wifi_details.wifi_ssid), false);
  memcpy(wifi_details.wifi_ssid, WIFI_SSID, sizeof(WIFI_SSID));

  ReturnAssert(sizeof(WIFI_PASSWORD) <= sizeof(wifi_details.wifi_password), false);
  memcpy(wifi_details.wifi_password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));

  wifi_details.ssid_len = strlen(WIFI_SSID);
  wifi_details.password_len = strlen(WIFI_PASSWORD);

  return (flash_eeprom_set_wifi_settings( &wifi_details,
                                          sizeof(wifi_details),
                                          true));

#else
  return false;
#endif
}

bool flash_eeprom_get_wifi_ssid( char *ssid_p,
                                 size_t ssid_buf_size)
{
#if (STORE_WIFI_SETTINGS)
  wifi_details_t wifi_details;
  bool result = false;

  ReturnAssert((ssid_p != NULL), false);

  memset(&wifi_details, 0, sizeof(wifi_details));

  if (flash_eeprom_get_wifi_settings( &wifi_details,
                                      sizeof(wifi_details))) {

    ReturnAssert((wifi_details.ssid_len < ssid_buf_size), false);
    memset(ssid_p, 0, ssid_buf_size);
    memcpy(ssid_p, wifi_details.wifi_ssid, wifi_details.ssid_len);

    ReturnAssert((wifi_details.ssid_len == strlen(ssid_p)), false);
    result = true;
  }
  else {
    DEBUG_PRINT(("flash_eeprom_get_wifi_settings failed\n"));
  }

  return result;

#else
  return false;
#endif
}

bool flash_eeprom_set_wifi_ssid(const char *ssid_p)
{
#if (STORE_WIFI_SETTINGS)
  wifi_details_t wifi_details;
  uint8_t len;
  bool result;

  ReturnAssert((ssid_p != NULL), false);

  len = strlen(ssid_p);
  ReturnAssert((len <= sizeof(wifi_details.wifi_ssid)), false);

  /* first read, then override ssid and ssid_len with new ones */
  memset(&wifi_details, 0, sizeof(wifi_details));

  result = flash_eeprom_get_wifi_settings(&wifi_details,
                                          sizeof(wifi_details));

  if (result) {
    memcpy(wifi_details.wifi_ssid, ssid_p, len);
    wifi_details.ssid_len = len;

    result = flash_eeprom_set_wifi_settings(&wifi_details,
                                            sizeof(wifi_details),
                                            false);
  }

  return result;

#else
  return false;
#endif
}

bool flash_eeprom_get_wifi_password( char *password_p,
                                     size_t password_buf_size)
{
#if (STORE_WIFI_SETTINGS)
  wifi_details_t wifi_details;
  bool result = false;

  ReturnAssert((password_p != NULL), false);

  memset(&wifi_details, 0, sizeof(wifi_details));

  if (flash_eeprom_get_wifi_settings( &wifi_details,
                                      sizeof(wifi_details))) {

    ReturnAssert((wifi_details.password_len < password_buf_size), false);
    memset(password_p, 0, password_buf_size);
    memcpy(password_p, wifi_details.wifi_password, wifi_details.password_len);

    ReturnAssert((wifi_details.password_len == strlen(password_p)), false);
    result = true;
  }
  else {
    DEBUG_PRINT(("flash_eeprom_get_wifi_settings failed\n"));
  }

  return result;

#else
  return false;
#endif
}

bool flash_eeprom_set_wifi_password(const char *password_p)
{
#if (STORE_WIFI_SETTINGS)
  wifi_details_t wifi_details;
  uint8_t len;
  bool result;

  ReturnAssert((password_p != NULL), false);

  len = strlen(password_p);
  ReturnAssert((len <= sizeof(wifi_details.wifi_password)), false);

  /* first read, then override password and password_len with new ones */
  memset(&wifi_details, 0, sizeof(wifi_details));

  result = flash_eeprom_get_wifi_settings(&wifi_details,
                                          sizeof(wifi_details));

  if (result) {
    memcpy(wifi_details.wifi_password, password_p, len);
    wifi_details.password_len = len;

    result = flash_eeprom_set_wifi_settings(&wifi_details,
                                            sizeof(wifi_details),
                                            false);
  }

  return result;

#else
  return false;
#endif
}

bool flash_eeprom_get_wifi_security_type(uint32_t *security_type_p)
{
#if (STORE_WIFI_SETTINGS)
  wifi_details_t wifi_details;
  bool result = false;

  ReturnAssert((security_type_p != NULL), false);

  memset(&wifi_details, 0, sizeof(wifi_details));

  if (flash_eeprom_get_wifi_settings( &wifi_details,
                                      sizeof(wifi_details))) {
    *security_type_p = wifi_details.security;
    result = true;
  }
  else {
    DEBUG_PRINT(("flash_eeprom_get_wifi_settings failed\n"));
  }

  return result;

#else
  return false;
#endif
}

bool flash_eeprom_set_wifi_security_type(uint32_t security_type)
{
#if (STORE_WIFI_SETTINGS)
  wifi_details_t wifi_details;
  bool result;

  /* first read, then override security with new value */
  memset(&wifi_details, 0, sizeof(wifi_details));

  result = flash_eeprom_get_wifi_settings(&wifi_details,
                                          sizeof(wifi_details));

  if (result) {
    wifi_details.security = security_type;

    result = flash_eeprom_set_wifi_settings(&wifi_details,
                                            sizeof(wifi_details),
                                            false);
  }

  return result;

#else
  return false;
#endif
}


// custom activation codes
bool flash_eeprom_get_custom_activation_codes(void *buf_p,
                                              size_t buf_size)
{
#if (STORE_CUSTOM_ACTIVATION_CODES)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == sizeof(customActivationCodeList_t), false);

  status = Cy_Em_EEPROM_Read( CUSTOM_ACTIVATION_CODES_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_set_custom_activation_codes(const void *buf_p,
                                              size_t buf_size,
                                              bool ignore_checksum_error)
{
#if (STORE_CUSTOM_ACTIVATION_CODES)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == sizeof(customActivationCodeList_t), false);

  /* Write data to EEPROM. */
  status = Cy_Em_EEPROM_Write(CUSTOM_ACTIVATION_CODES_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  if (status != CY_EM_EEPROM_SUCCESS) {
      DEBUG_PRINT(("Error: Cy_Em_EEPROM_Write returned status = 0x%08x\n", status));

      if (ignore_checksum_error && (status == CY_EM_EEPROM_BAD_CHECKSUM)) {
          status = CY_EM_EEPROM_SUCCESS;
      }
  }

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_reset_custom_activation_codes(void)
{
#if (STORE_CUSTOM_ACTIVATION_CODES)
  customActivationCodeList_t codes;
  memset(&codes, 0, sizeof(codes));

  return (flash_eeprom_set_custom_activation_codes( &codes,
                                                    sizeof(codes),
                                                    true));

#else
  return false;
#endif
}

// kiosk id
bool flash_eeprom_get_kiosk_id( void *buf_p,
                                size_t buf_size)
{
#if (STORE_KIOSK_ID)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == KIOSK_ID_DATA_SIZE, false);

  status = Cy_Em_EEPROM_Read( KIOSK_ID_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_set_kiosk_id( const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error)
{
#if (STORE_KIOSK_ID)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == KIOSK_ID_DATA_SIZE, false);

  /* Write data to EEPROM. */
  status = Cy_Em_EEPROM_Write(KIOSK_ID_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  if (status != CY_EM_EEPROM_SUCCESS) {
      DEBUG_PRINT(("Error: Cy_Em_EEPROM_Write returned status = 0x%08x\n", status));

      if (ignore_checksum_error && (status == CY_EM_EEPROM_BAD_CHECKSUM)) {
          status = CY_EM_EEPROM_SUCCESS;
      }
  }

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_reset_kiosk_id(void)
{
#if (STORE_KIOSK_ID)
  int16_t id = DEFAULT_KIOSK_ID;

  return (flash_eeprom_set_kiosk_id(&id,
                                    sizeof(id),
                                    true));

#else
  return false;
#endif
}

// time info
bool flash_eeprom_get_time_info(void *buf_p,
                                size_t buf_size)
{
#if (STORE_TIME_INFO)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == TIME_INFO_DATA_SIZE, false);

  status = Cy_Em_EEPROM_Read( TIME_INFO_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_set_time_info(const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error)
{
#if (STORE_TIME_INFO)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == TIME_INFO_DATA_SIZE, false);

  /* Write data to EEPROM. */
  status = Cy_Em_EEPROM_Write(TIME_INFO_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  if (status != CY_EM_EEPROM_SUCCESS) {
      DEBUG_PRINT(("Error: Cy_Em_EEPROM_Write returned status = 0x%08x\n", status));

      if (ignore_checksum_error && (status == CY_EM_EEPROM_BAD_CHECKSUM)) {
          status = CY_EM_EEPROM_SUCCESS;
      }
  }

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_reset_time_info(void)
{
#if (STORE_TIME_INFO)
  cy_time_info_t time_info;
  memset(&time_info, 0, sizeof(time_info));

  time_info.timestamp = DEFAULT_TIMESTAMP_SEC;
  time_info.timezone_diff = DEFAULT_TIMEZONE_DIFF;

  return (flash_eeprom_set_time_info( &time_info,
                                      sizeof(time_info),
                                      true));

#else
  return false;
#endif
}

// gps info
bool flash_eeprom_get_gps_info( void *buf_p,
                                size_t buf_size)
{
#if (STORE_GPS_INFO)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == GPS_INFO_DATA_SIZE, false);

  status = Cy_Em_EEPROM_Read( GPS_INFO_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_set_gps_info( const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error)
{
#if (STORE_GPS_INFO)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == GPS_INFO_DATA_SIZE, false);

  /* Write data to EEPROM. */
  status = Cy_Em_EEPROM_Write(GPS_INFO_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  if (status != CY_EM_EEPROM_SUCCESS) {
      DEBUG_PRINT(("Error: Cy_Em_EEPROM_Write returned status = 0x%08x\n", status));

      if (ignore_checksum_error && (status == CY_EM_EEPROM_BAD_CHECKSUM)) {
          status = CY_EM_EEPROM_SUCCESS;
      }
  }

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_reset_gps_info(void)
{
#if (STORE_GPS_INFO)
  ifx_gps_info_t gps_info;
  uint8_t len = strlen(DEFAULT_GPS_COORDS);

  ReturnAssert(len <= sizeof(gps_info.buf), false);

  memset(&gps_info, 0, sizeof(gps_info));
  memcpy(gps_info.buf, DEFAULT_GPS_COORDS, len);
  gps_info.len = len;

  return (flash_eeprom_set_gps_info(&gps_info,
                                    sizeof(gps_info),
                                    true));

#else
  return false;
#endif
}

bool flash_eeprom_get_gps_location( char *location_p,
                                    size_t location_size)
{
#if (STORE_GPS_INFO)
  bool result = false;
  ifx_gps_info_t gps_info;

  memset(&gps_info, 0, sizeof(gps_info));
  if (flash_eeprom_get_gps_info(&gps_info,
                                sizeof(gps_info))) {

    ReturnAssert((gps_info.len < location_size), false);
    memset(location_p, 0, location_size);
    memcpy(location_p, gps_info.buf, gps_info.len);

    ReturnAssert((gps_info.len == strlen(location_p)), false);
    result = true;
  }
  else {
    DEBUG_PRINT(("flash_eeprom_get_gps_info failed\n"));
  }

  return result;

#else
  return false;
#endif
}

bool flash_eeprom_set_gps_location(const char *location_p)
{
#if (STORE_GPS_INFO)

  ifx_gps_info_t gps_info;
  uint8_t len = strlen(location_p);

  ReturnAssert((len <= sizeof(gps_info.buf)), false);

  memset(&gps_info, 0, sizeof(gps_info));
  memcpy(gps_info.buf, location_p, len);
  gps_info.len = len;

  return (flash_eeprom_set_gps_info(&gps_info,
                                    sizeof(gps_info),
                                    false));

#else
  return false;
#endif
}

// mask qty
bool flash_eeprom_get_mask_qty( int mask_color,
                                void *buf_p,
                                size_t buf_size)
{
#if (STORE_MASK_QTY)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == MASK_QTY_DATA_SIZE, false);

  uint32_t start_location = 0;
  bool result = get_mask_start_location(mask_color, &start_location);
  ReturnAssert(result, false);

  status = Cy_Em_EEPROM_Read( start_location,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_set_mask_qty( int mask_color,
                                const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error)
{
#if (STORE_MASK_QTY)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == MASK_QTY_DATA_SIZE, false);

  uint32_t start_location = 0;
  bool result = get_mask_start_location(mask_color, &start_location);
  ReturnAssert(result, false);

  /* Write data to EEPROM. */
  status = Cy_Em_EEPROM_Write(start_location,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  if (status != CY_EM_EEPROM_SUCCESS) {
      DEBUG_PRINT(("Error: Cy_Em_EEPROM_Write returned status = 0x%08x\n", status));

      if (ignore_checksum_error && (status == CY_EM_EEPROM_BAD_CHECKSUM)) {
          status = CY_EM_EEPROM_SUCCESS;
      }
  }

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}


bool flash_eeprom_reset_mask_qty(int mask_color)
{
#if (STORE_MASK_QTY)
  int16_t qty = DEFAULT_MASK_QTY;

  uint32_t start_location = 0;
  bool result = get_mask_start_location(mask_color, &start_location);
  ReturnAssert(result, false);

  return (flash_eeprom_set_mask_qty(mask_color,
                                    &qty,
                                    sizeof(qty),
                                    true));

#else
  return false;
#endif
}


// url info
bool flash_eeprom_get_url_info( void *buf_p,
                                size_t buf_size)
{
#if (STORE_URL_INFO)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == URL_INFO_DATA_SIZE, false);

  status = Cy_Em_EEPROM_Read( URL_INFO_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_set_url_info( const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error)
{
#if (STORE_URL_INFO)
  /* Return status for EEPROM. */
  cy_en_em_eeprom_status_t status;

  ReturnAssert(buf_p != NULL, false);
  ReturnAssert(buf_size == URL_INFO_DATA_SIZE, false);

  /* Write data to EEPROM. */
  status = Cy_Em_EEPROM_Write(URL_INFO_DATA_START,
                              buf_p,
                              buf_size,
                              &s_Em_EEPROM_context);

  if (status != CY_EM_EEPROM_SUCCESS) {
      DEBUG_PRINT(("Error: Cy_Em_EEPROM_Write returned status = 0x%08x\n", status));

      if (ignore_checksum_error && (status == CY_EM_EEPROM_BAD_CHECKSUM)) {
          status = CY_EM_EEPROM_SUCCESS;
      }
  }

  return (status == CY_EM_EEPROM_SUCCESS)? true : false;

#else
  return false;
#endif
}

bool flash_eeprom_reset_url_info(void)
{
#if (STORE_URL_INFO)
  ifx_url_info_t url_info;
  uint8_t len = strlen(DEFAULT_DEVICE_URL_PATTERN);

  ReturnAssert(len <= sizeof(url_info.buf), false);

  memset(&url_info, 0, sizeof(url_info));
  memcpy(url_info.buf, DEFAULT_DEVICE_URL_PATTERN, len);
  url_info.len = len;

  return (flash_eeprom_set_url_info(&url_info,
                                    sizeof(url_info),
                                    true));

#else
  return false;
#endif
}

bool flash_eeprom_get_url_pattern(char *url_p,
                                  size_t url_size)
{
#if (STORE_URL_INFO)
  bool result = false;
  ifx_url_info_t url_info;

  memset(&url_info, 0, sizeof(url_info));
  if (flash_eeprom_get_url_info(&url_info,
                                sizeof(url_info))) {

    ReturnAssert((url_info.len < url_size), false);
    memset(url_p, 0, url_size);
    memcpy(url_p, url_info.buf, url_info.len);

    ReturnAssert((url_info.len == strlen(url_p)), false);
    result = true;
  }
  else {
    DEBUG_PRINT(("flash_eeprom_get_url_info failed\n"));
  }

  return result;

#else
  return false;
#endif
}

bool flash_eeprom_set_url_pattern(const char *url_p)
{
#if (STORE_URL_INFO)

  ifx_url_info_t url_info;
  uint8_t len = strlen(url_p);

  ReturnAssert((len <= sizeof(url_info.buf)), false);

  memset(&url_info, 0, sizeof(url_info));
  memcpy(url_info.buf, url_p, len);
  url_info.len = len;

  return (flash_eeprom_set_url_info(&url_info,
                                    sizeof(url_info),
                                    false));

#else
  return false;
#endif
}

#endif
