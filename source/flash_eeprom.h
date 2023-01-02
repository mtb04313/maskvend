/******************************************************************************
* File Name:   flash_eeprom.h
*
* Description: This file is the public interface of flash_eeprom.c source file
*
* Related Document: See README.md
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


#ifndef SOURCE_FLASH_EEPROM_H_
#define SOURCE_FLASH_EEPROM_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <cmsis_gcc.h>  // for __PACKED_STRUCT

#ifdef __cplusplus
extern "C"
{
#endif


/*-- Public Definitions -------------------------------------------------*/

#define HAVE_FLASH_EEPROM   1 /* 1=enable, 0=disable */

#define MAX_CUSTOM_ACTIVATION_CODES     5
#define CUSTOM_ACTIVATION_CODE_LEN      80

/* Structure to store data that goes into EEPROM */
typedef __PACKED_STRUCT
{
  uint8_t buf[CUSTOM_ACTIVATION_CODE_LEN];
  uint8_t len;
} customActivationCode_t;

typedef __PACKED_STRUCT
{
  customActivationCode_t array[MAX_CUSTOM_ACTIVATION_CODES];
  uint8_t num_items;
} customActivationCodeList_t;


/*-- Public Functions -------------------------------------------------*/

#if (HAVE_FLASH_EEPROM == 1)
bool flash_eeprom_init(void);

// Wi-Fi
bool flash_eeprom_get_wifi_settings(void *buf_p,
                                    size_t buf_size);

bool flash_eeprom_set_wifi_settings(const void *buf_p,
                                    size_t buf_size,
                                    bool ignore_checksum_error);

bool flash_eeprom_reset_wifi_settings(void);

bool flash_eeprom_get_wifi_ssid( char *ssid_p,
                                 size_t ssid_buf_size);

bool flash_eeprom_set_wifi_ssid(const char *ssid_p);

bool flash_eeprom_get_wifi_password( char *password_p,
                                     size_t password_buf_size);

bool flash_eeprom_set_wifi_password(const char *password_p);

bool flash_eeprom_get_wifi_security_type(uint32_t *security_type_p);

bool flash_eeprom_set_wifi_security_type(uint32_t security_type);


// custom activation codes
bool flash_eeprom_get_custom_activation_codes(void *buf_p,
                                              size_t buf_size);

bool flash_eeprom_set_custom_activation_codes(const void *buf_p,
                                              size_t buf_size,
                                              bool ignore_checksum_error);

bool flash_eeprom_reset_custom_activation_codes(void);


// kiosk id
bool flash_eeprom_get_kiosk_id( void *buf_p,
                                size_t buf_size);

bool flash_eeprom_set_kiosk_id( const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error);

bool flash_eeprom_reset_kiosk_id(void);

// time_info
bool flash_eeprom_get_time_info(void *buf_p,
                                size_t buf_size);

bool flash_eeprom_set_time_info(const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error);

bool flash_eeprom_reset_time_info(void);

// gps_info
bool flash_eeprom_get_gps_info( void *buf_p,
                                size_t buf_size);

bool flash_eeprom_set_gps_info( const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error);

bool flash_eeprom_reset_gps_info(void);

bool flash_eeprom_get_gps_location( char *location_p,
                                    size_t location_size);

bool flash_eeprom_set_gps_location(const char *location_p);

// mask qty
bool flash_eeprom_get_mask_qty( int mask_color,
                                void *buf_p,
                                size_t buf_size);

bool flash_eeprom_set_mask_qty( int mask_color,
                                const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error);

bool flash_eeprom_reset_mask_qty(int mask_color);

// url_info
bool flash_eeprom_get_url_info( void *buf_p,
                                size_t buf_size);

bool flash_eeprom_set_url_info( const void *buf_p,
                                size_t buf_size,
                                bool ignore_checksum_error);

bool flash_eeprom_reset_url_info(void);

bool flash_eeprom_get_url_pattern(char *url_p,
                                  size_t url_size);

bool flash_eeprom_set_url_pattern(const char *url_p);

#endif


#ifdef __cplusplus
}
#endif

#endif // SOURCE_FLASH_EEPROM_H_

/* [] END OF FILE */
