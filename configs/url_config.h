/******************************************************************************
* File Name:   url_config.h
*
* Description: This file has URL related configuration
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

#ifndef SOURCE_URL_CONFIG_H_
#define SOURCE_URL_CONFIG_H_

#include <stdint.h>
#include <cmsis_gcc.h>  // for __PACKED_STRUCT

#ifdef __cplusplus
extern "C"
{
#endif

/* Configure the device URL */
#define DEFAULT_DEVICE_URL_PATTERN    "MY_WEB_SERVER_PATH/?ID="

#define URL_INFO_MAX_LEN              80
#define URL_INFO_MIN_LEN              20

typedef __PACKED_STRUCT
{
  uint8_t buf[URL_INFO_MAX_LEN];  // may not be null-terminated
  uint8_t len;
} ifx_url_info_t;

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_URL_CONFIG_H_ */

/* [] END OF FILE */
