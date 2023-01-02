/*******************************************************************************
* File Name: feature_config.h
*
* Description: This file defines whether features are enabled / disabled
*
* Related Document: See README.md
*
*
*********************************************************************************
 Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef SOURCE_FEATURE_CONFIG_H_
#define SOURCE_FEATURE_CONFIG_H_

#ifdef __cplusplus
extern "C"
{
#endif


/*-- Public Definitions -------------------------------------------------*/

#define ENABLE_FEATURE                  1
#define DISABLE_FEATURE                 2

// core features
#if 1
  // enable Cellular; disable WIFI
  #define FEATURE_PPP                     ENABLE_FEATURE
  #define FEATURE_WIFI                    DISABLE_FEATURE
#else
  // enable WIFI; disable Cellular
  #define FEATURE_PPP                     DISABLE_FEATURE
  #define FEATURE_WIFI                    ENABLE_FEATURE
#endif

#if 1
#define FEATURE_CONSOLE                 ENABLE_FEATURE
#else
#define FEATURE_CONSOLE                 DISABLE_FEATURE
#endif

#if defined (TARGET_APP_CY8CEVAL_062S2_LAI_4373M2) // CY8CEVAL Eval Kit has on-board Trust-M
#define FEATURE_TRUST_M                 ENABLE_FEATURE
#else
#define FEATURE_TRUST_M                 DISABLE_FEATURE
#endif

#define FEATURE_ESIM_LPA_MENU           DISABLE_FEATURE
#define FEATURE_APPS                    ENABLE_FEATURE
#define FEATURE_MQTT                    ENABLE_FEATURE
#define FEATURE_BLE_MODEM               ENABLE_FEATURE
#define FEATURE_FLASH_EEPROM            ENABLE_FEATURE

// eSIM LPA menu features (only takes effect if FEATURE_ESIM_LPA_MENU is enabled)
#define FEATURE_ADD_PROFILE             ENABLE_FEATURE
#define FEATURE_ADVANCED_OPTIONS        ENABLE_FEATURE
#define FEATURE_SWITCH_PROFILE          ENABLE_FEATURE
#define FEATURE_DELETE_PROFILE          ENABLE_FEATURE
#define FEATURE_SET_PROFILE_NICKNAME    ENABLE_FEATURE

// misc features
#define FEATURE_PLATFORM_TIME           ENABLE_FEATURE
#define FEATURE_DATA_STORE              ENABLE_FEATURE
#define FEATURE_GUI_MASK_KIOSK          ENABLE_FEATURE
#define FEATURE_GUI_MARKETING_SCREEN    DISABLE_FEATURE
#define FEATURE_CAPSENSE                ENABLE_FEATURE

// BLE features only takes effect if FEATURE_BLE_APP is enabled
#define FEATURE_BLE_APP                 ENABLE_FEATURE
#define FEATURE_BLE_DATA_STORE          ENABLE_FEATURE

// unit tests
#define FEATURE_UNIT_TEST_CURL          DISABLE_FEATURE
#define FEATURE_UNIT_TEST_ESIM_LPA      DISABLE_FEATURE
#define FEATURE_UNIT_TEST_RTOS          DISABLE_FEATURE

#ifdef __cplusplus
}
#endif

#endif      /* SOURCE_FEATURE_CONFIG_H_ */

/* [] END OF FILE */
