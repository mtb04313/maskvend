/******************************************************************************
* File Name:   user_button_config.h
*
* Description: This file contains all the configuration macros used by the
*              user buttons in this example.
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

#ifndef SOURCE_USER_BUTTON_CONFIG_H_
#define SOURCE_USER_BUTTON_CONFIG_H_

#include "cycfg_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined (TARGET_APP_CY8CEVAL_062S2_LAI_4373M2) // CY8CEVAL Eval Kit

/* Configure the button that toggles the GUI screen */
#define USER_BUTTON_CHANGE_GUI_SCREEN           CYBSP_USER_BTN  // 1st button from the left edge

/* Configure the button that turns on BLE advertisement */
#define USER_BUTTON_TURN_ON_BLE_ADVERTISEMENT   CYBSP_USER_BTN2 // 2nd button from the left edge

#elif defined (TARGET_APP_CY8CKIT_062S2_43012) // 62S2 Pioneer Kit

/* Configure the button that toggles the GUI screen */
#define USER_BUTTON_CHANGE_GUI_SCREEN           CYBSP_USER_BTN2 // 1st button from the left edge

/* Configure the button that turns on BLE advertisement */
#define USER_BUTTON_TURN_ON_BLE_ADVERTISEMENT   CYBSP_USER_BTN  // 2nd button from the left edge

#else
#pragma GCC error "Unsupported TARGET board: " __FILE__
#endif

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_USER_BUTTON_CONFIG_H_ */

/* [] END OF FILE */
