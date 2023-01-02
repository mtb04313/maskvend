/******************************************************************************
* File Name: gui_mask_task.h
*
* Description: This file is the public interface of gui_mask_task.c source file
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
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


/*******************************************************************************
 * Include guard
 ******************************************************************************/
#ifndef SOURCE_GUI_MASK_TASK_H_
#define SOURCE_GUI_MASK_TASK_H_


/*******************************************************************************
 * Header file includes
 ******************************************************************************/

#include "feature_config.h"
#include "cy_debug.h"
#include "cyabs_rtos.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define GUI_MASK_TASK_NAME        "gui_mask task"
#define GUI_MASK_TASK_PRIORITY    CY_RTOS_PRIORITY_BELOWNORMAL //CY_RTOS_PRIORITY_LOW // use HIGH for smoother graphics during capsense
#define GUI_MASK_TASK_STACK_SIZE  (5 * 1024)


/*******************************************************************************
* Global constants
*******************************************************************************/
/* Allowed TCPWM compare value for maximum position */
#define GUI_MASK_MAX_POSITION  (100u)

/* Allowed TCPWM compare value for minimum position*/
#define GUI_MASK_MIN_POSITION  (2u)


/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/
/* Available GUI Menu */
typedef enum
{
  GUI_MENU_STATUS_APP,      // must be the first item
  GUI_MENU_MASK_APP,        // must be the second item
  GUI_MENU_NETWORK_INFO_APP,
  GUI_MENU_QRCODE_APP,

#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
  GUI_MENU_MARKETING_APP,
#endif

  GUI_MENU_CLOCK_FACE_APP,
  GUI_MENU_INVALID_APP,
} gui_menu_t;

/* Available GUI_MASK commands */
typedef enum
{
  GUI_CAPSENSE_OK,
  GUI_CAPSENSE_CANCEL,
  GUI_CAPSENSE_UPDATE_POSITION,
  GUI_USER_LEFT_BUTTON_PRESSED,
  GUI_CLOCK_TIMER_TICK,
  GUI_PUBLISH_MASK_STOCK_LEVEL,
} gui_mask_command_t;

/* Structure used for storing LED data */
typedef struct
{
    gui_mask_command_t command;
    uint32_t position;
} gui_mask_command_data_t;


/*******************************************************************************
 * Global variable
 ******************************************************************************/
extern cy_thread_t g_gui_mask_task_handle;
extern cy_queue_t g_gui_mask_command_data_q;

/*******************************************************************************
 * Function prototype
 ******************************************************************************/
void gui_mask_task(cy_thread_arg_t param);

const char* get_mask_stock_level(void);

bool send_gui_publish_mask_stock_level(void);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_GUI_MASK_TASK_H_ */

/* [] END OF FILE  */
