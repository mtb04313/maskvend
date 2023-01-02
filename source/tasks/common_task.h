/******************************************************************************
* File Name:   common_task.h
*
* Description: This file is the public interface of common_task.c
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

#ifndef SOURCE_COMMON_TASK_H_
#define SOURCE_COMMON_TASK_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*-- Public Definitions -------------------------------------------------*/

/* Task notification value */
enum common_task_notifications {
    NOTIF_GATT_DB                   = 1,
    NOTIF_DISCONNECT_GATT_DB        = 2,
    NOTIF_DISCONNECT_BTN            = 3,
    NOTIF_RESTART_BT_ADVERT         = 4,
    NOTIF_GATT_DB_CONNECTION_SELECT = 5,
    NOTIF_GATT_DB_TASK_SELECT       = 6,
    NOTIF_RESTART_IO                = 7,
    NOTIF_START_IO                  = 8,
    NOTIF_STOP_IO                   = 9,
    NOTIF_SHUTDOWN_IO               = 10,
    NOTIF_GATT_DB_RW_ITEM           = 11,
    NOTIF_RESTART_APP               = 12,
    NOTIF_START_APP                 = 13,
    NOTIF_STOP_APP                  = 14,
    NOTIF_SHUTDOWN_APP              = 15,
};

typedef enum {
    APPS_UNKNOWN,
    APPS_MQTT,
} apps_t;

typedef enum {
    COMMON_STATUS_STARTING,
    COMMON_STATUS_STARTED,
    COMMON_STATUS_STOPPING,
    COMMON_STATUS_STOPPED,
    COMMON_STATUS_FAILED_TO_START,
    COMMON_STATUS_UNKNOWN,
} common_status_t;


/*-- Public Functions -------------------------------------------------*/

void print_notified_value(uint32_t ulNotifiedValue);

const char* get_connectivity_type(int type);

const char* get_common_status_str(int status);

#ifdef __cplusplus
}
#endif

#endif      /* SOURCE_COMMON_TASK_H_ */

/* [] END OF FILE */
