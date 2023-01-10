/******************************************************************************
* File Name:   common_task.c
*
* Description: This file contains common code used by tasks
*
* Related Document: See README.md
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

#include "feature_config.h"
#include "common_task.h"
#include "cy_debug.h"
#include "cy_pcm.h"


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "common";


/*-- Public Functions -------------------------------------------------*/

void print_notified_value(uint32_t ulNotifiedValue)
{
    switch(ulNotifiedValue) {
    case NOTIF_GATT_DB:
        CY_LOGD(TAG, "NOTIF_GATT_DB\n");
        break;

    case NOTIF_DISCONNECT_GATT_DB:
        CY_LOGD(TAG, "NOTIF_DISCONNECT_GATT_DB\n");
        break;

    case NOTIF_DISCONNECT_BTN:
        CY_LOGD(TAG, "NOTIF_DISCONNECT_BTN\n");
        break;

    case NOTIF_RESTART_BT_ADVERT:
        CY_LOGD(TAG, "NOTIF_RESTART_BT_ADVERT\n");
        break;

    case NOTIF_GATT_DB_CONNECTION_SELECT:
        CY_LOGD(TAG, "NOTIF_GATT_DB_CONNECTION_SELECT\n");
        break;

    case NOTIF_GATT_DB_TASK_SELECT:
        CY_LOGD(TAG, "NOTIF_GATT_DB_TASK_SELECT\n");
        break;

    case NOTIF_RESTART_IO:
        CY_LOGD(TAG, "NOTIF_RESTART_IO\n");
        break;

    case NOTIF_START_IO:
        CY_LOGD(TAG, "NOTIF_START_IO\n");
        break;

    case NOTIF_STOP_IO:
        CY_LOGD(TAG, "NOTIF_STOP_IO\n");
        break;

    case NOTIF_SHUTDOWN_IO:
        CY_LOGD(TAG, "NOTIF_SHUTDOWN_IO\n");
        break;

    case NOTIF_GATT_DB_RW_ITEM:
        CY_LOGD(TAG, "NOTIF_GATT_DB_RW_ITEM\n");
        break;

    case NOTIF_RESTART_APP:
        CY_LOGD(TAG, "NOTIF_RESTART_APP\n");
        break;

    case NOTIF_START_APP:
        CY_LOGD(TAG, "NOTIF_START_APP\n");
        break;

    case NOTIF_STOP_APP:
        CY_LOGD(TAG, "NOTIF_STOP_APP\n");
        break;

    case NOTIF_SHUTDOWN_APP:
        CY_LOGD(TAG, "NOTIF_SHUTDOWN_APP\n");
        break;

    default:
        CY_LOGD(TAG, "Invalid notified value: %d\n", ulNotifiedValue);
        break;
    }
}

const char* get_connectivity_type(int type)
{
    switch (type) {
    case CELLULAR_CONNECTIVITY:
        return "Cellular";

    case WIFI_STA_CONNECTIVITY:
        return "Wi-Fi";

    default:
        break;
    }

    return "None";
}

const char* get_common_status_str(int status)
{
    switch (status) {
    case COMMON_STATUS_STARTING:
        return "Starting";

    case COMMON_STATUS_STARTED:
        return "Started";

    case COMMON_STATUS_STOPPING:
        return "Stopping";

    case COMMON_STATUS_STOPPED:
        return "Stopped";

    case COMMON_STATUS_FAILED_TO_START:
        return "Failed to start";

    case COMMON_STATUS_UNKNOWN:
    default:
        break;
    }

    return "Unknown status";
}
