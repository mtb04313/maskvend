/******************************************************************************
* File Name:   ppp_task.c
*
* Description: This file contains the task that handles initialization &
*              connection of PPP client. The task also implements
*              reconnection mechanisms to handle PPP disconnections.
*              The task also handles all the cleanup operations to gracefully
*              terminate the PPP connection in case of any failure.
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
#include "ppp_task.h"
#include "app_ble.h"

#include <lwip/ip_addr.h>
#include "cy_wcm.h"
#include <string.h>


/* PPP connection manager header files. */
#include "cy_pcm.h"
#include "netif/ppp/pppapi.h"

/* Wi-Fi connection manager header files. */
#include "cy_wcm.h"
#include "cy_wcm_error.h"
#include "ppp_config.h"

#include "cy_memtrack.h"
#include "cy_debug.h"

#include "mbedtls/platform_time.h"  /* for mbedtls_time_t */

#include "cy_conio.h"
#include "cy_string.h"
#include "cy_time_misc.h"

#include <lwip/api.h>     /* for netconn_gethostbyname */
#include <lwip/dns.h>     /* for dns_getserver/dns_setserver */
#include <arpa/inet.h>

#include "wifi_task.h"
#include "common_task.h"
#include "cycfg_gatt_db.h"
#include "wiced_bt_stack.h"
#include "app_utils.h"

#include "cy_modem.h"
#include "cy_status.h"
#include "strings.h"

#include "cy_notification.h"
#include "cy_console_ui.h"

/*-- Local Definitions -------------------------------------------------*/

#define WIFI_INTERFACE_TYPE                      CY_WCM_INTERFACE_TYPE_STA
#define PROMPT_USER_TO_START_IO                  1


/*-- Public Data -------------------------------------------------*/

#if (FEATURE_PPP == ENABLE_FEATURE)
cy_thread_t g_ppp_task_handle = NULL;
#endif


/*-- Local Data -------------------------------------------------*/

#if (FEATURE_PPP == ENABLE_FEATURE)
static const char *TAG = "ppp_task";
static cy_notification_t s_notification = {0};
#endif

static bool s_ppp_connected = false;
static ip_addr_t s_ppp_dns_addr[2] = {0};
static cy_wcm_ip_address_t s_ppp_ip_addr = {0};
static common_status_t s_ppp_status = COMMON_STATUS_STOPPED;


/*-- Local Functions -------------------------------------------------*/

#if (FEATURE_PPP == ENABLE_FEATURE)
static void user_ip_lost(void)
{
    cy_time_t now = 0;

    cy_rtos_get_time(&now);
    CY_LOGI(TAG, "{%ld} User IP lost!", now);

    bool result = notify_ppp(NOTIF_RESTART_IO, false);
    DEBUG_PRINT(("notify_ppp returned: %d\n", result));
}

/* PPP Username and Password defined in ppp_config.h */
static cy_rslt_t connect_to_ppp(void)
{
    cy_rslt_t result;

    /* Variables used by PPP connection manager.*/
    cy_pcm_connect_params_t ppp_conn_param;
    cy_wcm_ip_address_t ip_address;

    /* Set the PPP username, password and security type. */
    memset(&ppp_conn_param, 0, sizeof(cy_pcm_connect_params_t));
    memcpy(ppp_conn_param.credentials.username, PPP_AUTH_USERNAME, sizeof(PPP_AUTH_USERNAME));
    memcpy(ppp_conn_param.credentials.password, PPP_AUTH_PASSWORD, sizeof(PPP_AUTH_PASSWORD));
    ppp_conn_param.credentials.security = PPP_SECURITY_TYPE;
    memcpy(ppp_conn_param.apn, PPP_APN, sizeof(PPP_APN));
    ppp_conn_param.user_ip_lost_fn = user_ip_lost;
    ppp_conn_param.connect_ppp = true;

    /* Join the network. */
    for (uint32_t conn_retries = 0; conn_retries < MAX_PPP_CONN_RETRIES; conn_retries++ ) {

#if PROMPT_USER_TO_START_IO
        // Ask the user whether to connect to PPP
        // (This is useful if the eSIM profile is a test or terminated profile,
        //  which will always fail to connect)
        PRINT_MSG(("\n# Waiting %d sec for user intervention\n", PPP_CONN_RETRY_INTERVAL_MSEC/1000));
        PRINT_MSG(("  If you do not wish to start PPP, press a key to enter the Console Menu,\n"));
        PRINT_MSG(("  select Manage I/O -> Cellular PPP -> Stop\n"));

        uint32_t ulNotifiedValue = 0;
        result = cy_notification_wait(&s_notification,
                                      0x00,              /* Don't clear any notification bits on entry. */
                                      UINT32_MAX,        /* Reset the notification value to 0 on exit. */
                                      &ulNotifiedValue,  /* Notified value pass out in */
                                      PPP_CONN_RETRY_INTERVAL_MSEC);
        if (ulNotifiedValue != 0) {
            print_notified_value(ulNotifiedValue);

            if (ulNotifiedValue == NOTIF_STOP_IO) {
                CY_LOGD(TAG, "User does not want to start PPP\n");
                return CY_RSLT_PCM_FAILED;
            }
        }
#endif

        result = cy_pcm_connect_modem(&ppp_conn_param,
                                      &ip_address,
                                      CY_RTOS_NEVER_TIMEOUT);

        if (result == CY_RSLT_SUCCESS) {
            bool is_valid_ip = false;
            CY_LOGD(TAG, "Successfully connected to PPP network.");

            if (ip_address.version == CY_WCM_IP_VER_V4) {
                CY_LOGD(TAG, "IPv4 Address Assigned: %d.%d.%d.%d",
                        (uint8_t)ip_address.ip.v4,
                        (uint8_t)(ip_address.ip.v4 >> 8),
                        (uint8_t)(ip_address.ip.v4 >> 16),
                        (uint8_t)(ip_address.ip.v4 >> 24));

                is_valid_ip = (ip_address.ip.v4 != 0);

            } else if (ip_address.version == CY_WCM_IP_VER_V6) {
                CY_LOGD(TAG, "IPv6 Address Assigned: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x",
                        (uint16_t)HIWORD(ip_address.ip.v6[0]),
                        (uint16_t)LOWORD(ip_address.ip.v6[0]),
                        (uint16_t)HIWORD(ip_address.ip.v6[1]),
                        (uint16_t)LOWORD(ip_address.ip.v6[1]),
                        (uint16_t)HIWORD(ip_address.ip.v6[2]),
                        (uint16_t)LOWORD(ip_address.ip.v6[2]),
                        (uint16_t)HIWORD(ip_address.ip.v6[3]),
                        (uint16_t)LOWORD(ip_address.ip.v6[3]));

                is_valid_ip = ((ip_address.ip.v6[0] != 0) && (ip_address.ip.v6[1] != 0) &&
                               (ip_address.ip.v6[2] != 0) && (ip_address.ip.v6[3] != 0));
            }

            if (is_valid_ip) {
                memcpy(&s_ppp_dns_addr[0], dns_getserver(0), sizeof(s_ppp_dns_addr[0]));
                memcpy(&s_ppp_dns_addr[1], dns_getserver(1), sizeof(s_ppp_dns_addr[1]));

                if (s_ppp_dns_addr[0].type == IPADDR_TYPE_V4) {
                    CY_LOGD(TAG, "PPP dns_server[0] = %s",
                            inet_ntoa(s_ppp_dns_addr[0]));
                } else if (s_ppp_dns_addr[0].type == IPADDR_TYPE_V6) {
                    CY_LOGD(TAG, "PPP dns_server[0] = %s",
                            inet6_ntoa(s_ppp_dns_addr[0]));
                }

                if (s_ppp_dns_addr[0].type == IPADDR_TYPE_V4) {
                    CY_LOGD(TAG, "PPP dns_server[1] = %s",
                            inet_ntoa(s_ppp_dns_addr[1]));
                } else if (s_ppp_dns_addr[0].type == IPADDR_TYPE_V6) {
                    CY_LOGD(TAG, "PPP dns_server[1] = %s",
                            inet6_ntoa(s_ppp_dns_addr[1]));
                }

                memcpy(&s_ppp_ip_addr, &ip_address, sizeof(s_ppp_ip_addr));
                return result;

            } else {
                CY_LOGE(TAG, "IP address is not valid!");
                s_ppp_status = COMMON_STATUS_STOPPING;

                result = cy_pcm_disconnect_modem(CY_RTOS_NEVER_TIMEOUT, true);
                if (result != CY_RSLT_SUCCESS) {
                    CY_LOGD(TAG, "cy_pcm_disconnect_modem failed!");

                } else {
                    CY_LOGD(TAG, "cy_pcm_disconnect_modem ok");
                }
            }
        }

        CY_LOGE(TAG, "Connection to PPP failed with error code %d", (int)result);

        if (result == CY_RSLT_PCM_MODEM_IN_USE) {
            CY_LOGE(TAG, "modem is in-use");
            return result;
        }
    }

    /* Stop retrying after maximum retry attempts. */
    CY_LOGD(TAG, "Exceeded %d PPP connection attempts", MAX_PPP_CONN_RETRIES);

    return CY_RSLT_PCM_FAILED;
}

#endif


/*-- Public Functions -------------------------------------------------*/

void ppp_task(cy_thread_arg_t arg)
{
#if (FEATURE_PPP == ENABLE_FEATURE)
    cy_rslt_t result;

    cy_pcm_config_t ppp_config = {
        .default_type = CELLULAR_CONNECTIVITY,
        .wifi_interface_type = WIFI_INTERFACE_TYPE,
    };

    result = cy_notification_init( &s_notification,
                                   0);
    VoidAssert(result == CY_RSLT_SUCCESS);

    /* Initialize PPP connection manager. */
    result = cy_pcm_init(&ppp_config, is_wcm_initialized());

    if (result != CY_RSLT_SUCCESS) {
        CY_LOGD(TAG, "PPP Connection Manager initialization failed!");
        DEBUG_ASSERT(0);
    }
    CY_LOGD(TAG, "PPP Connection Manager initialized.");

    while (true) {
        bool repeat;

        /* Notification values received from other tasks */
        uint32_t ulNotifiedValue = 0;

        s_ppp_status = COMMON_STATUS_STARTING;
        set_status_str(STR_STARTING_PPP);

        if (connect_to_ppp() != CY_RSLT_SUCCESS ) {
            CY_LOGD(TAG, "\nFailed to connect to PPP.");
            s_ppp_connected = false;
            s_ppp_status = COMMON_STATUS_FAILED_TO_START;

            set_status_str(STR_PPP_FAILED_TO_START);
        } else {
            s_ppp_connected = true;
            s_ppp_status = COMMON_STATUS_STARTED;
            set_status_str(STR_PPP_STARTED);
        }

        do {
            repeat = false;

            /* Wait for a notification */
            CY_LOGD(TAG, "Waiting for next notification");

            result = cy_notification_wait(&s_notification,
                                          0x00,              /* Don't clear any notification bits on entry. */
                                          UINT32_MAX,        /* Reset the notification value to 0 on exit. */
                                          &ulNotifiedValue,  /* Notified value pass out in */
                                          CY_RTOS_NEVER_TIMEOUT);

            print_notified_value(ulNotifiedValue);

            if (NOTIF_START_IO == ulNotifiedValue) {
                if (s_ppp_connected) {
                    CY_LOGD(TAG, "PPP already started");

                    // wait for the next notification
                    repeat = true;
                }

            } else if ((NOTIF_STOP_IO == ulNotifiedValue) ||
                       (NOTIF_RESTART_IO == ulNotifiedValue) ||
                       (NOTIF_SHUTDOWN_IO == ulNotifiedValue)) {

                if (s_ppp_connected) {
                    s_ppp_status = COMMON_STATUS_STOPPING;
                    set_status_str(STR_STOPPING_PPP);

                    result = cy_pcm_disconnect_modem(CY_RTOS_NEVER_TIMEOUT, true);
                    if (result != CY_RSLT_SUCCESS) {
                        CY_LOGD(TAG, "cy_pcm_disconnect_modem failed!");

                    } else {
                        CY_LOGD(TAG, "cy_pcm_disconnect_modem ok");
                    }

                    s_ppp_connected = false;
                    memset(&s_ppp_ip_addr, 0, sizeof(s_ppp_ip_addr));
                    memset(s_ppp_dns_addr, 0, sizeof(s_ppp_dns_addr));

                    s_ppp_status = COMMON_STATUS_STOPPED;
                    set_status_str(STR_PPP_STOPPED);

                } else {
                    CY_LOGD(TAG, "PPP already stopped");
                }

                if (NOTIF_STOP_IO == ulNotifiedValue) {
                    // wait for the next notification
                    repeat = true;
                }

            } else {
                // invalid command, wait for the next one
                repeat = true;
            }
        } while (repeat);

        if (NOTIF_SHUTDOWN_IO == ulNotifiedValue) {
            break;  // end task
        }
    }

    result = cy_pcm_deinit();
    if (result != CY_RSLT_SUCCESS) {
        CY_LOGD(TAG, "cy_pcm_deinit failed!");
    }
    else {
        CY_LOGD(TAG, "cy_pcm_deinit ok");
    }

    cy_notification_deinit(&s_notification);
#endif

    while (true);
}


bool is_ppp_connected(void)
{
    return s_ppp_connected && cy_pcm_is_ppp_connected();
}

const ip_addr_t* get_ppp_dns_address(void)
{
    return &s_ppp_dns_addr[0];
}

const ip_addr_t* get_ppp_dns_2_address(void)
{
    return &s_ppp_dns_addr[1];
}

const cy_wcm_ip_address_t* get_ppp_ip_address(void)
{
    return &s_ppp_ip_addr;
}

bool notify_ppp(uint32_t new_notification_value,
                bool in_isr)
{
#if (FEATURE_PPP == ENABLE_FEATURE)
    return (cy_notification_set(&s_notification,
                                new_notification_value,
                                in_isr) == CY_RSLT_SUCCESS);
#else
    return false;
#endif
}

const char* get_ppp_status(void)
{
    return get_common_status_str((int)s_ppp_status);
}
