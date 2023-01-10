/*******************************************************************************
 * File Name:   wifi_task.c
 *
* Description: This file contains the task that handles initialization &
*              connection of Wi-Fi client. The task also implements
*              reconnection mechanisms to handle Wi-Fi disconnections.
*              The task also handles all the cleanup operations to gracefully
*              terminate the Wi-Fi connection in case of any failure.
 *
 * Related Document: See Readme.md
 *
 *******************************************************************************
 * (c) 2020, Cypress Semiconductor Corporation. All rights reserved.
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
 ******************************************************************************/

#include "feature_config.h"

#include "wifi_task.h"
#include "common_task.h"

#include "cy_notification.h"

#include <lwip/api.h>     /* for netconn_gethostbyname */
#include <lwip/dns.h>     /* for dns_getserver/dns_setserver */
#include <arpa/inet.h>

#include "wifi_config.h"
#include "cy_debug.h"
#include "cy_console_ui.h"
#include "flash_eeprom.h" /* for HAVE_FLASH_EEPROM */
#include "cy_status.h"
#include "strings.h"


/*-- Local Definitions -------------------------------------------------*/

#define WIFI_INTERFACE_TYPE   CY_WCM_INTERFACE_TYPE_STA


/*-- Public Data -------------------------------------------------*/

#if (FEATURE_WIFI == ENABLE_FEATURE)
cy_thread_t g_wifi_task_handle = NULL;
#endif

/*-- Local Data -------------------------------------------------*/

#if (FEATURE_WIFI == ENABLE_FEATURE)
static const char *TAG = "wifi_task";
static cy_notification_t s_notification = {0};
#endif

static bool s_wcm_initialized = false;
static bool s_wifi_connected = false;
static ip_addr_t s_wifi_dns_addr = {0};
static cy_wcm_ip_address_t s_wifi_ip_addr = {0};
static common_status_t s_wifi_status = COMMON_STATUS_STOPPED;


/*-- Local Functions -------------------------------------------------*/

#if (FEATURE_WIFI == ENABLE_FEATURE)
/* WIFI SSID and Password defined in wifi_config.h */
static cy_rslt_t connect_to_wifi_ap(void)
{
    cy_rslt_t result;
    bool use_default_wifi_settings = true;

    /* Variables used by Wi-Fi connection manager.*/
    cy_wcm_connect_params_t wifi_conn_param;

    cy_wcm_ip_address_t ip_address;

    /* Set the Wi-Fi SSID, password and security type. */
    memset(&wifi_conn_param, 0, sizeof(cy_wcm_connect_params_t));

#if (HAVE_FLASH_EEPROM == 1)
    wifi_details_t wifi_details;

    /* Set the Wi-Fi SSID, password and security type. */
    memset(&wifi_details, 0, sizeof(wifi_details_t));

    if (flash_eeprom_get_wifi_settings( &wifi_details,
                                        sizeof(wifi_details))) {
      memcpy(wifi_conn_param.ap_credentials.SSID, wifi_details.wifi_ssid, wifi_details.ssid_len);
      memcpy(wifi_conn_param.ap_credentials.password, wifi_details.wifi_password, wifi_details.password_len);
      wifi_conn_param.ap_credentials.security = wifi_details.security;

      use_default_wifi_settings = false;
    }
#endif

    if (use_default_wifi_settings) {
        memcpy(wifi_conn_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
        memcpy(wifi_conn_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
        wifi_conn_param.ap_credentials.security = WIFI_SECURITY;
    }

    /* Join the Wi-Fi AP. */
    for (uint32_t conn_retries = 0; conn_retries < MAX_WIFI_CONN_RETRIES; conn_retries++) {

        // Ask the user whether to connect to Wi-Fi
        // (This is useful if the Wi-Fi credentials are incorrect,
        //  which will always fail to connect)
        PRINT_MSG(("\n# Waiting %d sec for user intervention\n", WIFI_CONN_RETRY_INTERVAL_MS/1000));
        PRINT_MSG(("  If you do not wish to start Wi-Fi, press a key to enter the Console Menu,\n"));
        PRINT_MSG(("  select Manage I/O -> Wi-Fi -> Stop\n"));

        uint32_t ulNotifiedValue = 0;
        result = cy_notification_wait(&s_notification,
                                      0x00,              /* Don't clear any notification bits on entry. */
                                      UINT32_MAX,        /* Reset the notification value to 0 on exit. */
                                      &ulNotifiedValue,  /* Notified value pass out in */
                                      WIFI_CONN_RETRY_INTERVAL_MS);
        if (ulNotifiedValue != 0) {
            print_notified_value(ulNotifiedValue);

            if (ulNotifiedValue == NOTIF_STOP_IO) {
                CY_LOGD(TAG, "User does not want to start Wi-Fi\n");
                return CY_RSLT_WCM_INTERFACE_NOT_UP;
            }
        }

        result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);

        if (result == CY_RSLT_SUCCESS) {
            CY_LOGD(TAG, "Successfully connected to Wi-Fi network '%s'.",
                    wifi_conn_param.ap_credentials.SSID);

            CY_LOGD(TAG, "IP Address Assigned: %d.%d.%d.%d",
                    (uint8_t)ip_address.ip.v4,
                    (uint8_t)(ip_address.ip.v4 >> 8),
                    (uint8_t)(ip_address.ip.v4 >> 16),
                    (uint8_t)(ip_address.ip.v4 >> 24));

            memcpy(&s_wifi_dns_addr, dns_getserver(0), sizeof(s_wifi_dns_addr));
            CY_LOGD(TAG, "WIFI dns_server[0] = %s",
                    inet_ntoa(s_wifi_dns_addr));

            memcpy(&s_wifi_ip_addr, &ip_address, sizeof(s_wifi_ip_addr));

            return result;
        }

        CY_LOGE(TAG, "Connection to Wi-Fi network failed with error code %d", (int)result);
    }

    /* Stop retrying after maximum retry attempts. */
    CY_LOGD(TAG, "Exceeded %d Wi-Fi connection attempts", MAX_WIFI_CONN_RETRIES);

    return result;
}
#endif


/*-- Public Functions -------------------------------------------------*/

void wifi_task(cy_thread_arg_t arg)
{
#if (FEATURE_WIFI == ENABLE_FEATURE)
    cy_rslt_t result;
    cy_wcm_config_t wifi_config = {
        .interface = WIFI_INTERFACE_TYPE
    };

    result = cy_notification_init( &s_notification,
                                   0);
    VoidAssert(result == CY_RSLT_SUCCESS);

    /* Initialize Wi-Fi connection manager. */
    result = cy_wcm_init(&wifi_config);

    if (result != CY_RSLT_SUCCESS) {
        CY_LOGD(TAG, "Wi-Fi Connection Manager initialization failed!");
        DEBUG_ASSERT(0);
    }

    s_wcm_initialized = true;
    CY_LOGD(TAG, "Wi-Fi Connection Manager initialized.");

    while (true) {
        bool repeat;

        /* Notification values received from other tasks */
        uint32_t ulNotifiedValue;

        set_status_str(STR_STARTING_WIFI);
        s_wifi_status = COMMON_STATUS_STARTING;

        if (connect_to_wifi_ap() != CY_RSLT_SUCCESS ) {
            CY_LOGD(TAG, "\nFailed to connect to Wi-FI AP.");
            s_wifi_connected = false;
            s_wifi_status = COMMON_STATUS_FAILED_TO_START;
            set_status_str(STR_WIFI_FAILED_TO_START);

        } else {
            s_wifi_connected = true;
            s_wifi_status = COMMON_STATUS_STARTED;
            set_status_str(STR_WIFI_STARTED);
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
                if (s_wifi_connected) {
                    CY_LOGD(TAG, "Wi-Fi already started");

                    // wait for the next notification
                    repeat = true;
                }

            } else if ((NOTIF_STOP_IO == ulNotifiedValue) ||
                       (NOTIF_RESTART_IO == ulNotifiedValue) ||
                       (NOTIF_SHUTDOWN_IO == ulNotifiedValue)) {

                if (s_wifi_connected) {
                    s_wifi_status = COMMON_STATUS_STOPPING;
                    set_status_str(STR_STOPPING_WIFI);

                    result = cy_wcm_disconnect_ap();
                    if (result != CY_RSLT_SUCCESS) {
                        CY_LOGD(TAG, "cy_wcm_disconnect_ap failed!");
                    }
                    else {
                        CY_LOGD(TAG, "cy_wcm_disconnect_ap ok");
                    }

                    s_wifi_connected = false;
                    memset(&s_wifi_ip_addr, 0, sizeof(s_wifi_ip_addr));
                    memset(&s_wifi_dns_addr, 0, sizeof(s_wifi_dns_addr));

                    s_wifi_status = COMMON_STATUS_STOPPED;
                    set_status_str(STR_WIFI_STOPPED);

                } else {
                    CY_LOGD(TAG, "Wi-Fi already stopped");
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

    result = cy_wcm_deinit();
    if (result != CY_RSLT_SUCCESS) {
        CY_LOGD(TAG, "cy_wcm_deinit failed!");
    } else {
        CY_LOGD(TAG, "cy_wcm_deinit ok");
    }

    s_wcm_initialized = false;

    cy_notification_deinit(&s_notification);
#endif

    while (true);
}

bool is_wcm_initialized(void)
{
    return s_wcm_initialized;
}

bool is_wifi_connected(void)
{
    return s_wifi_connected && cy_wcm_is_connected_to_ap();
}

const ip_addr_t* get_wifi_dns_address(void)
{
    return &s_wifi_dns_addr;
}

const cy_wcm_ip_address_t* get_wifi_ip_address(void)
{
    return &s_wifi_ip_addr;
}

bool notify_wifi(uint32_t new_notification_value,
                 bool in_isr)
{
#if (FEATURE_WIFI == ENABLE_FEATURE)
    return (cy_notification_set(&s_notification,
                                new_notification_value,
                                in_isr) == CY_RSLT_SUCCESS);
#else
    return false;
#endif
}

const char* get_wifi_status(void)
{
    return get_common_status_str((int)s_wifi_status);
}

/* [] END OF FILE */
