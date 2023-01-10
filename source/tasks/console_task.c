/******************************************************************************
* File Name:   console_task.c
*
* Description: This file contains the task that presents a menu in the UART
*              console for a user to view and control Wi-Fi and PPP
*              connections, and the eSIM LPA.
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

#include <string.h>
#include <stdio.h>

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#include <lwip/api.h>     /* for netconn_gethostbyname */
#include <lwip/dns.h>     /* for dns_getserver/dns_setserver */
#include <arpa/inet.h>

#include "cy_debug.h"
#include "cy_string.h"
#include "cy_conio.h"

#include "common_task.h"
#include "console_task.h"
#include "wifi_task.h"
#include "ppp_task.h"
#include "mqtt_task.h"

#include "cy_pcm.h"
#include "cy_memtrack.h"
#include "cy_atmodem.h"

#if ((FEATURE_ESIM_LPA_MENU == ENABLE_FEATURE) || (FEATURE_UNIT_TEST_CURL == ENABLE_FEATURE))
#include "cy_esim_lpa_stack_api.h"
#endif

#include "cy_modem.h"
#include "cy_console_ui.h"

#if (FEATURE_UNIT_TEST_RTOS == ENABLE_FEATURE)
#include "cy_unit_test_rtos.h"
#endif


/*-- Local Definitions -------------------------------------------------*/

#if (defined PPP_MODEM_CAN_SUPPORT_ESIM_LPA && \
    (FEATURE_ESIM_LPA_MENU == ENABLE_FEATURE) && \
    (FEATURE_PPP == ENABLE_FEATURE))

#define SHOW_ESIM_LPA_MENU  true
#else
#define SHOW_ESIM_LPA_MENU  false
#endif

#if SHOW_ESIM_LPA_MENU
#include "esim_lpa_stack_client.h"
#endif


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "console_task";

/*-- Public Data -------------------------------------------------*/

cy_thread_t g_console_task_handle = NULL;


/*-- Local Functions -------------------------------------------------*/

static void draw_menu_border()
{
    (void)TAG;  // avoid unused variable warning

    PRINT_MSG(("\n===============================================================\n"));
}

static bool is_within(uint8_t key, uint8_t min, uint8_t max)
{
    bool is_min_numeric = ((min >= '0') && (min <= '9'));
    bool is_max_numeric = ((max >= '0') && (max <= '9'));

    key = tolower(key);
    min = tolower(min);
    max = tolower(max);

    if ((is_min_numeric && is_max_numeric) ||
            (!is_min_numeric && !is_max_numeric)) {
        return ((key >= min) && (key <= max));
    }
    else if (is_min_numeric && !is_max_numeric) {
        return ((key >= min) && (key <= '9')) || ((key >= 'a') && (key <= max));
    }
    return false;
}

static void notify_io_task( connectivity_t chosen_io,
                            uint32_t notification_value)
{
    VoidAssert(chosen_io != NO_CONNECTIVITY);

#if (FEATURE_WIFI == ENABLE_FEATURE)
    if (chosen_io == WIFI_STA_CONNECTIVITY) {
        bool result = notify_wifi(notification_value, false);
        PRINT_MSG(("# notify_wifi returned: %d\n", result));

    }
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
    if (chosen_io == CELLULAR_CONNECTIVITY) {
        bool result = notify_ppp(notification_value, false);
        PRINT_MSG(("# notify_ppp returned: %d\n", result));
    }
#endif
}

#if (FEATURE_APPS == ENABLE_FEATURE)
static void notify_app_task(apps_t chosen_app,
                            uint32_t notification_value)
{
#if (FEATURE_MQTT == ENABLE_FEATURE)
    if (chosen_app == APPS_MQTT) {
        bool result = notify_mqtt(notification_value, false);
        PRINT_MSG(("# notify_mqtt returned: %d\n", result));
    }
#endif
}
#endif

static void set_default_io( connectivity_t *default_io,
                            connectivity_t chosen_io)
{
    VoidAssert(default_io != NULL);

#if (FEATURE_WIFI == ENABLE_FEATURE)
    if (chosen_io == WIFI_STA_CONNECTIVITY) {
        cy_rslt_t result;
        const ip_addr_t* wifi_dns_addr = get_wifi_dns_address();
        DEBUG_ASSERT(wifi_dns_addr != NULL);

        result = cy_pcm_set_default_connectivity(chosen_io);
        dns_setserver(0, wifi_dns_addr);

        PRINT_MSG(("# cy_pcm_set_default_connectivity returned: %lu\n", result));
    }
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
    if (chosen_io == CELLULAR_CONNECTIVITY) {
        cy_rslt_t result;
        const ip_addr_t* ppp_dns_addr = get_ppp_dns_address();
        const ip_addr_t* ppp_dns_2_addr = get_ppp_dns_2_address();
        DEBUG_ASSERT(ppp_dns_addr != NULL);
        DEBUG_ASSERT(ppp_dns_2_addr != NULL);

        result = cy_pcm_set_default_connectivity(chosen_io);
        dns_setserver(0, ppp_dns_addr);
        dns_setserver(1, ppp_dns_2_addr);

        PRINT_MSG(("# cy_pcm_set_default_connectivity returned: %lu\n", result));
    }
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
    *default_io = cy_pcm_get_default_connectivity();
#endif

#if (FEATURE_APPS == ENABLE_FEATURE)
    notify_app_task(APPS_MQTT, NOTIF_RESTART_APP);
#endif
}

static void handle_manage_io_tasks_menu(connectivity_t *default_io,
                                        connectivity_t chosen_io)
{
    do {
        uint8_t subSelection = 0x00;
        uint8_t optionFinal = '3';

        draw_menu_border();

        if (chosen_io == WIFI_STA_CONNECTIVITY) {
            PRINT_MSG(("# Manage Wi-Fi\n"));
        } else {
            PRINT_MSG(("# Manage Cellular PPP\n"));
        }

        PRINT_MSG(("  1  Stop\n"));
        PRINT_MSG(("  2  Start\n"));
        PRINT_MSG(("  3  Restart I/O\n"));

#if ((FEATURE_PPP == ENABLE_FEATURE) && (FEATURE_WIFI == ENABLE_FEATURE))
        PRINT_MSG(("  4  Set as default I/O\n"));
        ++optionFinal;
#endif

        PRINT_MSG(("  X  Exit\n"));

        subSelection = tolower(wait_for_key());
        PRINT_MSG(("\n"));

        if (!is_within(subSelection, '1', optionFinal))
            break;

        switch (subSelection)
        {
        case '1':
            notify_io_task(chosen_io, NOTIF_STOP_IO);
            break;

        case '2':
            notify_io_task(chosen_io, NOTIF_START_IO);
            break;

        case '3':
            notify_io_task(chosen_io, NOTIF_RESTART_IO);
            break;

        case '4':
            set_default_io(default_io, chosen_io);
            break;

        default:
            DEBUG_ASSERT(0);
            break;
        }

    } while(true);
}

static void handle_manage_io_types_menu(connectivity_t *default_io)
{
    do {
        uint8_t subSelection = 0x00;
        uint8_t optionFinal = '0';

#if (FEATURE_WIFI == ENABLE_FEATURE)
        uint8_t optionWifi = ++optionFinal;
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
        uint8_t optionPPP = ++optionFinal;
        *default_io = cy_pcm_get_default_connectivity();
#endif

        draw_menu_border();
        PRINT_MSG(("# Manage I/O\n"));

#if (FEATURE_WIFI == ENABLE_FEATURE)
        const char *wifi_status = get_wifi_status();

        if (*default_io == WIFI_STA_CONNECTIVITY) {
            PRINT_MSG(("  %c  Wi-Fi (default) - %s\n", optionWifi, wifi_status));
        }
        else {
            PRINT_MSG(("  %c  Wi-Fi - %s\n", optionWifi, wifi_status));
        }
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
        const char *ppp_status = get_ppp_status();

        if (*default_io == CELLULAR_CONNECTIVITY) {
            PRINT_MSG(("  %c  Cellular PPP (default) - %s\n", optionPPP, ppp_status));
        }
        else {
            PRINT_MSG(("  %c  Cellular PPP - %s\n", optionPPP, ppp_status));
        }
#endif

        PRINT_MSG(("  X  Exit\n"));

        subSelection = tolower(wait_for_key());
        PRINT_MSG(("\n"));

        if (!is_within(subSelection, '1', optionFinal))
            break;

        connectivity_t chosen_io = *default_io;

#if (FEATURE_WIFI == ENABLE_FEATURE)
        if (subSelection == optionWifi) {
            char buf[80];
            const cy_wcm_ip_address_t* wifi_ip_addr = get_wifi_ip_address();
            const ip_addr_t* wifi_dns_addr = get_wifi_dns_address();

            VoidAssert(wifi_ip_addr != NULL);
            VoidAssert(wifi_dns_addr != NULL);

            SNPRINTF( buf,
                      sizeof(buf),
                      "%d.%d.%d.%d",
                      (uint8_t)(wifi_ip_addr->ip.v4),
                      (uint8_t)(wifi_ip_addr->ip.v4 >> 8),
                      (uint8_t)(wifi_ip_addr->ip.v4 >> 16),
                      (uint8_t)(wifi_ip_addr->ip.v4 >> 24));
            PRINT_MSG(("\n# Wi-Fi IP: %s\n", buf));
            PRINT_MSG(("# DNS: %s\n", inet_ntoa(*wifi_dns_addr)));

            chosen_io = WIFI_STA_CONNECTIVITY;
        }
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
        if (subSelection == optionPPP) {
            char buf[80];
            const cy_wcm_ip_address_t* ppp_ip_addr = get_ppp_ip_address();
            const ip_addr_t* ppp_dns_addr = get_ppp_dns_address();
            const ip_addr_t* ppp_dns_2_addr = get_ppp_dns_2_address();

            VoidAssert(ppp_ip_addr != NULL);
            VoidAssert(ppp_dns_addr != NULL);
            VoidAssert(ppp_dns_2_addr != NULL);

            SNPRINTF( buf,
                      sizeof(buf),
                      "%d.%d.%d.%d",
                      (uint8_t)(ppp_ip_addr->ip.v4),
                      (uint8_t)(ppp_ip_addr->ip.v4 >> 8),
                      (uint8_t)(ppp_ip_addr->ip.v4 >> 16),
                      (uint8_t)(ppp_ip_addr->ip.v4 >> 24));
            PRINT_MSG(("\n# PPP IP: %s\n", buf));

            PRINT_MSG(("# DNS1: %s\n", inet_ntoa(*ppp_dns_addr)));
            PRINT_MSG(("# DNS2: %s\n", inet_ntoa(*ppp_dns_2_addr)));

            chosen_io = CELLULAR_CONNECTIVITY;
        }
#endif
        handle_manage_io_tasks_menu(default_io, chosen_io);

    } while(true);
}

#if (FEATURE_APPS == ENABLE_FEATURE)
static void handle_manage_apps_tasks_menu(apps_t chosen_app)
{
    VoidAssert(chosen_app != APPS_UNKNOWN);

    do {
        uint8_t subSelection = 0x00;
        uint8_t optionFinal = '3';

        draw_menu_border();

        if (chosen_app == APPS_MQTT) {
            PRINT_MSG(("# Manage MQTT\n"));
        }

        PRINT_MSG(("  1  Stop\n"));
        PRINT_MSG(("  2  Start\n"));
        PRINT_MSG(("  3  Restart\n"));
        PRINT_MSG(("  X  Exit\n"));

        subSelection = tolower(wait_for_key());
        PRINT_MSG(("\n"));

        if (!is_within(subSelection, '1', optionFinal))
            break;

        switch (subSelection)
        {
        case '1':
            notify_app_task(chosen_app, NOTIF_STOP_APP);
            break;

        case '2':
            notify_app_task(chosen_app, NOTIF_START_APP);
            break;

        case '3':
            notify_app_task(chosen_app, NOTIF_RESTART_APP);
            break;

        default:
            DEBUG_ASSERT(0);
            break;
        }

    } while(true);
}

static void handle_manage_apps_types_menu(void)
{
    do {
        uint8_t subSelection = 0x00;
        uint8_t optionFinal = '0';

#if (FEATURE_MQTT == ENABLE_FEATURE)
        uint8_t optionMqtt = ++optionFinal;
#endif

        draw_menu_border();
        PRINT_MSG(("# Manage Apps\n"));

#if (FEATURE_MQTT == ENABLE_FEATURE)
        const char *mqtt_status = get_mqtt_status();
        PRINT_MSG(("  %c  MQTT - %s\n", optionMqtt, mqtt_status));
#endif

        PRINT_MSG(("  X  Exit\n"));

        subSelection = tolower(wait_for_key());
        PRINT_MSG(("\n"));

        if (!is_within(subSelection, '1', optionFinal))
            break;

        apps_t chosen_app = APPS_UNKNOWN;

#if (FEATURE_MQTT == ENABLE_FEATURE)
        if (subSelection == optionMqtt) {
            chosen_app = APPS_MQTT;
        }
#endif

        handle_manage_apps_tasks_menu(chosen_app);

    } while(true);
}
#endif

#if SHOW_ESIM_LPA_MENU
static void handle_lpa_menu(void)
{
    cy_rslt_t result;
    cy_modem_mode_t current_mode;
    bool is_modem_connected = false;

    result = cy_pcm_get_modem_mode(&current_mode);

    if (result == CY_RSLT_PCM_MODEM_IS_NULL) {
        // PPP has not been connected, hence modem is NULL
        // Proceed to connect the modem for LPA, getting it into
        // command mode if everything goes well

        cy_pcm_connect_params_t lpa_conn_param;
        current_mode = CY_MODEM_COMMAND_MODE;

        memset(&lpa_conn_param, 0, sizeof(lpa_conn_param));
        lpa_conn_param.connect_ppp = false;

        result = cy_pcm_connect_modem(&lpa_conn_param, NULL, PCM_CONNECT_MODEM_TIMEOUT_MSEC);

        if (result == CY_RSLT_PCM_TIMEOUT) {
            // someone is using the modem
            PRINT_MSG(("# cy_pcm_connect_modem timeout, try again later\n"));
            return;

        } else if (result == CY_RSLT_SUCCESS) {
            is_modem_connected = true;
        }
    }

    if (result != CY_RSLT_SUCCESS) {
        PRINT_MSG(("# try again later\n"));
        return;
    }

    if (current_mode == CY_MODEM_PPP_MODE) {
        result = cy_pcm_change_modem_mode(CY_MODEM_COMMAND_MODE);

        if (result != CY_RSLT_SUCCESS) {
            PRINT_MSG(("# cy_pcm_change_modem_mode %d failed\n", CY_MODEM_COMMAND_MODE));
        }
    }

#if (FEATURE_ESIM_LPA_MENU == ENABLE_FEATURE)
    if (result == CY_RSLT_SUCCESS) {
        esim_lpa_stack_menu();

#else
        int result = lpa_simple_initialize();

        if (result == OK_RETURN) {

            result = lpa_simple_get_eid();
            if (result != OK_RETURN) {
                PRINT_MSG(("# lpa_simple_get_eid failed, result = %d\n", result));
            }

            result = lpa_simple_get_profiles();
            if (result != OK_RETURN) {
                PRINT_MSG(("# lpa_simple_get_profiles failed, result = %d\n", result));
            }

            result = lpa_simple_finalize();
            if (result != OK_RETURN) {
                PRINT_MSG(("# lpa_simple_finalize failed, result = %d\n", result));
            }

        } else {
            PRINT_MSG(("# lpa_simple_initialize failed, result = %d\n", result));
        }
#endif
    }

    if (current_mode == CY_MODEM_PPP_MODE) {
        // restore the previous mode
        result = cy_pcm_change_modem_mode(current_mode);

        if (result != CY_RSLT_SUCCESS) {
            PRINT_MSG(("# cy_pcm_change_modem_mode %d failed\n", current_mode));
        }
    }

    if (is_modem_connected) {
        result = cy_pcm_disconnect_modem(CY_RTOS_NEVER_TIMEOUT, false);
        if (result != CY_RSLT_SUCCESS) {
            CY_LOGD(TAG, "cy_pcm_disconnect_modem failed!");

        } else {
            CY_LOGD(TAG, "cy_pcm_disconnect_modem ok");
        }
    }
}
#endif


static void console_menu(void)
{
    uint8_t optionFinal = '1';
    connectivity_t default_io = NO_CONNECTIVITY;

#if (FEATURE_APPS == ENABLE_FEATURE)
    uint8_t optionManageApps = ++optionFinal;
#else
    uint8_t optionManageApps = -1;
#endif

#if SHOW_ESIM_LPA_MENU
    uint8_t optionLPA = ++optionFinal;
#endif

#if (FEATURE_UNIT_TEST_CURL == ENABLE_FEATURE)
    uint8_t optionUnitTestCurl = ++optionFinal;
#endif

#if (FEATURE_UNIT_TEST_RTOS == ENABLE_FEATURE)
    uint8_t optionUnitTestRtos = ++optionFinal;
#endif


    do {
        uint8_t selection = 0x00;

        draw_menu_border();
        PRINT_MSG(("# Console Menu\n"));
        PRINT_MSG(("  1  Manage I/O\n"));

#if (FEATURE_APPS == ENABLE_FEATURE)
        PRINT_MSG(("  %c  Manage Apps\n", optionManageApps));
#endif

#if SHOW_ESIM_LPA_MENU
        PRINT_MSG(("  %c  eSIM LPA\n", optionLPA));
#endif

#if (FEATURE_UNIT_TEST_CURL == ENABLE_FEATURE)
        PRINT_MSG(("  %c  Run cURL unit tests\n", optionUnitTestCurl));
#endif

#if (FEATURE_UNIT_TEST_RTOS == ENABLE_FEATURE)
        PRINT_MSG(("  %c  Run RTOS unit tests\n", optionUnitTestRtos));
#endif

        PRINT_MSG(("  X  Exit\n"));

        selection = tolower(wait_for_key());
        PRINT_MSG(("\n"));

        if (!is_within(selection, '1', optionFinal))
            break;

        switch (selection)
        {
        case '1':
            handle_manage_io_types_menu(&default_io);
            break;

        default:
            if (selection == optionManageApps) {

#if (FEATURE_APPS == ENABLE_FEATURE)
                handle_manage_apps_types_menu();
#endif

            }
#if SHOW_ESIM_LPA_MENU
            else if (selection == optionLPA) {
                connectivity_t default_io = cy_pcm_get_default_connectivity();

#if (FEATURE_WIFI == ENABLE_FEATURE)
                if (default_io == CELLULAR_CONNECTIVITY) {
                    PRINT_MSG(("\n# To add profiles, you need to make Wi-Fi the default I/O.\n"));

                    if (get_user_confirmation()) {
                        set_default_io( &default_io,
                                        WIFI_STA_CONNECTIVITY);
                    }
                }
#endif
                handle_lpa_menu();
            }
#endif

#if (FEATURE_UNIT_TEST_CURL == ENABLE_FEATURE)
            else if (selection == optionUnitTestCurl) {
                if (get_user_confirmation()) {
                    esim_lpa_stack_platform_unit_test_curl();
                }
            }
#endif

#if (FEATURE_UNIT_TEST_RTOS == ENABLE_FEATURE)
            else if (selection == optionUnitTestRtos) {
                if (get_user_confirmation()) {
                    unit_test_rtos_main();
                }
            }
#endif

            else {
                DEBUG_ASSERT(0);
            }
            break;
        }

    } while (true);
}


/*-- Public Functions -------------------------------------------------*/

void console_task(cy_thread_arg_t arg)
{
    while (true) {
        console_menu();

        CY_MEMTRACK_MALLOC_STATS();
    }
}
