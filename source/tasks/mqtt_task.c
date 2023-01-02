/******************************************************************************
* File Name:   mqtt_task.c
*
* Description: This file contains the task that handles initialization &
*              connection of MQTT client. The task then starts
*              the subscriber and the publisher tasks. The task also implements
*              reconnection mechanisms to handle MQTT disconnections.
*              The task also handles all the cleanup operations to gracefully
*              terminate the MQTT connections in case of any failure.
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

#include "cyhal.h"
#include "cybsp.h"

/* Task header files */
#include "mqtt_task.h"
#include "common_task.h"
#include "subscriber_task.h"
#include "publisher_task.h"

/* Configuration file for Wi-Fi and MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_retarget_io.h"
#include "cy_wcm.h"
#include "cy_lwip.h"

#include "cy_mqtt_api.h"
#include "clock.h"

/* LwIP header files */
#include "lwip/netif.h"

#include "cy_pcm.h"
#include "cy_notification.h"
#include "cy_console_ui.h"
#include "cy_sntp_time.h"
#include "ppp_task.h"
#include "wifi_task.h"
#include "cy_debug.h"
#include "time_config.h"
#include "cy_status.h"
#include "gui_mask_task.h"

/*-- Local Definitions -------------------------------------------------*/

#define WAIT_FOR_SNTP_TIMEOUT_SEC   20

/* Queue length of a message queue that is used to communicate the status of
 * various operations.
 */
#define MQTT_TASK_QUEUE_LENGTH           (3u)

/* Time in milliseconds to wait before creating the publisher task. */
#define TASK_CREATION_DELAY_MS           (2000u)

/* Flag Masks for tracking which cleanup functions must be called. */
#define LIBS_INITIALIZED                 (1lu << 2)
#define BUFFER_INITIALIZED               (1lu << 3)
#define MQTT_INSTANCE_CREATED            (1lu << 4)
#define MQTT_CONNECTION_SUCCESS          (1lu << 5)
#define MQTT_MSG_RECEIVED                (1lu << 6)

/* Macro to check if the result of an operation was successful and set the
 * corresponding bit in the s_status_flag based on 'init_mask' parameter. When
 * it has failed, print the error message and return the result to the
 * calling function.
 */
#define CHECK_RESULT(result, init_mask, error_message...)      \
                     do                                        \
                     {                                         \
                         if ((int)result == CY_RSLT_SUCCESS)   \
                         {                                     \
                             s_status_flag |= init_mask;         \
                         }                                     \
                         else                                  \
                         {                                     \
                             CY_LOGD(TAG, error_message);      \
                             return result;                    \
                         }                                     \
                     } while(0)


/*-- Public Data -------------------------------------------------*/

/* MQTT connection handle. */
cy_mqtt_t g_mqtt_connection = NULL;

cy_thread_t g_mqtt_task_handle = NULL;

/* Queue handle used to communicate results of various operations - MQTT
 * Publish, MQTT Subscribe, MQTT connection, and Wi-Fi connection between tasks
 * and callbacks.
 */
cy_queue_t g_mqtt_task_q = NULL;


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "mqtt_task";

/* Flag to denote initialization status of various operations. */
static uint32_t s_status_flag = 0;

/* Pointer to the network buffer needed by the MQTT library for MQTT send and
 * receive operations.
 */
static uint8_t *s_mqtt_network_buffer = NULL;

static cy_notification_t s_notification = {0};

static bool s_mqtt_started = false;
static common_status_t s_mqtt_status = COMMON_STATUS_STOPPED;


/*-- Local Functions -------------------------------------------------*/

static void handle_mqtt_disconnect_event(void)
{
    mqtt_task_cmd_t mqtt_task_cmd;

    /* Clear the status flag bit to indicate MQTT disconnection. */
    s_status_flag &= ~(MQTT_CONNECTION_SUCCESS);

    /* MQTT connection with the MQTT broker is broken as the client
     * is unable to communicate with the broker. Set the appropriate
     * command to be sent to the MQTT task.
     */
    CY_LOGD(TAG, "Unexpectedly disconnected from MQTT broker!");
    mqtt_task_cmd = HANDLE_DISCONNECTION;

    /* Send the message to the MQTT client task to handle the
     * disconnection.
     */
    if (CY_RSLT_SUCCESS != cy_rtos_put_queue(   &g_mqtt_task_q,
            (void *)&mqtt_task_cmd,
            CY_RTOS_NEVER_TIMEOUT,
            false
                                            )) {
        CY_LOGD(TAG, "cy_rtos_put_queue(g_mqtt_task_q) failed!");
    }
}


/******************************************************************************
 * Function Name: mqtt_event_callback
 ******************************************************************************
 * Summary:
 *  Callback invoked by the MQTT library for events like MQTT disconnection,
 *  incoming MQTT subscription messages from the MQTT broker.
 *    1. In case of MQTT disconnection, the MQTT client task is communicated
 *       about the disconnection using a message queue.
 *    2. When an MQTT subscription message is received, the subscriber callback
 *       function implemented in subscriber_task.c is invoked to handle the
 *       incoming MQTT message.
 *
 * Parameters:
 *  cy_mqtt_t mqtt_handle : MQTT handle corresponding to the MQTT event (unused)
 *  cy_mqtt_event_t event : MQTT event information
 *  void *user_data : User data pointer passed during cy_mqtt_create() (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void mqtt_event_callback(cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data)
{
    cy_mqtt_publish_info_t *received_msg;

    (void) mqtt_handle;
    (void) user_data;

    switch(event.type) {
        case CY_MQTT_EVENT_TYPE_DISCONNECT: {
            handle_mqtt_disconnect_event();
            break;
        }

        case CY_MQTT_EVENT_TYPE_SUBSCRIPTION_MESSAGE_RECEIVE: {
            s_status_flag |= MQTT_MSG_RECEIVED;

            /* Incoming MQTT message has been received. Send this message to
             * the subscriber callback function to handle it.
             */
            received_msg = &(event.data.pub_msg.received_message);
            mqtt_subscription_callback(received_msg);
            break;
        }

        default : {
            /* Unknown MQTT event */
            CY_LOGD(TAG, "Unknown Event received from MQTT callback!");
            break;
        }
    }
}


/******************************************************************************
 * Function Name: mqtt_init
 ******************************************************************************
 * Summary:
 *  Function that initializes the MQTT library and creates an instance for the
 *  MQTT client. The network buffer needed by the MQTT library for MQTT send
 *  send and receive operations is also allocated by this function.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t : CY_RSLT_SUCCESS on a successful initialization, else an error
 *              code indicating the failure.
 *
 ******************************************************************************/
static cy_rslt_t mqtt_init(void)
{
    /* Variable to indicate status of various operations. */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the MQTT library. */
    result = cy_mqtt_init();
    CHECK_RESULT(result, LIBS_INITIALIZED, "MQTT library initialization failed!\n");

    /* Allocate buffer for MQTT send and receive operations. */
    s_mqtt_network_buffer = (uint8_t *) malloc(sizeof(uint8_t) * MQTT_NETWORK_BUFFER_SIZE);
    if(s_mqtt_network_buffer == NULL) {
        result = ~CY_RSLT_SUCCESS;
    }
    CHECK_RESULT(result, BUFFER_INITIALIZED, "Network Buffer allocation failed!\n");

    /* Create the MQTT client instance. */
    result = cy_mqtt_create(s_mqtt_network_buffer, MQTT_NETWORK_BUFFER_SIZE,
                            security_info, &broker_info,
                            (cy_mqtt_callback_t)mqtt_event_callback, NULL,
                            &g_mqtt_connection);
    CHECK_RESULT(result, MQTT_INSTANCE_CREATED, "MQTT instance creation failed!\n");
    CY_LOGD(TAG, "MQTT library initialization successful.\n");

    return result;
}


/******************************************************************************
 * Function Name: mqtt_cleanup
 ******************************************************************************
 * Summary:
 *  Function that invokes the deinit and cleanup functions for various
 *  operations based on the s_status_flag.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void mqtt_cleanup(void)
{
    /* Disconnect the MQTT connection if it was established. */
    if (s_status_flag & MQTT_CONNECTION_SUCCESS) {
        CY_LOGD(TAG, "Disconnecting from the MQTT Broker...");
        cy_mqtt_disconnect(g_mqtt_connection);
    }
    /* Delete the MQTT instance if it was created. */
    if (s_status_flag & MQTT_INSTANCE_CREATED) {
        cy_mqtt_delete(g_mqtt_connection);
        g_mqtt_connection = NULL;
    }
    /* Deallocate the network buffer. */
    if (s_status_flag & BUFFER_INITIALIZED) {
        free((void *) s_mqtt_network_buffer);
    }
    /* Deinit the MQTT library. */
    if (s_status_flag & LIBS_INITIALIZED) {
        cy_mqtt_deinit();
    }
}


#if GENERATE_UNIQUE_CLIENT_ID
/******************************************************************************
 * Function Name: mqtt_get_unique_client_identifier
 ******************************************************************************
 * Summary:
 *  Function that generates unique client identifier for the MQTT client by
 *  appending a timestamp to a common prefix 'MQTT_CLIENT_IDENTIFIER'.
 *
 * Parameters:
 *  char *mqtt_client_identifier : Pointer to the string that stores the
 *                                 generated unique identifier
 *
 * Return:
 *  cy_rslt_t : CY_RSLT_SUCCESS on successful generation of the client
 *              identifier, else a non-zero value indicating failure.
 *
 ******************************************************************************/
static cy_rslt_t mqtt_get_unique_client_identifier( char *mqtt_client_identifier,
                                                    size_t buf_size)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Check for errors from snprintf. */
    if (0 > snprintf(mqtt_client_identifier,
                     buf_size,
                     MQTT_CLIENT_IDENTIFIER "%lu" "%u",
                     (long unsigned int)Clock_GetTimeMs(), DEFAULT_KIOSK_ID))
    {
        status = ~CY_RSLT_SUCCESS;
    }

    return status;
}
#endif /* GENERATE_UNIQUE_CLIENT_ID */


/******************************************************************************
 * Function Name: mqtt_connect
 ******************************************************************************
 * Summary:
 *  Function that initiates MQTT connect operation. The connection is retried
 *  a maximum of 'MAX_MQTT_CONN_RETRIES' times with interval of
 *  'MQTT_CONN_RETRY_INTERVAL_MS' milliseconds.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t : CY_RSLT_SUCCESS upon a successful MQTT connection, else an
 *              error code indicating the failure.
 *
 ******************************************************************************/
static cy_rslt_t mqtt_connect(void)
{
    /* Variable to indicate status of various operations. */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* MQTT client identifier string. */
    char mqtt_client_identifier[(MQTT_CLIENT_IDENTIFIER_MAX_LEN + 1)] = MQTT_CLIENT_IDENTIFIER;

    /* Configure the user credentials as a part of MQTT Connect packet */
    if (strlen(MQTT_USERNAME) > 0) {
        g_mqtt_connection_info.username = MQTT_USERNAME;
        g_mqtt_connection_info.password = MQTT_PASSWORD;
        g_mqtt_connection_info.username_len = sizeof(MQTT_USERNAME) - 1;
        g_mqtt_connection_info.password_len = sizeof(MQTT_PASSWORD) - 1;
    }

    mqtt_set_will_topic();

    /* Generate a unique client identifier with 'MQTT_CLIENT_IDENTIFIER' string
     * as a prefix if the `GENERATE_UNIQUE_CLIENT_ID` macro is enabled.
     */
#if GENERATE_UNIQUE_CLIENT_ID
    result = mqtt_get_unique_client_identifier(mqtt_client_identifier, sizeof(mqtt_client_identifier));
    CHECK_RESULT(result, 0, "Failed to generate unique client identifier for the MQTT client!\n");
#endif /* GENERATE_UNIQUE_CLIENT_ID */

    /* Set the client identifier buffer and length. */
    g_mqtt_connection_info.client_id = mqtt_client_identifier;
    g_mqtt_connection_info.client_id_len = strlen(mqtt_client_identifier);

    CY_LOGD(TAG, "MQTT client '%.*s' connecting to MQTT broker '%.*s'...\n",
           g_mqtt_connection_info.client_id_len,
           g_mqtt_connection_info.client_id,
           broker_info.hostname_len,
           broker_info.hostname);

    result = CY_RSLT_MODULE_MQTT_ERROR;

    for (uint32_t retry_count = 0; retry_count < MAX_MQTT_CONN_RETRIES; retry_count++) {
        bool is_io_ready = false;
        connectivity_t default_io = NO_CONNECTIVITY;

        if (retry_count > 0) {
            // Ask the user whether to connect to MQTT
            // (This is useful if the eSIM profile is a test or terminated profile,
            //  which will always fail to connect)
            PRINT_MSG(("\n# Waiting %d sec for user intervention\n", MQTT_CONN_RETRY_INTERVAL_MS/1000));
            PRINT_MSG(("  If you do not wish to start MQTT, press a key to enter the Console Menu,\n"));
            PRINT_MSG(("  select Manage Apps -> MQTT -> Stop\n"));

            uint32_t ulNotifiedValue = 0;
            result = cy_notification_wait(&s_notification,
                                          0x00,              /* Don't clear any notification bits on entry. */
                                          UINT32_MAX,        /* Reset the notification value to 0 on exit. */
                                          &ulNotifiedValue,  /* Notified value pass out in */
                                          MQTT_CONN_RETRY_INTERVAL_MS);
            if (ulNotifiedValue != 0) {
                print_notified_value(ulNotifiedValue);

                if (ulNotifiedValue == NOTIF_STOP_APP) {
                    CY_LOGD(TAG, "User does not want to start MQTT\n");
                    return CY_RSLT_MODULE_MQTT_ERROR;
                }
            }
        }

#if (FEATURE_PPP == ENABLE_FEATURE)
        default_io = cy_pcm_get_default_connectivity();
#elif (FEATURE_WIFI == ENABLE_FEATURE)
        default_io = WIFI_STA_CONNECTIVITY;
#endif

        if (default_io == CELLULAR_CONNECTIVITY) {
#if (FEATURE_PPP == ENABLE_FEATURE)
            is_io_ready = cy_pcm_is_ppp_connected();
#endif
        } else if (default_io == WIFI_STA_CONNECTIVITY) {
#if (FEATURE_WIFI == ENABLE_FEATURE)
            is_io_ready = cy_wcm_is_connected_to_ap();
#endif
        } else {
            CY_LOGI(TAG, "default_io: NO_CONNECTIVITY");
        }

        if (is_io_ready) {
            // wait until PPP is available, otherwise wait until WIFI is avail

            /* Establish the MQTT connection. */
            result = cy_mqtt_connect(g_mqtt_connection, &g_mqtt_connection_info);

            if (result == CY_RSLT_SUCCESS) {
                CY_LOGD(TAG, "MQTT connection successful on %s.\n",
                        get_connectivity_type(default_io));

                /* Set the appropriate bit in the s_status_flag to denote successful
                 * MQTT connection, and return the result to the calling function.
                 */
                s_status_flag |= MQTT_CONNECTION_SUCCESS;
                return result;
            }

            CY_LOGD(TAG, "MQTT connection failed with error code 0x%0X. Retrying in %d ms. Retries left: %d",
                   (int)result, MQTT_CONN_RETRY_INTERVAL_MS, (int)(MAX_MQTT_CONN_RETRIES - retry_count - 1));
        } else {
            CY_LOGD(TAG, "MQTT connection waiting for %s. Retrying in %d ms. Retries left: %d",
                   get_connectivity_type(default_io),
                   MQTT_CONN_RETRY_INTERVAL_MS,
                   (int)(MAX_MQTT_CONN_RETRIES - retry_count - 1));
        }
    }

    CY_LOGD(TAG, "Exceeded %d MQTT connection attempts", MAX_MQTT_CONN_RETRIES);
    CY_LOGD(TAG, "MQTT connection failed after retrying for %d mins\n", 
           (int)(MQTT_CONN_RETRY_INTERVAL_MS * MAX_MQTT_CONN_RETRIES) / 60000u);
    return result;
}

static cy_rslt_t mqtt_create_subtasks(void)
{
    cy_rslt_t result;

    /* Create the subscriber task and cleanup if the operation fails. */
    result = cy_rtos_create_thread( &g_subscriber_task_handle,
                                    subscriber_task,
                                    SUBSCRIBER_TASK_NAME,
                                    NULL,
                                    SUBSCRIBER_TASK_STACK_SIZE,
                                    SUBSCRIBER_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL);

    if (result == CY_RSLT_SUCCESS) {
        /* Wait for the subscribe operation to complete. */
        cy_rtos_delay_milliseconds(TASK_CREATION_DELAY_MS);

        /* Create the publisher task and cleanup if the operation fails. */
        result = cy_rtos_create_thread( &g_publisher_task_handle,
                                        publisher_task,
                                        PUBLISHER_TASK_NAME,
                                        NULL,
                                        PUBLISHER_TASK_STACK_SIZE,
                                        PUBLISHER_TASK_PRIORITY,
                                        (cy_thread_arg_t) NULL);

        if (result != CY_RSLT_SUCCESS) {
            CY_LOGD(TAG, "Failed to create the Publisher task!");
        }

    } else {
        CY_LOGD(TAG, "Failed to create the Subscriber task!");
    }

    return result;
}

static void mqtt_delete_subtasks(void)
{
    CY_LOGD(TAG, "Terminating Publisher and Subscriber tasks...");

    if (g_subscriber_task_handle != NULL) {
        if (CY_RSLT_SUCCESS != cy_rtos_terminate_thread(&g_subscriber_task_handle)) {
            CY_LOGD(TAG, "Failed to terminate the Subscriber thread!");
        }

        if (CY_RSLT_SUCCESS != cy_rtos_join_thread(&g_subscriber_task_handle)) {
            CY_LOGD(TAG, "Failed to join the Subscriber thread!");
        }
        g_subscriber_task_handle = NULL;
    }

    if (g_publisher_task_handle != NULL) {
        if (CY_RSLT_SUCCESS != cy_rtos_terminate_thread(&g_publisher_task_handle)) {
            CY_LOGD(TAG, "Failed to delete the Publisher thread!");
        }

        if (CY_RSLT_SUCCESS != cy_rtos_join_thread(&g_publisher_task_handle)) {
            CY_LOGD(TAG, "Failed to join the Publisher thread!");
        }
        g_publisher_task_handle = NULL;
    }
}

static void handle_mqtt_operations(void)
{
    while (true) {
        bool abort = false;
        mqtt_task_cmd_t mqtt_status;

        /* Wait for results of MQTT operations from other tasks and callbacks. */
        if (CY_RSLT_SUCCESS == cy_rtos_get_queue(   &g_mqtt_task_q,
                                                    (void *)&mqtt_status,
                                                    CY_RTOS_NEVER_TIMEOUT,
                                                    false))
        {
            /* In this code example, the disconnection from the MQTT Broker or
             * the Wi-Fi network is handled by the case 'HANDLE_DISCONNECTION'.
             *
             * The publish and subscribe failures (`HANDLE_MQTT_PUBLISH_FAILURE`
             * and `HANDLE_MQTT_SUBSCRIBE_FAILURE`) does not initiate
             * reconnection in this example, but they can be handled as per the
             * application requirement in the following switch cases.
             */
            switch(mqtt_status) {
                case HANDLE_MQTT_PUBLISH_FAILURE: {
                    /* Handle Publish Failure here. */
                    break;
                }

                case HANDLE_MQTT_SUBSCRIBE_FAILURE: {
                    /* Handle Subscribe Failure here. */
                    break;
                }

                case HANDLE_DISCONNECTION: {
                    publisher_data_t publisher_q_data;

                    /* Deinit the publisher before initiating reconnections. */
                    publisher_q_data.cmd = PUBLISHER_DEINIT;

                    CY_LOGD(TAG, "cy_rtos_put_queue: PUBLISHER_DEINIT");
                    if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_publisher_task_q,
                                                             (void *)&publisher_q_data,
                                                             CY_RTOS_NEVER_TIMEOUT,
                                                             false)) {
                        CY_LOGD(TAG, "cy_rtos_put_queue(g_publisher_task_q) failed!");
                    }

                    set_status_str(STR_MQTT_DISCONNECTING);

                    /* Although the connection with the MQTT Broker is lost,
                     * call the MQTT disconnect API for cleanup of threads and
                     * other resources before reconnection.
                     */
                    cy_mqtt_disconnect(g_mqtt_connection);

                    set_status_str(STR_MQTT_RECONNECTING);

                    CY_LOGD(TAG, "Initiating MQTT Reconnection...");
                    if (CY_RSLT_SUCCESS == mqtt_connect()) {
                        subscriber_data_t subscriber_q_data;

                        /* Initiate MQTT subscribe post the reconnection. */
                        subscriber_q_data.cmd = SUBSCRIBE_TO_TOPIC;

                        CY_LOGD(TAG, "cy_rtos_put_queue: SUBSCRIBE_TO_TOPIC");
                        if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_subscriber_task_q,
                                                                (void *)&subscriber_q_data,
                                                                CY_RTOS_NEVER_TIMEOUT,
                                                                false)) {
                            CY_LOGD(TAG, "cy_rtos_put_queue(g_subscriber_task_q) failed!");
                            abort = true;
                        }

                        /* Initialize Publisher post the reconnection. */
                        publisher_q_data.cmd = PUBLISHER_INIT;

                        CY_LOGD(TAG, "cy_rtos_put_queue: PUBLISHER_INIT");
                        if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_publisher_task_q,
                                                                (void *)&publisher_q_data,
                                                                CY_RTOS_NEVER_TIMEOUT,
                                                                false)) {
                            CY_LOGD(TAG, "cy_rtos_put_queue(g_publisher_task_q) failed!");
                            abort = true;
                        }

                    } else {
                        abort = true;
                    }

                    // publish the current levels whe MQTT is up, so as to update device web page
                    send_gui_publish_mask_stock_level();
                    break;
                }

                case HANDLE_EXIT_LOOP:
                    abort = true;
                    break;

                default:
                    CY_LOGD(TAG, "Unknown mqtt_task_cmd_t: %d", mqtt_status);
                    break;
            }
        }

        if (abort) {
            break;
        }
    }
}


/*-- Public Functions -------------------------------------------------*/

/******************************************************************************
 * Function Name: mqtt_client_task
 ******************************************************************************
 * Summary:
 *  Task for handling initialization & connection of Wi-Fi and the MQTT client.
 *  The task also creates and manages the subscriber and publisher tasks upon
 *  successful MQTT connection. The task also handles the WiFi and MQTT
 *  connections by initiating reconnection on the event of disconnections.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/

void mqtt_client_task(cy_thread_arg_t pvParameters)
{
    cy_rslt_t result;

    /* To avoid compiler warnings */
    (void) pvParameters;

    result = cy_notification_init( &s_notification,
                                   0);
    VoidAssert(result == CY_RSLT_SUCCESS);

    /* Create a message queue to communicate with other tasks and callbacks. */
    if (CY_RSLT_SUCCESS !=  cy_rtos_init_queue( &g_mqtt_task_q,
                                                MQTT_TASK_QUEUE_LENGTH,
                                                sizeof(mqtt_task_cmd_t))) {
        CY_LOGD(TAG, "cy_rtos_init_queue(g_mqtt_task_q) failed!");
        DEBUG_ASSERT(0);
    }

    while (true) {
        bool repeat;

        /* Notification values received from other tasks */
        uint32_t ulNotifiedValue = 0;

        s_mqtt_status = COMMON_STATUS_STARTING;
        set_status_str(STR_MQTT_CONNECTING);

        /* Set-up the MQTT client and connect to the MQTT broker. Jump to the
         * cleanup block if any of the operations fail.
         */
        if ((CY_RSLT_SUCCESS == mqtt_init()) &&
            (CY_RSLT_SUCCESS == mqtt_connect()) &&
            (CY_RSLT_SUCCESS == mqtt_create_subtasks())) {

            s_mqtt_started = true;
            s_mqtt_status = COMMON_STATUS_STARTED;
            set_status_str(STR_MQTT_CONNECTED);

            // get network time
            set_status_str(STR_GETTING_NETWORK_TIME);

            cy_sntp_time_start_sync();
            cy_wait_for_sntp_time(WAIT_FOR_SNTP_TIMEOUT_SEC);
            cy_sntp_time_stop_sync();

            // ready
            set_status_str(STR_READY);

            // publish the current levels whe MQTT is up, so as to update device web page
            send_gui_publish_mask_stock_level();

            handle_mqtt_operations();

            s_mqtt_status = COMMON_STATUS_STOPPED;

        } else {
            s_mqtt_status = COMMON_STATUS_FAILED_TO_START;
        }

        /* Cleanup section: Delete subscriber and publisher tasks and perform
         * cleanup for various operations based on the s_status_flag.
         */

        mqtt_delete_subtasks();
        mqtt_cleanup();

        s_mqtt_started = false;

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

            if (NOTIF_STOP_APP == ulNotifiedValue) {
                // already stopped
                // wait for the next notification
                repeat = true;

            } else if ((NOTIF_START_APP == ulNotifiedValue) ||
                       (NOTIF_RESTART_APP == ulNotifiedValue) ||
                       (NOTIF_SHUTDOWN_APP == ulNotifiedValue)) {
                // leave the loop

            } else {
                // invalid command, wait for the next one
                repeat = true;
            }
        } while (repeat);

        if (NOTIF_SHUTDOWN_APP == ulNotifiedValue) {
            break;  // end task
        }
    }

    CY_LOGD(TAG, "Terminating the MQTT task...\n");
    if (CY_RSLT_SUCCESS != cy_rtos_terminate_thread(&g_mqtt_task_handle)) {
        CY_LOGD(TAG, "Failed to terminate the MQTT thread!");
    }

    if (CY_RSLT_SUCCESS != cy_rtos_join_thread(&g_mqtt_task_handle)) {
        CY_LOGD(TAG, "Failed to join the MQTT thread!");
    }
    g_mqtt_task_handle = NULL;

    cy_notification_deinit(&s_notification);
}

bool notify_mqtt(uint32_t new_notification_value,
                 bool in_isr)
{
#if (FEATURE_MQTT == ENABLE_FEATURE)

    if (new_notification_value == NOTIF_START_APP) {
        if (s_mqtt_started) {
            CY_LOGD(TAG, "MQTT already started");
            return true;
        }

    } else if ((new_notification_value == NOTIF_STOP_APP) ||
               (new_notification_value == NOTIF_RESTART_APP) ||
               (new_notification_value == NOTIF_SHUTDOWN_APP)) {

        if (s_mqtt_started) {
            // MQTT is executing handle_mqtt_operations(),
            // send a message to its queue to make it exit
            mqtt_task_cmd_t mqtt_task_cmd = HANDLE_EXIT_LOOP;

            if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_mqtt_task_q,
                                                     (void *)&mqtt_task_cmd,
                                                     CY_RTOS_NEVER_TIMEOUT,
                                                     false
                                                    )) {
                CY_LOGD(TAG, "cy_rtos_put_queue(g_mqtt_task_q) failed!");
            }
        }
    }

    return (cy_notification_set(&s_notification,
                                new_notification_value,
                                in_isr) == CY_RSLT_SUCCESS);
#else
    return false;
#endif
}

const char* get_mqtt_status(void)
{
    return get_common_status_str((int)s_mqtt_status);
}

/* [] END OF FILE */
