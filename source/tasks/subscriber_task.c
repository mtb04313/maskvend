/******************************************************************************
* File Name:   subscriber_task.c
*
* Description: This file contains the task that initializes the user LED GPIO,
*              subscribes to the topic 'MQTT_SUB_TOPIC', and actuates the user LED
*              based on the notifications received from the MQTT subscriber
*              callback.
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
#include "string.h"

/* Task header files */
#include "subscriber_task.h"
#include "mqtt_task.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cy_retarget_io.h"

/*-- Local Definitions -------------------------------------------------*/

/* Maximum number of retries for MQTT subscribe operation */
#define MAX_SUBSCRIBE_RETRIES                   (3u)

/* Time interval in milliseconds between MQTT subscribe retries. */
#define MQTT_SUBSCRIBE_RETRY_INTERVAL_MS        (1000)

/* The number of MQTT topics to be subscribed to. */
#define SUBSCRIPTION_COUNT                      (1)

/* Queue length of a message queue that is used to communicate with the
 * subscriber task.
 */
#define SUBSCRIBER_TASK_QUEUE_LENGTH            (1u)


/*-- Public Data -------------------------------------------------*/

cy_thread_t g_subscriber_task_handle = NULL;

/* Handle of the queue holding the commands for the subscriber task */
cy_queue_t g_subscriber_task_q = NULL;

/* Variable to denote the current state of the user LED that is also used by
 * the publisher task.
 */
uint32_t g_current_device_state = DEVICE_OFF_STATE;


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "subscriber_task";

/* Configure the subscription information structure. */
static cy_mqtt_subscribe_info_t s_subscribe_info = {
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = MQTT_SUB_TOPIC,
    .topic_len = (sizeof(MQTT_SUB_TOPIC) - 1)
};


/*-- Local Functions -------------------------------------------------*/

/******************************************************************************
 * Function Name: subscribe_to_topic
 ******************************************************************************
 * Summary:
 *  Function that subscribes to the MQTT topic specified by the macro
 *  'MQTT_SUB_TOPIC'. This operation is retried a maximum of
 *  'MAX_SUBSCRIBE_RETRIES' times with interval of
 *  'MQTT_SUBSCRIBE_RETRY_INTERVAL_MS' milliseconds.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void subscribe_to_topic(void)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;

    /* Subscribe with the configured parameters. */
    for (uint32_t retry_count = 0; retry_count < MAX_SUBSCRIBE_RETRIES; retry_count++) {
        result = cy_mqtt_subscribe(g_mqtt_connection, &s_subscribe_info, SUBSCRIPTION_COUNT);
        if (result == CY_RSLT_SUCCESS) {
            CY_LOGD(TAG, "MQTT client subscribed to the topic '%.*s' successfully.\n",
                   s_subscribe_info.topic_len, s_subscribe_info.topic);
            break;
        }

        cy_rtos_delay_milliseconds(MQTT_SUBSCRIBE_RETRY_INTERVAL_MS);
    }

    if (result != CY_RSLT_SUCCESS) {
        CY_LOGD(TAG, "MQTT Subscribe failed with error 0x%0X after %d retries...\n",
               (int)result, MAX_SUBSCRIBE_RETRIES);

        /* Notify the MQTT client task about the subscription failure */
        mqtt_task_cmd = HANDLE_MQTT_SUBSCRIBE_FAILURE;

        if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_mqtt_task_q,
                (void *)&mqtt_task_cmd,
                CY_RTOS_NEVER_TIMEOUT,
                false
                                                )) {
            CY_LOGD(TAG, "cy_rtos_put_queue(g_mqtt_task_q) failed!");
        }
    }
}

/******************************************************************************
 * Function Name: mqtt_subscription_callback
 ******************************************************************************
 * Summary:
 *  Callback to handle incoming MQTT messages. This callback prints the
 *  contents of the incoming message and informs the subscriber task, via a
 *  message queue, to turn on / turn off the device based on the received
 *  message.
 *
 * Parameters:
 *  cy_mqtt_publish_info_t *received_msg_info : Information structure of the
 *                                              received MQTT message
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mqtt_subscription_callback(cy_mqtt_publish_info_t *received_msg_info)
{
    CY_LOGD(TAG, "Subsciber: Incoming MQTT message received:\n"
           "    Publish topic name: %.*s\n"
           "    Publish QoS: %d\n"
           "    Publish payload: %.*s\n",
           received_msg_info->topic_len, received_msg_info->topic,
           (int) received_msg_info->qos,
           (int) received_msg_info->payload_len, (const char *)received_msg_info->payload);
}

/******************************************************************************
 * Function Name: unsubscribe_from_topic
 ******************************************************************************
 * Summary:
 *  Function that unsubscribes from the topic specified by the macro
 *  'MQTT_SUB_TOPIC'.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void unsubscribe_from_topic(void)
{
    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);

    cy_rslt_t result = cy_mqtt_unsubscribe(g_mqtt_connection,
                                           (cy_mqtt_unsubscribe_info_t *) &s_subscribe_info,
                                           SUBSCRIPTION_COUNT);

    if (result != CY_RSLT_SUCCESS) {
        CY_LOGD(TAG, "MQTT Unsubscribe operation failed with error 0x%0X!", (int)result);
    }
}


/*-- Public Functions -------------------------------------------------*/

/******************************************************************************
 * Function Name: subscriber_task
 ******************************************************************************
 * Summary:
 *  Task that sets up the user LED GPIO, subscribes to the specified MQTT topic,
 *  and controls the user LED based on the received commands over the message
 *  queue. The task can also unsubscribe from the topic based on the commands
 *  via the message queue.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void subscriber_task(cy_thread_arg_t pvParameters)
{
    subscriber_data_t subscriber_q_data;

    /* To avoid compiler warnings */
    (void) pvParameters;

    /* Subscribe to the specified MQTT topic. */
    subscribe_to_topic();

    /* Create a message queue to communicate with other tasks and callbacks. */
    if (CY_RSLT_SUCCESS !=  cy_rtos_init_queue( &g_subscriber_task_q,
            SUBSCRIBER_TASK_QUEUE_LENGTH,
            sizeof(subscriber_data_t)
                                              )) {
        CY_LOGD(TAG, "cy_rtos_init_queue(g_subscriber_task_q) failed!");
        DEBUG_ASSERT(0);
    }

    while (true) {
        /* Wait for commands from other tasks and callbacks. */
        if (CY_RSLT_SUCCESS == cy_rtos_get_queue(  &g_subscriber_task_q,
                (void *)&subscriber_q_data,
                CY_RTOS_NEVER_TIMEOUT,
                false))
        {
            switch(subscriber_q_data.cmd) {
                case SUBSCRIBE_TO_TOPIC: {
                    subscribe_to_topic();
                    break;
                }

                case UNSUBSCRIBE_FROM_TOPIC: {
                    unsubscribe_from_topic();
                    break;
                }

                case UPDATE_DEVICE_STATE: {
                    break;
                }
            }
        }
    }
}

/* [] END OF FILE */
