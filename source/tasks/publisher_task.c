/******************************************************************************
* File Name:   publisher_task.c
*
* Description: This file contains the task that sets up the user button GPIO 
*              for the publisher and publishes MQTT messages on the topic
*              'MQTT_PUB_TOPIC' to control a device that is actuated by the
*              subscriber task. The file also contains the ISR that notifies
*              the publisher task about the new device state to be published.
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
#include "publisher_task.h"
#include "mqtt_task.h"
#include "subscriber_task.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cy_retarget_io.h"

#include "cy_string.h"
#include "flash_eeprom.h"

/*-- Local Definitions -------------------------------------------------*/

/* Interrupt priority for User Button Input. */
#define USER_BTN_INTR_PRIORITY          (5)

/* The maximum number of times each PUBLISH in this example will be retried. */
#define PUBLISH_RETRY_LIMIT             (10)

/* A PUBLISH message is retried if no response is received within this 
 * time (in milliseconds).
 */
#define PUBLISH_RETRY_MS                (1000)

/* Queue length of a message queue that is used to communicate with the 
 * publisher task.
 */
#define PUBLISHER_TASK_QUEUE_LENGTH     (3u)


/*-- Public Data -------------------------------------------------*/

cy_thread_t g_publisher_task_handle = NULL;

/* Handle of the queue holding the commands for the publisher task */
cy_queue_t g_publisher_task_q = NULL;


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "publisher_task";

static char s_publish_topic[MQTT_PUB_TOPIC_MAX_LEN] = "";

/* Structure to store publish message information. */
static cy_mqtt_publish_info_t s_publish_info =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = s_publish_topic,
    .topic_len = 0,
    .retain = false,
    .dup = false
};


/*-- Local Functions -------------------------------------------------*/

static void set_publisher_topic(void)
{
  int16_t id = DEFAULT_KIOSK_ID;

#if (HAVE_FLASH_EEPROM == 1)
  // read the kiosk id and set the topic, topic_len
  if (!flash_eeprom_get_kiosk_id(&id, sizeof(id))) {
    CY_LOGD(TAG, "flash_eeprom_get_kiosk_id failed");
  }
#endif

  SNPRINTF(s_publish_topic, sizeof(s_publish_topic), MQTT_PUB_TOPIC_PATTERN, id);
  s_publish_info.topic_len = strlen(s_publish_topic);
}

/******************************************************************************
 * Function Name: publisher_init
 ******************************************************************************
 * Summary:
 *  Function that initializes and sets-up the user button GPIO pin along with  
 *  its interrupt.
 * 
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void publisher_init(void)
{
    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);

    set_publisher_topic();
}

/******************************************************************************
 * Function Name: publisher_deinit
 ******************************************************************************
 * Summary:
 *  Cleanup function for the publisher task that disables the user button  
 *  interrupt and deinits the user button GPIO pin.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void publisher_deinit(void)
{
    CY_LOGD(TAG, "%s [%d]", __FUNCTION__, __LINE__);
}


/*-- Public Functions -------------------------------------------------*/

/******************************************************************************
 * Function Name: publisher_task
 ******************************************************************************
 * Summary:
 *  Task that sets up the user button GPIO for the publisher and publishes
 *  MQTT messages to the broker. The user button init and deinit operations,
 *  and the MQTT publish operation is performed based on commands sent by other
 *  tasks and callbacks over a message queue.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void publisher_task(cy_thread_arg_t pvParameters)
{
    /* Status variable */
    cy_rslt_t result;

    publisher_data_t publisher_q_data;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;

    /* To avoid compiler warnings */
    (void) pvParameters;

    /* Initialize and set-up the user button GPIO. */
    publisher_init();

    /* Create a message queue to communicate with other tasks and callbacks. */
    if (CY_RSLT_SUCCESS !=  cy_rtos_init_queue( &g_publisher_task_q,
                                                PUBLISHER_TASK_QUEUE_LENGTH,
                                                sizeof(publisher_data_t)
                                              )) {
        CY_LOGD(TAG, "cy_rtos_init_queue(g_publisher_task_q) failed!");
        DEBUG_ASSERT(0);
    }

    while (true)
    {
        /* Wait for commands from other tasks and callbacks. */
        if (CY_RSLT_SUCCESS == cy_rtos_get_queue(  &g_publisher_task_q,
                                                   (void *)&publisher_q_data,
                                                   CY_RTOS_NEVER_TIMEOUT,
                                                   false))
        {
            switch(publisher_q_data.cmd)
            {
                case PUBLISHER_INIT:
                {
                    /* Initialize */
                    publisher_init();
                    break;
                }

                case PUBLISHER_DEINIT:
                {
                    /* Deinit */
                    publisher_deinit();
                    break;
                }

                case PUBLISH_MQTT_MSG:
                {
                    DEBUG_ASSERT(s_publish_topic[0] != '\0');

                    /* Publish the data received over the message queue. */
                    s_publish_info.payload = publisher_q_data.data;
                    s_publish_info.payload_len = strlen(s_publish_info.payload);

                    CY_LOGD(TAG, "Publisher: Publishing '%s' on the topic '%s'\n",
                           (char *) s_publish_info.payload, s_publish_info.topic);

                    result = cy_mqtt_publish(g_mqtt_connection, &s_publish_info);

                    if (result != CY_RSLT_SUCCESS)
                    {
                        CY_LOGD(TAG, "Publisher: MQTT Publish failed with error 0x%0X.\n", (int)result);

                        /* Communicate the publish failure with the the MQTT
                         * client task.
                         */
                        mqtt_task_cmd = HANDLE_MQTT_PUBLISH_FAILURE;

                        if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_mqtt_task_q,
                                                                 (void *)&mqtt_task_cmd,
                                                                 CY_RTOS_NEVER_TIMEOUT,
                                                                 false
                                                                )) {
                            CY_LOGD(TAG, "cy_rtos_put_queue(g_mqtt_task_q) failed!");
                        }
                    }
                    break;
                }
            }
        }
    }
}

bool is_publisher_task_init(void)
{
  return (g_publisher_task_q != NULL);
}

/* [] END OF FILE */
