/******************************************************************************
* File Name:   main.c
*
* Description: This file implements the main function.
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

#include "feature_config.h"

#include <app_utils.h>
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_tls.h"

#include "cyabs_rtos.h"

#include "wiced_bt_stack.h"
#include "cycfg_gatt_db.h"
#include "wifi_task.h"
#include "ppp_task.h"
#include "console_task.h"
#include "capsense_task.h"
#include "led_task.h"

#include "cyabs_rtos.h"
#include "cy_memtrack.h"
#include "cy_debug.h"
#include "cy_conio.h"
#include "cy_sntp_time.h"

#include "app_ble.h"

#include "GUI.h"
#include "mtb_st7789v.h"
#include "cy8ckit_028_tft_pins.h"

#include "mqtt_task.h"
#include "gui_mask_task.h"
#include "flash_eeprom.h"
#include "cy_status.h"
#include "data_store_task.h"
#include "cy_modem.h"

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
#include "optiga/pal/pal_os_event.h"
#include "optiga/pal/pal_i2c.h"
#include "optiga_trust.h"
#include "mqtt_client_config.h"
#endif

/* Queue lengths of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE (1u)

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
#define OPTIGA_TRUST_M_TASK_NAME           "OPTIGA"
#define OPTIGA_TRUST_M_TASK_PRIORITY       (2)
#define OPTIGA_TRUST_M_TASK_STACK_SIZE     (1024 * 12)
#endif

/*-- Public Data -------------------------------------------------*/

/* This enables RTOS aware debugging */
volatile int uxTopUsedPriority;


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "main";

/* The pins are defined by the st7789v library. If the display is being used
 *  on different hardware the mappings will be different. */

const mtb_st7789v_pins_t tft_pins =
{
    .db08 = CY8CKIT_028_TFT_PIN_DISPLAY_DB8,
    .db09 = CY8CKIT_028_TFT_PIN_DISPLAY_DB9,
    .db10 = CY8CKIT_028_TFT_PIN_DISPLAY_DB10,
    .db11 = CY8CKIT_028_TFT_PIN_DISPLAY_DB11,
    .db12 = CY8CKIT_028_TFT_PIN_DISPLAY_DB12,
    .db13 = CY8CKIT_028_TFT_PIN_DISPLAY_DB13,
    .db14 = CY8CKIT_028_TFT_PIN_DISPLAY_DB14,
    .db15 = CY8CKIT_028_TFT_PIN_DISPLAY_DB15,
    .nrd  = CY8CKIT_028_TFT_PIN_DISPLAY_NRD,
    .nwr  = CY8CKIT_028_TFT_PIN_DISPLAY_NWR,
    .dc   = CY8CKIT_028_TFT_PIN_DISPLAY_DC,
    .rst  = CY8CKIT_028_TFT_PIN_DISPLAY_RST
};

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
static cy_thread_t s_optiga_task_handle = NULL;
#endif


/*-- Public Functions -------------------------------------------------*/

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
/* Extract the certificate and use it for the actual communication */

/* This is a place from which we can poll the status of operation */
void vApplicationTickHook( void )
{
    pal_os_event_trigger_registered_callback();
}
#endif

void start_others(void)
{
    cy_rslt_t result;

#if (HAVE_FLASH_EEPROM == 1)
  if (!flash_eeprom_init()) {
    CY_LOGE(TAG, "Error initializing Flash EMEEPROM\n");
    CY_ASSERT(0);
  }
#endif

  if (!ble_init()) {
    CY_LOGE(TAG, "Error initializing BT stack\n");
    CY_ASSERT(0);
  }

  if (cy_tls_init() != CY_RSLT_SUCCESS) /* for RTC initialization */
  {
    CY_LOGD(TAG, "cy_tls_init failed!");
    CY_ASSERT(0);
  }

  // restore time from eeprom
  cy_sntp_set_system_time_callback(0, 0);


#if (FEATURE_CAPSENSE == ENABLE_FEATURE)

  /* Create the queues. See the respective data-types for details of queue
   * contents
   */
  if (CY_RSLT_SUCCESS !=  cy_rtos_init_queue( &g_led_command_data_q,
                                              SINGLE_ELEMENT_QUEUE,
                                              sizeof(led_command_data_t))) {
      CY_LOGD(TAG, "cy_rtos_init_queue(g_led_command_data_q) failed!");
      DEBUG_ASSERT(0);
  }

  if (CY_RSLT_SUCCESS !=  cy_rtos_init_queue( &g_capsense_command_q,
                                              SINGLE_ELEMENT_QUEUE,
                                              sizeof(capsense_command_t))) {
      CY_LOGD(TAG, "cy_rtos_init_queue(g_capsense_command_q) failed!");
      DEBUG_ASSERT(0);
  }

  if (CY_RSLT_SUCCESS !=  cy_rtos_init_queue( &g_gui_mask_command_data_q,
                                              SINGLE_ELEMENT_QUEUE,
                                              sizeof(gui_mask_command_data_t))) {
      CY_LOGD(TAG, "cy_rtos_init_queue(gui_mask_command_data_t) failed!");
      DEBUG_ASSERT(0);
  }

  /* Create the user tasks. See the respective task definition for more
   * details of these tasks.
   */
  result = cy_rtos_create_thread( &g_capsense_task_handle,
                                  task_capsense,
                                  CAPSENSE_TASK_NAME,
                                  NULL,
                                  CAPSENSE_TASK_STACK_SIZE,
                                  CAPSENSE_TASK_PRIORITY,
                                  (cy_thread_arg_t) NULL
                                );
  DEBUG_ASSERT(result == CY_RSLT_SUCCESS);

  result = cy_rtos_create_thread( &g_led_task_handle,
                                  task_led,
                                  LED_TASK_NAME,
                                  NULL,
                                  LED_TASK_STACK_SIZE,
                                  LED_TASK_PRIORITY,
                                  (cy_thread_arg_t) NULL
                                );
  DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#endif

#if (FEATURE_WIFI == ENABLE_FEATURE)
    result = cy_rtos_create_thread( &g_wifi_task_handle,
                                    wifi_task,
                                    WIFI_TASK_NAME,
                                    NULL,
                                    WIFI_TASK_STACK_SIZE,
                                    WIFI_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                  );
    DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#endif

#if (FEATURE_PPP == ENABLE_FEATURE) || (FEATURE_BLE_MODEM == ENABLE_FEATURE)
    // modem is required by PPP or BLE feature
    cy_modem_init();
#endif

#if (FEATURE_PPP == ENABLE_FEATURE)
    result = cy_rtos_create_thread( &g_ppp_task_handle,
                                    ppp_task,
                                    PPP_TASK_NAME,
                                    NULL,
                                    PPP_TASK_STACK_SIZE,
                                    PPP_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                  );
    DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#endif

#if (FEATURE_GUI_MASK_KIOSK == ENABLE_FEATURE)
    result = cy_rtos_create_thread( &g_gui_mask_task_handle,
                                    gui_mask_task,
                                    GUI_MASK_TASK_NAME,
                                    NULL,
                                    GUI_MASK_TASK_STACK_SIZE,
                                    GUI_MASK_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                  );
    DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#endif

#if (FEATURE_MQTT == ENABLE_FEATURE)

    result = cy_rtos_create_thread( &g_mqtt_task_handle,
                                    mqtt_client_task,
                                    MQTT_CLIENT_TASK_NAME,
                                    NULL,
                                    MQTT_CLIENT_TASK_STACK_SIZE,
                                    MQTT_CLIENT_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                  );
    DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#endif

#if (FEATURE_DATA_STORE == ENABLE_FEATURE)
    result = cy_rtos_create_thread( &g_data_store_task_handle,
                                    data_store_task,
                                    DATA_STORE_TASK_NAME,
                                    NULL,
                                    DATA_STORE_TASK_STACK_SIZE,
                                    DATA_STORE_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                  );
    DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#endif

#if (FEATURE_CONSOLE == ENABLE_FEATURE)
    result = cy_rtos_create_thread( &g_console_task_handle,
                                    console_task,
                                    CONSOLE_TASK_NAME,
                                    NULL,
                                    CONSOLE_TASK_STACK_SIZE,
                                    CONSOLE_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                  );
    DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#endif
}

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
//void optiga_client_task(void *pvParameters)
void optiga_client_task(cy_thread_arg_t pvParameters)
{
    pal_i2c_init(NULL);
    optiga_trust_init();
    use_optiga_certificate();

    start_others();

    cy_rtos_terminate_thread(&s_optiga_task_handle);
    s_optiga_task_handle = NULL;
}
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU
*    1. Initializes the BSP
*    2. Configures the button for interrupt
*    3. Initializes retarget IO for UART debug printing
*    4. Initializes platform configuration
*    5. Initializes BT stack and heap
*    6. Creates WiFi connect and disconnect tasks
*    7. Starts the RTOS scheduler
*
* Return:
*  int
*
*******************************************************************************/
int main()
{
  /* This enables RTOS aware debugging in OpenOCD */
  uxTopUsedPriority = configMAX_PRIORITIES - 1 ;

  /* Initialize the board support package */
  if(CY_RSLT_SUCCESS != cybsp_init())
  {
    CY_ASSERT(0);
  }

  ble_button_init();

  /* Enable global interrupts */
  __enable_irq();

#if (FEATURE_GUI_MASK_KIOSK == ENABLE_FEATURE)
  cy_rslt_t result;
  /* Initialize the display controller */
  result = mtb_st7789v_init8(&tft_pins);
  CY_ASSERT(result == CY_RSLT_SUCCESS);

#endif

  /* Initialize retarget-io to use the debug UART port */
  cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                      CY_RETARGET_IO_BAUDRATE);

  DEBUG_PRINT(("\x1b[2J\x1b[;H"));
  DEBUG_PRINT(("===============================================================\n"));
  DEBUG_PRINT(("Mask Vending Machine\n"));
  DEBUG_PRINT(("===============================================================\n\n"));

  CY_MEMTRACK_INITIALIZE();

  status_initialize();

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
    /* Create an OPTIGA task to make sure everything related to
     * the OPTIGA stack will be called from the scheduler */
    result = cy_rtos_create_thread( &s_optiga_task_handle,
                                    optiga_client_task,
                                    OPTIGA_TRUST_M_TASK_NAME,
                                    NULL,
                                    OPTIGA_TRUST_M_TASK_STACK_SIZE,
                                    OPTIGA_TRUST_M_TASK_PRIORITY,
                                    (cy_thread_arg_t) NULL
                                  );
    DEBUG_ASSERT(result == CY_RSLT_SUCCESS);
#else
  start_others();
#endif

  /* Start the FreeRTOS scheduler */
  vTaskStartScheduler() ;

  status_destroy();

  CY_MEMTRACK_DESTROY();

  /* Should never get here */
  CY_ASSERT(0) ;
}

/* [] END OF FILE */
