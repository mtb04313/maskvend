/******************************************************************************
* File Name: gui_mask_task.c
*
* Description: This file contains the task that handles gui mask app.
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
 * Header file includes
 ******************************************************************************/
#include "gui_mask_task.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "display_screen.h"
#include "cy_time_misc.h"
#include "cy_sntp_time.h"
#include "cy_debug.h"
#include "cy_string.h"

#include "publisher_task.h"
#include "kiosk_config.h"
#include "gps_config.h"
#include "cy_modem.h"
#include "flash_eeprom.h"
#include "wifi_task.h"
#include "ppp_task.h"
#include "mask_config.h"
#include "user_button_config.h"


/*******************************************************************************
* Local Definitions
*******************************************************************************/
/* Interrupt priority for User Button Input. */
#define USER_BTN2_INTR_PRIORITY          (5)

// CapSense slider position --> Mask color
#define BLACK_MASK_SLIDER_POSITION   80
#define BLUE_MASK_SLIDER_POSITION    60
#define ORANGE_MASK_SLIDER_POSITION  40
#define PINK_MASK_SLIDER_POSITION    20
#define VIOLET_MASK_SLIDER_POSITION    0

#define STOCK_LEVEL_MAX_LEN   200
#define TIME_MAX_LEN          40

#define GPS_UNKNOWN         "Unknown"
#define ICCID_UNKNOWN       ""

/* clock face timer clock value in Hz  */
#define CLOCK_FACE_TIMER_PERIOD_MSEC      1000
#define CLOCK_FACE_TIMER_CLOCK_HZ         (10000) // 10kHz (won't work if 1kHz is used)
#define CLOCK_FACE_TIMER_INTR_PRIORITY    7
#define MS_TO_COUNT(N)                    ((N)*CLOCK_FACE_TIMER_CLOCK_HZ/1000 - 1)


/* Marketing demo update interval */
#define MARKETING_DEMO_UPDATE_INTERVAL_SEC    10

#define GUI_USER_LEFT_BUTTON_DEBOUNCE_PERIOD_MSEC     500

typedef void (*draw_mask_fn)(void);

/* Structure used for storing mask data */
typedef struct
{
  int color;
  draw_mask_fn draw_fn;
} gui_mask_stock_t;


/*-- Local Function Prototypes -------------------------------------------------*/
static void isr_menu_button_press(void *callback_arg,
                                  cyhal_gpio_event_t event);


/*-- Local Data -------------------------------------------------*/

static const char *TAG = "gui_task";

/* Structure that stores the callback data for the GPIO interrupt event. */
static cyhal_gpio_callback_data_t s_cb_data =
{
    .callback = isr_menu_button_press,
    .callback_arg = NULL
};

static gui_mask_stock_t s_store[] = {
  {MASK_COLOR_VIOLET, violet_mask},
  {MASK_COLOR_PINK,   pink_mask},
  {MASK_COLOR_RED,    red_mask},
  {MASK_COLOR_BLUE,   blue_mask},
  {MASK_COLOR_BLACK,  black_mask},
};

static char s_stockLevelStr[STOCK_LEVEL_MAX_LEN] = "";
static char s_timeStr[TIME_MAX_LEN] = "";

static gui_menu_t s_current_menu = GUI_MENU_STATUS_APP; //GUI_MENU_CLOCK_FACE_APP

/* Timer object used for running a clock face */
static cyhal_timer_t s_clock_timer;
static bool s_is_clock_timer_running = false;
static uint32_t s_clock_timer_tick = 0;

/* flag to indicate we should transition to main app when boot up process is done */
static bool s_auto_switch_to_main_app = true;


/*******************************************************************************
 * Public Data
 ******************************************************************************/
cy_thread_t g_gui_mask_task_handle = NULL;
cy_queue_t g_gui_mask_command_data_q = NULL;


/*******************************************************************************
 * Local Functions
 ******************************************************************************/

static void change_gui_menu(void)
{
  /* if menu has been switched, don't auto-transition */
  s_auto_switch_to_main_app = false;

  s_current_menu++;
  if (s_current_menu == GUI_MENU_INVALID_APP) {
    s_current_menu = GUI_MENU_STATUS_APP;
  }
}

static bool is_gui_mask_task_init(void)
{
  return (g_gui_mask_command_data_q != NULL);
}

static void isr_menu_button_press(void *callback_arg,
                                  cyhal_gpio_event_t event)
{
  if (is_gui_mask_task_init()) {
    gui_mask_command_data_t gui_mask_cmd_data;

    /* To avoid compiler warnings */
    (void) callback_arg;
    (void) event;

    /* Assign the publish command to be sent to the publisher task. */
    gui_mask_cmd_data.command = GUI_USER_LEFT_BUTTON_PRESSED;

    /* Send the command to gui mask task over the queue */
    if (CY_RSLT_SUCCESS != cy_rtos_put_queue(   &g_gui_mask_command_data_q,
                                                (void *)&gui_mask_cmd_data,
                                                0,
                                                true
                                            )) {
    }
  }
}

static void menu_button_init(void)
{
  /* Initialize the user button GPIO and register interrupt on falling edge. */
  cyhal_gpio_init(USER_BUTTON_CHANGE_GUI_SCREEN,
                  CYHAL_GPIO_DIR_INPUT,
                  CYHAL_GPIO_DRIVE_PULLUP,
                  CYBSP_BTN_OFF);

  cyhal_gpio_register_callback(USER_BUTTON_CHANGE_GUI_SCREEN, &s_cb_data);

  cyhal_gpio_enable_event(USER_BUTTON_CHANGE_GUI_SCREEN,
                          CYHAL_GPIO_IRQ_FALL,
                          USER_BTN2_INTR_PRIORITY,
                          true);

  DEBUG_PRINT(("Press the user button (SW4) to change menu\n\n"));
}

#if 0
static void menu_button_deinit(void)
{
  /* Deregister the ISR and disable the interrupt on the user button. */
  cyhal_gpio_register_callback( USER_BUTTON_CHANGE_GUI_SCREEN,
                                NULL);

  cyhal_gpio_enable_event(USER_BUTTON_CHANGE_GUI_SCREEN,
                          CYHAL_GPIO_IRQ_FALL,
                          USER_BTN2_INTR_PRIORITY,
                          false);

  cyhal_gpio_free(USER_BUTTON_CHANGE_GUI_SCREEN);
}
#endif


static void publish_mask_stock_level(void)
{
  if (is_publisher_task_init()) {
    publisher_data_t publisher_q_data;

    /* Assign the publish command to be sent to the publisher task. */
    publisher_q_data.cmd = PUBLISH_MQTT_MSG;
    publisher_q_data.data = (char *)get_mask_stock_level();

    /* Send the command and data to publisher task over the queue */
    if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_publisher_task_q,
                                             (void *)&publisher_q_data,
                                             CY_RTOS_NEVER_TIMEOUT,
                                             false)) {
        CY_LOGD(TAG, "cy_rtos_put_queue(g_publisher_task_q) failed!");
    }
  }
}


// return true if there are still masks of this color in the store
static bool draw_mask(int mask_color)
{
  bool result;
  int16_t qty = -1;
  int i;
  draw_mask_fn draw_fn = NULL;

  for (i = 0; i < sizeof(s_store)/sizeof(s_store[0]); i++) {
    if (s_store[i].color == mask_color) {
      //count = s_store[i].count;
      draw_fn = s_store[i].draw_fn;
      break;
    }
  }

  result = flash_eeprom_get_mask_qty( mask_color,
                                      &qty,
                                      sizeof(qty));

  ReturnAssert(result, false);
  ReturnAssert(qty >= 0, false);
  ReturnAssert(draw_fn != NULL, false);

  draw_fn();
  if (qty == 0) {
    show_text_out_of_stock();
  }

  return (qty != 0);
}

static bool dispense_mask(int mask_color)
{
  bool result;
  int16_t qty = -1;

  result = flash_eeprom_get_mask_qty( mask_color,
                                      &qty,
                                      sizeof(qty));
  ReturnAssert(result, false);

  if (qty > 0) {
    qty--;
    result = flash_eeprom_set_mask_qty( mask_color,
                                        &qty,
                                        sizeof(qty),
                                        false);
    ReturnAssert(result, false);
  }
  else {
    result = false;
  }

  if (result) {
    show_text_mask_dispensed();
    msleep(2000);
    mask_home_screen();
    publish_mask_stock_level();
  }

  return result;
}


static int get_mask_color(uint32_t position)
{
  int temp_color;

  if (position > BLACK_MASK_SLIDER_POSITION) {
    temp_color = MASK_COLOR_BLACK;
  }
  else if (position > BLUE_MASK_SLIDER_POSITION) {
    temp_color = MASK_COLOR_BLUE;
  }
  else if (position > ORANGE_MASK_SLIDER_POSITION) {
    temp_color = MASK_COLOR_RED;
  }
  else if (position > PINK_MASK_SLIDER_POSITION) {
    temp_color = MASK_COLOR_PINK;
  }
  else {
    temp_color = MASK_COLOR_VIOLET;
  }

  return temp_color;
}


static bool encode_mask_stock_level(
  char *jsonStr_p,
  size_t jsonStrSize,
  const char *time_p,
  int16_t id,
  const char *gps_p,
  const gui_mask_stock_t *maskData_p,
  size_t maskDataCount,
  const char *iccid_p)
{
  char maskDataStr[80] = "";
  size_t i;

  ReturnAssert(jsonStr_p != NULL, false);
  ReturnAssert(time_p != NULL, false);
  ReturnAssert(gps_p != NULL, false);
  ReturnAssert(maskData_p != NULL, false);
  ReturnAssert(iccid_p != NULL, false);

  for (i = 0; i < maskDataCount; i++) {
    bool result;
    int16_t qty = -1;

    result = flash_eeprom_get_mask_qty( maskData_p[i].color,
                                        &qty,
                                        sizeof(qty));
    ReturnAssert(result, false);

    char tempStr[20] = "";
    SNPRINTF(tempStr, sizeof(tempStr), "\"%c\":%d",
        maskData_p[i].color, qty);

    DEBUG_ASSERT(strlen(tempStr) < (sizeof(tempStr) - 1));
    if (i < (maskDataCount - 1)) {
      strcat(tempStr, ",");
    }

    DEBUG_ASSERT((strlen(maskDataStr) + strlen(tempStr)) < (sizeof(maskDataStr) - 1));
    strcat(maskDataStr, tempStr);
  }

  SNPRINTF(
    jsonStr_p,
    jsonStrSize,
    "{\"time\":\"%s\",\"id\":%d,\"gps\":\"%s\",\"maskData\":{%s},\"iccid\":\"%s\"}",
    time_p,
    id,
    (strlen(gps_p) > 0)? gps_p : GPS_UNKNOWN,
    maskDataStr,
    (strlen(iccid_p) > 0)? iccid_p : ICCID_UNKNOWN);

  return true;
}


/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_clock_timer(void *callback_arg,
                            cyhal_timer_event_t event)
{

  (void) callback_arg;
  (void) event;

  /* Set the interrupt flag and process it from the main while(1) loop */
  s_clock_timer_tick++;

  /* Send command to process timer tick */
  gui_mask_command_data_t gui_mask_cmd_data = {GUI_CLOCK_TIMER_TICK, 0};

  if (CY_RSLT_SUCCESS != cy_rtos_put_queue(   &g_gui_mask_command_data_q,
                                              (void *)&gui_mask_cmd_data,
                                              0,
                                              true
                                          )) {
  }
}

#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
static uint32_t get_clock_timer_tick(void)
{
  return s_clock_timer_tick;
}
#endif

/*******************************************************************************
* Function Name: clock_timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks
* continuously and produces a periodic interrupt on every terminal count
* event. The period is defined by the 'period' and 'compare_value' of the
* timer configuration structure 'led_blink_timer_cfg'. Without any changes,
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
static void clock_timer_init(uint32_t duration_ms)
{
  cy_rslt_t result;

  const cyhal_timer_cfg_t clock_timer_cfg =
  {
    .compare_value = 0,                 /* Timer compare value, not used */
    .period = MS_TO_COUNT(duration_ms), /* Defines the timer period */
    .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
    .is_compare = false,                /* Don't use compare mode */
    .is_continuous = true,              /* Run timer indefinitely */
    .value = 0                          /* Initial value of counter */
  };

  /* Initialize the timer object. Does not use pin output ('pin' is NC) and
   * does not use a pre-configured clock source ('clk' is NULL). */
  result = cyhal_timer_init(&s_clock_timer,
                            NC,
                            NULL);

  /* timer init failed. Stop program execution */
  if (result != CY_RSLT_SUCCESS)
  {
    CY_ASSERT(0);
  }

  /* Configure timer period and operation mode such as count direction,
     duration */
  cyhal_timer_configure(&s_clock_timer,
                        &clock_timer_cfg);

  /* Set the frequency of timer's clock source */
  cyhal_timer_set_frequency(&s_clock_timer,
                            CLOCK_FACE_TIMER_CLOCK_HZ);

  /* Assign the ISR to execute on timer interrupt */
  cyhal_timer_register_callback(&s_clock_timer,
                                isr_clock_timer,
                                NULL);

  /* Set the event on which timer interrupt occurs and enable it */
  cyhal_timer_enable_event( &s_clock_timer,
                            CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                            CLOCK_FACE_TIMER_INTR_PRIORITY,
                            true);
}

static void clock_timer_cleanup(void)
{
  cyhal_timer_free(&s_clock_timer);
}

static void clock_timer_start(void)
{
  cyhal_timer_start(&s_clock_timer);
  s_is_clock_timer_running = true;
}

static void clock_timer_stop(void)
{
  cyhal_timer_stop(&s_clock_timer);
  s_is_clock_timer_running = false;
}

static bool is_clock_timer_running(void)
{
  return s_is_clock_timer_running;
}

#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
static void show_next_marketing_screen(void)
{
  static int screen_id = 0;
  screen_id++;

  if (screen_id == 1) {
    marketing_screen_1();
  }
  else {
    marketing_screen_2();
    screen_id = 0;
  }
}
#endif

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*******************************************************************************
* Function Name: gui_mask_task
********************************************************************************
* Summary:
*  Task that controls the GUI Mask app
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
*******************************************************************************/
void gui_mask_task(cy_thread_arg_t param)
{
    int mask_color = MASK_COLOR_UNKNOWN;
    bool tick_on = false;
    bool has_stock = false;
    gui_mask_command_data_t gui_mask_cmd_data;

    /* Suppress warning for unused parameter */
    (void)param;

    GUI_Init();

    /* Initialize and set-up the user button GPIO. */
    menu_button_init();

    draw_status(true);

    /* Initialize timer */
    clock_timer_init(CLOCK_FACE_TIMER_PERIOD_MSEC);

    clock_timer_start();
    //network_info_screen();
    //mask_home_screen();
    //publish_mask_stock_level();

    /* Repeatedly running part of the task */
    for(;;)
    {
        /* Block until a command has been received over queue */
        if (CY_RSLT_SUCCESS == cy_rtos_get_queue(   &g_gui_mask_command_data_q,
                                                    (void *)&gui_mask_cmd_data,
                                                    CY_RTOS_NEVER_TIMEOUT,
                                                    false))

        {
            switch(gui_mask_cmd_data.command)
            {
                /* Menu Button pressed */
                case GUI_USER_LEFT_BUTTON_PRESSED:
                {
                    static long s_leftButtonPressedTime = 0;
                    long currTime = current_time();
                    long deltaTime = currTime - s_leftButtonPressedTime;

                    if ((deltaTime > 0) &&
                        (deltaTime < GUI_USER_LEFT_BUTTON_DEBOUNCE_PERIOD_MSEC)) {
                      DEBUG_PRINT(("Menu Button Pressed: %ld (ignored)\n", deltaTime));
                    }
                    else {
                      DEBUG_PRINT(("Menu Button Pressed: %ld\n", deltaTime));
                      s_leftButtonPressedTime = currTime;

                      change_gui_menu();

                      if (s_current_menu == GUI_MENU_MASK_APP) {
                        mask_home_screen();
                        tick_on = false;
                        has_stock = false;
                        mask_color = MASK_COLOR_UNKNOWN;
                      }

#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
                      else if (s_current_menu == GUI_MENU_MARKETING_APP) {
                        marketing_screen_1();
                      }
#endif
                      else if (s_current_menu == GUI_MENU_CLOCK_FACE_APP) {
                        draw_clock_face(true);
                      }
                      else if (s_current_menu == GUI_MENU_STATUS_APP) {
                        draw_status(true);
                      }
                      else if (s_current_menu == GUI_MENU_QRCODE_APP) {
                        show_qrcode(true);
                      }
                      else {
                        network_info_screen();
                      }
                    }

                    break;
                }

                /* OK button pressed */
                case GUI_CAPSENSE_OK:
                {
                  if (s_current_menu == GUI_MENU_MASK_APP) {
                    if (tick_on) {
                      bool result = dispense_mask(mask_color);
                      DEBUG_ASSERT(result);

                      tick_on = false;
                      has_stock = false;
                      mask_color = MASK_COLOR_UNKNOWN;
                    }
                    else {
                      if (has_stock) {
                        tick_on = true;
                        green_tick_on();
                      }
                    }
                  }
                  break;
                }

                /* Cancel button pressed */
                case GUI_CAPSENSE_CANCEL:
                {
                  if (s_current_menu == GUI_MENU_MASK_APP) {
                    if (tick_on) {
                      tick_on = false;
                      green_tick_off();
                    }
                    else {
                      mask_home_screen();
                      tick_on = false;
                      has_stock = false;
                      mask_color = MASK_COLOR_UNKNOWN;
                    }
                  }
                  break;
                }

                /* Update position */
                case GUI_CAPSENSE_UPDATE_POSITION:
                {
                  if (s_current_menu == GUI_MENU_MASK_APP) {
                    if (gui_mask_cmd_data.position > 0)
                    {
                        uint32_t position = (gui_mask_cmd_data.position < GUI_MASK_MIN_POSITION) ?
                                               GUI_MASK_MIN_POSITION : gui_mask_cmd_data.position;

                        int temp_color = get_mask_color(position);

                        if (temp_color != mask_color) {
                          mask_color = temp_color;
                          has_stock = draw_mask(mask_color);
                          tick_on = false;
                        }
                    }
                  }
                  break;
                }

                /* timer tick */
                case GUI_CLOCK_TIMER_TICK:
                {
                  //DEBUG_PRINT(("GUI_CLOCK_TIMER_TICK: %ld\n", get_clock_timer_tick()));

                  if (s_current_menu == GUI_MENU_STATUS_APP) {
                    draw_status(false);

                    if (s_auto_switch_to_main_app) {
                      bool boot_completed;

#if (FEATURE_PPP == ENABLE_FEATURE)
                      boot_completed = is_ppp_connected();
#elif (FEATURE_WIFI == ENABLE_FEATURE)
                      boot_completed = is_wifi_connected();
#else
                      boot_completed = false;
#endif

                      /* check if boot up process is completed */
                      if (boot_completed) {
                          /* fake a button press */
                          gui_mask_command_data_t gui_mask_cmd_data = {GUI_USER_LEFT_BUTTON_PRESSED, 0};

                          if (CY_RSLT_SUCCESS != cy_rtos_put_queue(   &g_gui_mask_command_data_q,
                                                                      (void *)&gui_mask_cmd_data,
                                                                      0,
                                                                      false
                                                                  )) {
                              CY_LOGD(TAG, "cy_rtos_put_queue(g_gui_mask_command_data_q) failed!");
                          }
                      }
                    }
                  }
                  else if (s_current_menu == GUI_MENU_CLOCK_FACE_APP) {
                    draw_clock_face(false);
                  }

#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
                  else if (s_current_menu == GUI_MENU_MARKETING_APP) {
                    if ((get_clock_timer_tick() % MARKETING_DEMO_UPDATE_INTERVAL_SEC) == 0) {
                      show_next_marketing_screen();
                    }
                  }
#endif
                  break;
                }

                /* Publish the current mask stock level */
                case GUI_PUBLISH_MASK_STOCK_LEVEL:
                {
                  publish_mask_stock_level();
                  break;
                }

                /* Invalid command */
                default:
                {
                  /* Handle invalid command here */
                  break;
                }
            }
        }

        /* Task has timed out and received no data during an interval of
         * portMAXDELAY ticks.
         */
        else
        {
            /* Handle timeout here */
        }
    }

    if (is_clock_timer_running()) {
      clock_timer_stop();
    }

    clock_timer_cleanup();
}


const char* get_mask_stock_level(void)
{
  int16_t id = DEFAULT_KIOSK_ID;
  char location[GPS_INFO_MAX_LEN + 1] = "";

#if (HAVE_FLASH_EEPROM == 1)
  if (!flash_eeprom_get_kiosk_id(&id, sizeof(id))) {
    DEBUG_PRINT(("flash_eeprom_get_kiosk_id failed\n"));
  }

  if (!flash_eeprom_get_gps_location( location,
                                      sizeof(location))) {
    DEBUG_PRINT(("flash_eeprom_get_gps_location failed\n"));
  }
#endif

  cy_sntp_get_current_time_long( s_timeStr,
                                 sizeof(s_timeStr));

  encode_mask_stock_level(
      s_stockLevelStr,  // output string
      sizeof(s_stockLevelStr),
      s_timeStr,
      id,
      location,
      s_store,
      sizeof(s_store)/sizeof(s_store[0]),
      cy_modem_get_iccid());

  return s_stockLevelStr;
}

bool send_gui_publish_mask_stock_level(void)
{
  gui_mask_command_data_t gui_mask_cmd_data;
  gui_mask_cmd_data.command = GUI_PUBLISH_MASK_STOCK_LEVEL;

  if (CY_RSLT_SUCCESS != cy_rtos_put_queue(   &g_gui_mask_command_data_q,
                                              (void *)&gui_mask_cmd_data,
                                              0,
                                              false
                                          )) {
      CY_LOGD(TAG, "cy_rtos_put_queue(g_gui_mask_command_data_q) failed!");
      return false;
  }
  return true;
}

/* END OF FILE [] */
