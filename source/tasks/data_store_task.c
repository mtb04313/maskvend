/******************************************************************************
* File Name:   data_store_task.c
*
* Description: This file contains declaration of task related to using the
* data store.
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
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
*******************************************************************************/

//#include "feature_config.h"

#include "data_store_task.h"
#include "app_ble.h"

#include <string.h>

#include "cy_memtrack.h"
#include "cy_debug.h"

#include "cy_conio.h"
#include "cy_string.h"
#include "cy_time_misc.h"

#include "cycfg_gatt_db.h"
#include "wiced_bt_stack.h"
#include "app_utils.h"
#include "wifi_task.h"
#include "common_task.h"

#include "ppp_config.h"
#include "wiced_bt_ble.h"

#include "cy_misc_types.h"
#include "flash_eeprom.h"
#include "gps_config.h"
#include "time_config.h"
#include "mask_config.h"
#include "url_config.h"

#include "publisher_task.h"
#include "gui_mask_task.h"
#include "display_screen.h"
#include "cy_modem.h"
#include "cy_pcm.h"


/*-- Local Definitions -------------------------------------------------*/

// lengths
#define DS_MAX_PACKET_LEN                 app_data_store_service_read_write_item_len
#define DS_MAX_DATA_LEN                   (DS_MAX_PACKET_LEN - 3)
#define DS_READ_REQUEST_PACKET_LEN        2
#define DS_WRITE_REQUEST_MIN_PACKET_LEN   3

// types of packets
#define DS_OP_WRITE_REQUEST   0x66
#define DS_OP_WRITE_RESPONSE  0x6F
#define DS_OP_READ_REQUEST    0x99
#define DS_OP_READ_RESPONSE   0x9F

// read/write outcome
#define DS_OP_SUCCESS         0x01
#define DS_OP_FAILURE         0x00

// DS IDs
#define DS_DEVICE_ID          0x00
#define DS_GPS_COORDS         0x01
#define DS_TIMESTAMP          0x02
#define DS_TIMEZONE_DIFF      0x03
#define DS_WIFI_SSID          0x04
#define DS_WIFI_PASSWORD      0x05
#define DS_WIFI_SECURITY      0x06
#define DS_URL_PATTERN        0x07
#define DS_IS_WIFI_ENABLED    0x08
#define DS_IPV4_ADDRESS       0x09

#define DS_MASK_V             0x10
#define DS_MASK_P             0x11
#define DS_MASK_R             0x12
#define DS_MASK_B             0x13
#define DS_MASK_K             0x14

#define DS_APN                0x15
#define DS_OPERATOR           0x16
#define DS_UE_SYS_INFO        0x17
#define DS_ICCID              0x18

#define DS_INSTRUCTION        0x20

// DS Instruction Types
#define DS_INS_PUBLISH_MASK_STOCK_LEVEL   0x00


/*-- Public Data -------------------------------------------------*/

cy_thread_t g_data_store_task_handle = NULL;


/*-- Local Data -------------------------------------------------*/

#if (FEATURE_BLE_APP == ENABLE_FEATURE)
static const char *TAG = "data_store_task";

/* Structure used to map dsId and mask color */
typedef struct
{
  int color;
  uint8_t dsId;
} mask_color_data_t;

static mask_color_data_t s_mask_color_map_t[] = {
  {MASK_COLOR_VIOLET, DS_MASK_V},
  {MASK_COLOR_PINK,   DS_MASK_P},
  {MASK_COLOR_RED,    DS_MASK_R},
  {MASK_COLOR_BLUE,   DS_MASK_B},
  {MASK_COLOR_BLACK,  DS_MASK_K},
};
#endif


/*-- Local Functions -------------------------------------------------*/

#if (FEATURE_BLE_APP == ENABLE_FEATURE)

// return true if the mask color is found
static bool get_mask_color(uint8_t dsId, int *mask_color_p)
{
  int i;
  ReturnAssert(mask_color_p != NULL, false);

  for (i = 0; i < sizeof(s_mask_color_map_t)/sizeof(s_mask_color_map_t[0]); i++) {
    if (s_mask_color_map_t[i].dsId == dsId) {
      *mask_color_p = s_mask_color_map_t[i].color;
      return true;
    }
  }

  return false;
}

static void update_publisher_topic(void)
{
  if (is_publisher_task_init()) {
    publisher_data_t publisher_q_data;

    /* Assign the publish command to be sent to the publisher task. */
    publisher_q_data.cmd = PUBLISHER_INIT;

    /* Send the command and data to publisher task over the queue */
    if (CY_RSLT_SUCCESS != cy_rtos_put_queue(&g_publisher_task_q,
                                             (void *)&publisher_q_data,
                                             CY_RTOS_NEVER_TIMEOUT,
                                             false)) {
        CY_LOGD(TAG, "cy_rtos_put_queue(g_publisher_task_q) failed!");
    }
  }
}

static bool ds_write_item(uint8_t dsId,
                          uint8_t dataLen,
                          const uint8_t *data_p)
{
  bool result = false;

  ReturnAssert(dataLen != 0, false);
  ReturnAssert(data_p != NULL, false);

#if (HAVE_FLASH_EEPROM == 1)

  switch (dsId) {
  case DS_DEVICE_ID: {
    ReturnAssert(dataLen == 2, false);

    int16_t id = (int16_t)MAKE_UWORD(data_p[1], data_p[0]);
    CY_LOGD(TAG, "DS_DEVICE_ID = 0x%04x, %d", (uint16_t)id, id);

    result = flash_eeprom_set_kiosk_id( &id,
                                        sizeof(id),
                                        false);

    /* inform others that need to use the updated id */
    update_publisher_topic();

    break;
  }

  case DS_GPS_COORDS: {
    ReturnAssert(dataLen > GPS_INFO_MIN_LEN, false);
    ReturnAssert(dataLen <= GPS_INFO_MAX_LEN, false);

    char location[GPS_INFO_MAX_LEN + 1];
    memset(location, 0, sizeof(location));
    memcpy(location, data_p, dataLen);

    CY_LOGD(TAG, "DS_GPS_COORDS = %s", location);

    result = flash_eeprom_set_gps_location(location);
    break;
  }

  case DS_TIMESTAMP: {
    ReturnAssert(dataLen == 4, false);

    int32_t timestamp = (int32_t)MAKE_ULONG(MAKE_UWORD(data_p[3], data_p[2]),  //uwHigh
                                            MAKE_UWORD(data_p[1], data_p[0])); //uwLow
    CY_LOGD(TAG, "timestamp = %ld", timestamp);

    /* first read, then override timestamp with the new one */
    cy_time_info_t time_info;
    memset(&time_info, 0, sizeof(time_info));
    result = flash_eeprom_get_time_info(&time_info,
                                        sizeof(time_info));

    if (result) {
      time_info.timestamp = timestamp;
      result = flash_eeprom_set_time_info(&time_info,
                                          sizeof(time_info),
                                          false);
    }
    break;
  }

  case DS_TIMEZONE_DIFF: {
    ReturnAssert(dataLen == 4, false);

    /* convert from integer to float by dividing by 10 */
    int32_t iDiff = (int32_t)MAKE_ULONG(MAKE_UWORD(data_p[3], data_p[2]),  //uwHigh
                                        MAKE_UWORD(data_p[1], data_p[0])); //uwLow
    float timezone_diff = (float)iDiff / 10.0;

    CY_LOGD(TAG, "timezone_diff = %f", timezone_diff);

    /* first read, then override timestamp with the new one */
    cy_time_info_t time_info;
    memset(&time_info, 0, sizeof(time_info));
    result = flash_eeprom_get_time_info(&time_info,
                                        sizeof(time_info));

    if (result) {
      float previous_diff = time_info.timezone_diff;
      time_info.timezone_diff = timezone_diff;
      CY_LOGD(TAG, "time_info.timezone_diff = %f", time_info.timezone_diff);
      result = flash_eeprom_set_time_info(&time_info,
                                          sizeof(time_info),
                                          false);

      if (result) {
        // inform time component to update RTC
        result = cy_sntp_handle_timezone_change( previous_diff,
                                                 timezone_diff);
      }
    }
    break;
  }

  case DS_WIFI_SSID: {
    ReturnAssert(dataLen > 0, false);
    ReturnAssert(dataLen <= CY_WCM_MAX_SSID_LEN, false);

    char ssid[CY_WCM_MAX_SSID_LEN + 1];
    memset(ssid, 0, sizeof(ssid));
    memcpy(ssid, data_p, dataLen);

    CY_LOGD(TAG, "DS_WIFI_SSID = %s", ssid);

    result = flash_eeprom_set_wifi_ssid(ssid);
    break;
  }

  case DS_WIFI_PASSWORD: {
    ReturnAssert(dataLen > 0, false);
    ReturnAssert(dataLen <= CY_WCM_MAX_PASSPHRASE_LEN, false);

    char password[CY_WCM_MAX_PASSPHRASE_LEN + 1];
    memset(password, 0, sizeof(password));
    memcpy(password, data_p, dataLen);

    CY_LOGD(TAG, "DS_WIFI_PASSWORD = %s", password);

    result = flash_eeprom_set_wifi_password(password);
    break;
  }

  case DS_WIFI_SECURITY: {
    ReturnAssert(dataLen == 4, false);

    uint32_t security_type = (int32_t)MAKE_ULONG(MAKE_UWORD(data_p[3], data_p[2]),  //uwHigh
                                                 MAKE_UWORD(data_p[1], data_p[0])); //uwLow
    CY_LOGD(TAG, "security_type = %lu", security_type);

    result = flash_eeprom_set_wifi_security_type(security_type);
    break;
  }

  case DS_MASK_V:
  case DS_MASK_P:
  case DS_MASK_R:
  case DS_MASK_B:
  case DS_MASK_K:
  {
    ReturnAssert(dataLen == 2, false);

    int mask_color;
    result = get_mask_color(dsId, &mask_color);
    ReturnAssert(result, false);

    int16_t qty = (int16_t)MAKE_UWORD(data_p[1], data_p[0]);
    CY_LOGD(TAG, "DS_MASK_%c = 0x%04x, %d", mask_color, (uint16_t)qty, qty);

    result = flash_eeprom_set_mask_qty( mask_color,
                                        &qty,
                                        sizeof(qty),
                                        false);
    break;
  }

  case DS_URL_PATTERN: {
    ReturnAssert(dataLen > URL_INFO_MIN_LEN, false);
    ReturnAssert(dataLen <= URL_INFO_MAX_LEN, false);

    char url[URL_INFO_MAX_LEN + 1];
    memset(url, 0, sizeof(url));
    memcpy(url, data_p, dataLen);

    CY_LOGD(TAG, "DS_URL_PATTERN = %s", url);

    result = flash_eeprom_set_url_pattern(url);
    break;
  }

  case DS_INSTRUCTION:
  {
    ReturnAssert(dataLen == 2, false);

    uint16_t instruction = MAKE_UWORD(data_p[1], data_p[0]);
    CY_LOGD(TAG, "DS_INSTRUCTION = 0x%04x", instruction);

    if (instruction == DS_INS_PUBLISH_MASK_STOCK_LEVEL) {
      result = send_gui_publish_mask_stock_level();
    }
    break;
  }

  default:
    break;
  }

#endif

  return result;
}

static bool ds_read_item( uint8_t dsId,
                          uint8_t *dataLen_p,
                          uint8_t *data_p)
{
  bool result = false;

  ReturnAssert(dataLen_p != NULL, false);
  ReturnAssert(data_p != NULL, false);

#if (HAVE_FLASH_EEPROM == 1)

  switch (dsId) {
  case DS_DEVICE_ID: {
    int16_t id;
    result = flash_eeprom_get_kiosk_id( &id,
                                        sizeof(id));
    if (result) {
      ReturnAssert(*dataLen_p >= 2, false);

      data_p[0] = LOBYTE((uint16_t)id);
      data_p[1] = HIBYTE((uint16_t)id);
      *dataLen_p = 2;
    }
    else {
      *dataLen_p = 0;
    }
    break;
  }

  case DS_GPS_COORDS: {
    char *location_p = (char*)data_p;
    result = flash_eeprom_get_gps_location( location_p,
                                            *dataLen_p);

    *dataLen_p = result? strlen(location_p) : 0;
    break;
  }

  case DS_TIMESTAMP: {
    cy_time_info_t time_info;

    memset(&time_info, 0, sizeof(time_info));
    result = flash_eeprom_get_time_info(&time_info,
                                        sizeof(time_info));

    if (result) {
      uint32_t temp = (uint32_t)time_info.timestamp;

      ReturnAssert(*dataLen_p >= 4, false);

      data_p[0] = LOBYTE(LOWORD(temp));
      data_p[1] = HIBYTE(LOWORD(temp));
      data_p[2] = LOBYTE(HIWORD(temp));
      data_p[3] = HIBYTE(HIWORD(temp));
      *dataLen_p = 4;
    }
    else {
      *dataLen_p = 0;
    }
    break;
  }

  case DS_TIMEZONE_DIFF: {
    cy_time_info_t time_info;

    memset(&time_info, 0, sizeof(time_info));
    result = flash_eeprom_get_time_info(&time_info,
                                        sizeof(time_info));

    if (result) {
      CY_LOGD(TAG, "time_info.timezone_diff = %f", time_info.timezone_diff);

      /* convert float to integer by multiplying by 10 */
      int32_t iTemp = (int32_t)(time_info.timezone_diff * 10.0);
      CY_LOGD(TAG, "iTemp = %ld", iTemp);

      uint32_t ulTemp = (uint32_t)iTemp;
      CY_LOGD(TAG, "ulTemp = %lu", ulTemp);

      ReturnAssert(*dataLen_p >= 4, false);

      data_p[0] = LOBYTE(LOWORD(ulTemp));
      data_p[1] = HIBYTE(LOWORD(ulTemp));
      data_p[2] = LOBYTE(HIWORD(ulTemp));
      data_p[3] = HIBYTE(HIWORD(ulTemp));
      *dataLen_p = 4;
    }
    else {
      *dataLen_p = 0;
    }
    break;
  }

  case DS_WIFI_SSID: {
    char *ssid_p = (char*)data_p;
    result = flash_eeprom_get_wifi_ssid(ssid_p,
                                        *dataLen_p);

    *dataLen_p = result? strlen(ssid_p) : 0;
    break;
  }

  case DS_WIFI_PASSWORD: {
    char *password_p = (char*)data_p;
    result = flash_eeprom_get_wifi_password(password_p,
                                            *dataLen_p);

    *dataLen_p = result? strlen(password_p) : 0;
    break;
  }

  case DS_WIFI_SECURITY: {
    uint32_t security_type;
    result = flash_eeprom_get_wifi_security_type( &security_type);
    if (result) {
      ReturnAssert(*dataLen_p >= 4, false);

      data_p[0] = LOBYTE(LOWORD(security_type));
      data_p[1] = HIBYTE(LOWORD(security_type));
      data_p[2] = LOBYTE(HIWORD(security_type));
      data_p[3] = HIBYTE(HIWORD(security_type));
      *dataLen_p = 4;
    }
    else {
      *dataLen_p = 0;
    }
    break;
  }

  case DS_MASK_V:
  case DS_MASK_P:
  case DS_MASK_R:
  case DS_MASK_B:
  case DS_MASK_K:
  {
    int mask_color;
    result = get_mask_color(dsId, &mask_color);
    ReturnAssert(result, false);

    int16_t qty;
    result = flash_eeprom_get_mask_qty( mask_color,
                                        &qty,
                                        sizeof(qty));
    if (result) {
      ReturnAssert(*dataLen_p >= 2, false);

      data_p[0] = LOBYTE((uint16_t)qty);
      data_p[1] = HIBYTE((uint16_t)qty);
      *dataLen_p = 2;
    }
    else {
      *dataLen_p = 0;
    }
    break;
  }

  case DS_URL_PATTERN: {
    char *url_p = (char*)data_p;
    result = flash_eeprom_get_url_pattern(url_p,
                                          *dataLen_p);

    *dataLen_p = result? strlen(url_p) : 0;
    break;
  }

  case DS_IS_WIFI_ENABLED: {
    ReturnAssert(*dataLen_p >= 1, false);

#if (FEATURE_WIFI == ENABLE_FEATURE)
    if (cy_pcm_get_default_connectivity() == WIFI_STA_CONNECTIVITY) {
        data_p[0] = 1;
    }
    else {
        data_p[0] = 0;
    }
#else
    data_p[0] = 0;
#endif

    *dataLen_p = 1;
    result = true;
    break;
  }

  case DS_IPV4_ADDRESS: {
    char *ipv4_p = (char*)data_p;
    ReturnAssert(*dataLen_p >= IPV4_ADDRESS_STR_LEN, false);

    read_one_ip_address(ipv4_p,
                        *dataLen_p);

    *dataLen_p = strlen(ipv4_p);
    result = true;
    break;
  }

  case DS_APN: {
    char *apn_p = (char*)data_p;
    int len = strlen(PPP_APN);

    if (len > 0) {
      ReturnAssert(*dataLen_p > len, false);

      SNPRINTF(apn_p, *dataLen_p, "%s", PPP_APN);
      *dataLen_p = len;
    }
    else {
      *dataLen_p = 0;
    }
    result = true;
    break;
  }

  case DS_OPERATOR: {
    char *opr_p = (char*)data_p;
    const char* ptr = cy_modem_get_operator_name();
    int len = strlen(ptr);

    if (len > 0) {
      ReturnAssert(*dataLen_p > len, false);

      SNPRINTF(opr_p, *dataLen_p, "%s", ptr);
      *dataLen_p = len;
    }
    else {
      *dataLen_p = 0;
    }
    result = true;
    break;
  }

  case DS_UE_SYS_INFO: {
    char *sys_p = (char*)data_p;
    const char* ptr = cy_modem_get_ue_system_info();
    int len = strlen(ptr);

    if (len > 0) {
      ReturnAssert(*dataLen_p > len, false);

      SNPRINTF(sys_p, *dataLen_p, "%s", ptr);
      *dataLen_p = len;
    }
    else {
      *dataLen_p = 0;
    }
    result = true;
    break;
  }

  case DS_ICCID: {
    char *sys_p = (char*)data_p;
    const char* ptr = cy_modem_get_iccid();
    int len = strlen(ptr);

    if (len > 0) {
      ReturnAssert(*dataLen_p > len, false);

      SNPRINTF(sys_p, *dataLen_p, "%s", ptr);
      *dataLen_p = len;
    }
    else {
      *dataLen_p = 0;
    }
    result = true;
    break;
  }

  default:
    *dataLen_p = 0;
    break;
  }

#endif

  return result;
}

// send notification in chunks
static void update_gatt_db_rw_item( uint8_t operationResponse,
                                    uint8_t dsId,
                                    uint8_t responseLen,
                                    uint8_t *response_p)
{
  VoidAssert(response_p != NULL);

  /* Check if the connection is active, notifications are enabled */
  if ((g_conn_id != 0) &&
      (app_data_store_service_read_write_item_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION))
  {
    uint16_t tempLen;
    tempLen = responseLen + 3;

    VoidAssert(tempLen <= app_data_store_service_read_write_item_len);

    memset( app_data_store_service_read_write_item,
            0,
            app_data_store_service_read_write_item_len);

    app_data_store_service_read_write_item[0] = operationResponse;
    app_data_store_service_read_write_item[1] = dsId;
    app_data_store_service_read_write_item[2] = responseLen;

    memcpy( &app_data_store_service_read_write_item[3],
            response_p,
            responseLen);

    /* Send notification */
    //CY_LOGD(TAG, "g_conn_id = %d", g_conn_id);
    print_bytes("app_data_store_service_read_write_item: ",
                app_data_store_service_read_write_item,
                (int)tempLen);

    CY_LOGD(TAG, "*** Notification SENT ***");
    //wiced_bt_gatt_send_notification(g_conn_id,
    wiced_bt_gatt_server_send_notification(g_conn_id,
                                    HDLC_DATA_STORE_SERVICE_READ_WRITE_ITEM_VALUE,
                                    tempLen,
                                    app_data_store_service_read_write_item,
                                    NULL);
  }
  else /* Notification not sent */
  {
    CY_LOGE(TAG, "Notification not sent");
  }
}
#endif


/*-- Public Functions -------------------------------------------------*/

//void data_store_task(void *arg)
void data_store_task(cy_thread_arg_t arg)
{
#if (FEATURE_BLE_APP == ENABLE_FEATURE)

  while (true) {
    /* Notification values received from other tasks */
    uint32_t ulNotifiedValue;

    /* Wait for a notification */
    while (!xTaskNotifyWait( 0x00,              /* Don't clear any notification bits on entry. */
                             UINT32_MAX,        /* Reset the notification value to 0 on exit. */
                             &ulNotifiedValue,  /* Notified value pass out in
                                                   ulNotifiedValue. */
                             portMAX_DELAY ));  /* Block indefinitely. */

    if (NOTIF_RESTART_BT_ADVERT == ulNotifiedValue)
    {
      CY_LOGD(TAG, "NOTIF_RESTART_BT_ADVERT\n");
      if (wiced_bt_ble_get_current_advert_mode() == BTM_BLE_ADVERT_OFF) {
        cy_rslt_t result;
        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

        if (WICED_SUCCESS != result) {
          CY_LOGD(TAG, "Failed to start ADV");
        }
      }
    }
    else if(NOTIF_GATT_DB_RW_ITEM == ulNotifiedValue) {
      uint8_t operation;
      uint8_t dsId;

      /* Copy the RW Item value from GATT DB */
      gatt_db_lookup_table_t *puAttribute;
      CY_LOGD(TAG, "NOTIF_GATT_DB_RW_ITEM");

      puAttribute = app_get_attribute(HDLC_DATA_STORE_SERVICE_READ_WRITE_ITEM_VALUE);
      DEBUG_ASSERT(puAttribute->cur_len >= 2);

      print_bytes("puAttribute->p_data: ", puAttribute->p_data, puAttribute->cur_len);

      operation = puAttribute->p_data[0];
      dsId = puAttribute->p_data[1];

      CY_LOGD(TAG, "dsId = 0x%02x", dsId);

      if (operation == DS_OP_WRITE_REQUEST) {
        uint8_t chunkLen;
        uint8_t* chunkData_p;

        CY_LOGD(TAG, "operation = 0x%02x (WRITE_REQUEST)", operation);
        DEBUG_ASSERT(puAttribute->cur_len > DS_WRITE_REQUEST_MIN_PACKET_LEN);

        chunkLen = puAttribute->p_data[2];
        chunkData_p = &puAttribute->p_data[3];

        CY_LOGD(TAG, "chunkLen = 0x%02x", chunkLen);
        print_bytes("chunkData_p: ", chunkData_p, chunkLen);

        DEBUG_ASSERT(chunkLen == (puAttribute->cur_len - DS_WRITE_REQUEST_MIN_PACKET_LEN));

        // write the packet
        bool tempResult;
        tempResult = ds_write_item( dsId,
                                    chunkLen,
                                    chunkData_p);

        if (!tempResult) {
          CY_LOGE(TAG, "%s [%d] ds_write_item failed",
              __FUNCTION__, __LINE__);
        }

        // return the write response
        uint8_t response = tempResult? DS_OP_SUCCESS : DS_OP_FAILURE;
        update_gatt_db_rw_item(DS_OP_WRITE_RESPONSE,
                               dsId,
                               sizeof(response),
                               &response);
      }
      else if (operation == DS_OP_READ_REQUEST) {
        DEBUG_ASSERT(puAttribute->cur_len == DS_READ_REQUEST_PACKET_LEN);

        // read
        uint8_t dsData[DS_MAX_DATA_LEN];
        uint8_t dsDataLen = sizeof(dsData);
        bool tempResult;

        CY_LOGD(TAG, "operation = 0x%02x (READ_REQUEST)", operation);
        tempResult = ds_read_item(dsId,
                                  &dsDataLen,
                                  dsData);

        if (tempResult) {
          print_bytes("dsData: ", dsData, dsDataLen);
        }
        else {
          CY_LOGE(TAG, "%s [%d] ds_read_item failed",
              __FUNCTION__, __LINE__);
          dsDataLen = 0;
        }

        // return the read response
        update_gatt_db_rw_item(DS_OP_READ_RESPONSE,
                               dsId,
                               dsDataLen,
                               dsData);
      }
      else {
        CY_LOGE(TAG, "Unknown operation = 0x%02x", operation);
      }
    }
  }

#endif

  while (1);
}
