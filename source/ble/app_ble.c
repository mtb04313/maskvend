/******************************************************************************
* File Name:   app_ble.c
*
* Description: This file implements the BLE application
*
* Related Document: See Readme.md
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include <FreeRTOS.h>
#include <task.h>
#include <inttypes.h>

#include "cybsp.h"

#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"

#include "wiced_bt_stack.h"
#include "cycfg_gatt_db.h"
#include "wiced_memory.h"

#include "app_ble.h"
#include "app_utils.h"

#include "common_task.h"
#include "data_store_task.h"

#include "cy_debug.h"
#include "cy_string.h"
#include "kiosk_config.h"
#include "flash_eeprom.h"

#include "user_button_config.h"


/*-- Public Data -------------------------------------------------*/

/* Maintains the connection id of the current connection */
uint16_t g_conn_id = 0;


/*-- Local Definitions -------------------------------------------------*/

#if (FEATURE_BLE_APP == ENABLE_FEATURE)

/* Priority for the button interrupt */
#define GPIO_INTERRUPT_PRIORITY             (7)

/* Number of Advertisement elements */
#define ADV_ELEMENTS 3

/* LE Key Size */
#define MAX_KEY_SIZE (0x10)

/* WICED Heap size for app */
#define APP_HEAP_SIZE 0x1000

/******************************************************************************
 *                                 TYPEDEFS
 ******************************************************************************/
typedef void (*pfn_free_buffer_t)(uint8_t *);


/*-- Local Function Prototypes -------------------------------------------------*/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);


/*-- Local Data -------------------------------------------------*/

/* Structure that stores the callback data for the GPIO interrupt event. */
static cyhal_gpio_callback_data_t s_cb_data =
{
    .callback = gpio_interrupt_handler,
    .callback_arg = NULL
};

/* Pointer to the heap created */
static wiced_bt_heap_t * s_app_heap_pointer = NULL;

static const char *TAG = "app_ble";


/*-- Local Functions -------------------------------------------------*/

/*******************************************************************************
* Function Name: app_set_advertisement_data
********************************************************************************
* Summary:
* This function sets the advertising data.
*
*******************************************************************************/
static void app_set_advertisement_data(void)
{
  wiced_result_t result;

  wiced_bt_ble_advert_elem_t adv_elem[ADV_ELEMENTS];
  uint8_t num_elem = 0;
  uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
  uint8_t custom_service_uuid[LEN_UUID_128] = {__UUID_SERVICE_DATA_STORE_SERVICE};

  /* First element is the advertisment flags */
  adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
  adv_elem[num_elem].len          = sizeof(uint8_t);
  adv_elem[num_elem].p_data       = &flag;
  num_elem++;

  /* Second element is the 128 bit custom UUID */
  adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
  adv_elem[num_elem].len          = LEN_UUID_128;
  adv_elem[num_elem].p_data       = custom_service_uuid;
  num_elem++;

  /* Replace the last 3 digits of device_name with actual kiosk id */
  int16_t id = DEFAULT_KIOSK_ID;

#if (HAVE_FLASH_EEPROM == 1)
  if (!flash_eeprom_get_kiosk_id(&id, sizeof(id))) {
    DEBUG_PRINT(("flash_eeprom_get_kiosk_id failed\n"));
  }
#endif

  char id_str[8] = "";
  SNPRINTF(id_str, sizeof(id_str), "%03d", id);
  int id_len = strlen(id_str);

  VoidAssert(id_len == 3);
  VoidAssert(app_gap_device_name_len > id_len);
  memcpy(&app_gap_device_name[app_gap_device_name_len - id_len], id_str, id_len);

  /* Final element is the device name */
  adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
  adv_elem[num_elem].len          = app_gap_device_name_len;
  adv_elem[num_elem].p_data       = (uint8_t *)app_gap_device_name;
  num_elem++;

  result = wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
  if (WICED_SUCCESS != result)
  {
    CY_LOGD(TAG, "Set ADV data failed");
  }
}


/*******************************************************************************
* Function Name: app_gatt_connect_handler
********************************************************************************
* Summary:
* This function handles the GATT connect request events from the stack
*
* Parameters:
*  wiced_bt_gatt_connection_status_t *p_conn_status: Connection or disconnection
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatt_connect_handler(
                       wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t result;

    /* Check whether it is a connect event or disconnect event. If the device
    has been disconnected then restart advertisement */
    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
            /* Device got connected */
            printf("\nConnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");
            g_conn_id = p_conn_status->conn_id;
        }
        else /* Device got disconnected */
        {
            printf("\nDisconnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");

            printf("Reason for disconnection: %s\n",
                    get_bt_gatt_disconn_reason_name(p_conn_status->reason));

            g_conn_id = 0;

            /* Set the advertising params and make the device discoverable */
            app_set_advertisement_data();

            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                   //BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                                   0,
                                                   NULL);
            if(WICED_SUCCESS != result)
            {
                printf("Start ADV failed");
            }
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}

/*******************************************************************************
* Function Name: app_gatts_req_read_handler
********************************************************************************
* Summary:
* This function handles the GATT read request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_read_t * p_read_data: Read data structure
*  wiced_bt_gatt_opcode_t opcode: GATT opcode
*  uint16_t len_requested: Length requested
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_read_handler(uint16_t conn_id,
                                                  wiced_bt_gatt_opcode_t opcode,
                                                  wiced_bt_gatt_read_t *p_read_data,
                                                  uint16_t len_requested)
{

    gatt_db_lookup_table_t *puAttribute;
    int attr_len_to_copy;

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = app_get_attribute(p_read_data->handle)))
    {
        printf("Read handle attribute not found. Handle:0x%X\n",
                p_read_data->handle);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    printf("GATT Read handler: handle:0x%X, len:%d\n",
           p_read_data->handle, attr_len_to_copy);

    /* If the incoming offset is greater than the current length in the GATT DB
    then the data cannot be read back*/
    if (p_read_data->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return (WICED_BT_GATT_INVALID_OFFSET);
    }

    int to_send = MIN(len_requested, attr_len_to_copy - p_read_data->offset);

    uint8_t *from = ((uint8_t *)puAttribute->p_data) + p_read_data->offset;

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: app_gatt_read_by_type_handler
********************************************************************************
* Summary:
* This function handles the GATT read by type request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_opcode_t opcode: GATT opcode
*  wiced_bt_gatt_read_by_type_t * p_read_data: Read data structure
*  uint16_t len_requested: Length requested
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatt_read_by_type_handler(uint16_t conn_id,
                                         wiced_bt_gatt_opcode_t opcode,
                                         wiced_bt_gatt_read_by_type_t *p_read_data,
                                         uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t    attr_handle = p_read_data->s_handle;
    uint8_t     *p_rsp = wiced_bt_get_buffer(len_requested);
    uint8_t     pair_len = 0;
    int used = 0;

    if (p_rsp == NULL)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (WICED_TRUE)
    {
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                p_read_data->e_handle, &p_read_data->uuid);

        if (attr_handle == 0)
            break;

        if ((puAttribute = app_get_attribute(attr_handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode,
                    p_read_data->s_handle, WICED_BT_GATT_ERR_UNLIKELY);
            wiced_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_ERR_UNLIKELY;
        }

        int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(
                p_rsp + used, len_requested - used, &pair_len, attr_handle,
                 puAttribute->cur_len, puAttribute->p_data);
        if (filled == 0)
        {
            break;
        }
        used += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_data->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        wiced_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len,
            used, p_rsp, (wiced_bt_gatt_app_context_t)wiced_bt_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
* Function Name: app_gatts_req_write_handler
********************************************************************************
* Summary:
* This function handles the GATT write request events from the stack
*
* Parameters:
*  uint16_t conn_id: Connection ID
*  wiced_bt_gatt_opcode_t opcode: GATT opcode
*  wiced_bt_gatt_write_t * p_data: Write data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_write_handler(uint16_t conn_id,
                                                   wiced_bt_gatt_opcode_t opcode,
                                                   wiced_bt_gatt_write_req_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    uint8_t *p_attr = p_data->p_val;
    gatt_db_lookup_table_t *puAttribute;

    /* Return status for EEPROM. */
    //cy_en_em_eeprom_status_t eepromReturnValue;

    /* Variables for TLV data extraction */
    //uint8_t byte_no = 0;
    //uint8_t data_length = 0;

    printf("GATT write handler: handle:0x%X len:%d, opcode:0x%X\n",
           p_data->handle, p_data->val_len, opcode);

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = app_get_attribute(p_data->handle)))
    {
        printf("\nWrite Handle attr not found. Handle:0x%X\n", p_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }


    switch (p_data->handle)
    {
    /* Write request for the Read Write Item characteristic. */
    case HDLC_DATA_STORE_SERVICE_READ_WRITE_ITEM_VALUE:
        memset(app_data_store_service_read_write_item, 0, app_data_store_service_read_write_item_len);
        DEBUG_ASSERT(p_data->val_len <= app_data_store_service_read_write_item_len);

        memcpy(app_data_store_service_read_write_item, p_attr, p_data->val_len);
        puAttribute->cur_len = p_data->val_len;
        printf("RW Item: 0x%02x\n", app_data_store_service_read_write_item[0]);

        xTaskNotify(g_data_store_task_handle, NOTIF_GATT_DB_RW_ITEM, eSetValueWithOverwrite);
        break;

    /* Notification for Read Write Item characteristic. If enabled, notification can
        * be sent to the client when the operation is completed
        */
    case HDLD_DATA_STORE_SERVICE_READ_WRITE_ITEM_CLIENT_CHAR_CONFIG:
        app_data_store_service_read_write_item_client_char_config[0] = p_attr[0];
        app_data_store_service_read_write_item_client_char_config[1] = p_attr[1];
        printf("RW Item (Notify): 0x%02x\n", app_data_store_service_read_write_item_client_char_config[0]);
        break;

    default:
        printf("Write GATT Handle not found\n");
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_gatts_attr_req_handler
********************************************************************************
* Summary:
* This function redirects the GATT attribute requests to the appropriate
* functions
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
*
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_attr_req_handler(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;
    wiced_bt_gatt_write_req_t *p_write_request = &p_data->data.write_req;

    switch (p_data->opcode)
    {
    case GATT_REQ_READ:
    case GATT_REQ_READ_BLOB:
        result = app_gatts_req_read_handler(p_data->conn_id, p_data->opcode,
                                &p_data->data.read_req, p_data->len_requested);
        break;

    case GATT_REQ_READ_BY_TYPE:
        result = app_gatt_read_by_type_handler(p_data->conn_id, p_data->opcode,
                            &p_data->data.read_by_type, p_data->len_requested);
        break;

    case GATT_REQ_WRITE:
    case GATT_CMD_WRITE:
    case GATT_CMD_SIGNED_WRITE:
        result = app_gatts_req_write_handler(p_data->conn_id, p_data->opcode,
                                             &(p_data->data.write_req));
        if ((p_data->opcode == GATT_REQ_WRITE) && (result == WICED_BT_GATT_SUCCESS))
        {
            wiced_bt_gatt_server_send_write_rsp(p_data->conn_id, p_data->opcode,
                                                p_write_request->handle);
        }
        else
        {
            wiced_bt_gatt_server_send_error_rsp(p_data->conn_id, p_data->opcode,
                                                p_write_request->handle, result);
        }
        break;

    case GATT_REQ_MTU:
        printf("Exchanged MTU from client: %d\n", p_data->data.remote_mtu);
        wiced_bt_gatt_server_send_mtu_rsp(p_data->conn_id, p_data->data.remote_mtu,
                          wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
        result = WICED_BT_GATT_SUCCESS;
        break;

    default:
        break;
    }

    return result;
}

/*******************************************************************************
* Function Name: app_gatts_callback
********************************************************************************
* Summary:
* This function redirects the GATT requests to the appropriate functions
*
* Parameters:
*  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
*  wiced_bt_gatt_event_data_t *p_data       : Pointer to BLE GATT event structures
* Return:
*  wiced_bt_gatt_status_t: GATT result
*
*******************************************************************************/
wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event,
                                          wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    printf("GATTS event: %s\n", get_bt_gatt_evt_name(event));

    switch (event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = app_gatt_connect_handler(&p_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = app_gatts_attr_req_handler(&p_data->attribute_request);
        break;

    case GATT_GET_RESPONSE_BUFFER_EVT:
        p_data->buffer_request.buffer.p_app_rsp_buffer = wiced_bt_get_buffer(
                                          p_data->buffer_request.len_requested);

        p_data->buffer_request.buffer.p_app_ctxt = (void *)wiced_bt_free_buffer;

        if(NULL == p_data->buffer_request.buffer.p_app_rsp_buffer)
        {
            printf("Insufficient resources\n");
            result = WICED_BT_GATT_INSUF_RESOURCE;
        }
        else
        {
            result = WICED_BT_GATT_SUCCESS;
        }
        break;

    case GATT_APP_BUFFER_TRANSMITTED_EVT:
    {
        pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->
                                      buffer_xmitted.p_app_ctxt;

        /* If the buffer is dynamic, the context will point to a function to
         * free it.
         */
        if (pfn_free)
            pfn_free(p_data->buffer_xmitted.p_app_data);

        result = WICED_BT_GATT_SUCCESS;
    }
    break;

    default:
        printf("GATT event not handled\n");
    }
    return result;
}

/*******************************************************************************
* Function Name: application_init
********************************************************************************
* Summary:
* This function is called from the BTM enabled event
*    1. Initializes and registers the GATT DB
*    2. Sets pairable mode to true
*    3. Sets ADV data and starts advertising
*
*******************************************************************************/
void application_init(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_CONNECTION_ID;

    /* Return status for EEPROM */
    //cy_en_em_eeprom_status_t eepromReturnValue;

    /* Create a buffer heap, make it the default heap */
    s_app_heap_pointer = wiced_bt_create_heap("app", NULL, APP_HEAP_SIZE, NULL,
                                            WICED_TRUE);

    if (NULL == s_app_heap_pointer)
    {
        printf("failed to create heap\n");
        CY_ASSERT(0);
    }

    /* Enabling button interrupt */
    cyhal_gpio_enable_event(USER_BUTTON_TURN_ON_BLE_ADVERTISEMENT, CYHAL_GPIO_IRQ_FALL,
                            GPIO_INTERRUPT_PRIORITY, true);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_gatts_callback);

    if(WICED_BT_GATT_SUCCESS !=gatt_status)
    {
        printf("\nGATT register failed. Status: %s\n", get_bt_gatt_status_name(
                                                       gatt_status));
    }

    /*  Inform the stack to use our GATT database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);

    if(WICED_BT_GATT_SUCCESS !=gatt_status)
    {
        printf("\nGATT db init failed. Status :%s\n",get_bt_gatt_status_name(
                                                     gatt_status));
    }

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, false);

    /* Set the advertising params and make the device discoverable */
    app_set_advertisement_data();

    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                           //BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                           BLE_ADDR_PUBLIC, NULL);
    if (WICED_SUCCESS != result)
    {
        printf("Start ADV failed");
    }
}

/*******************************************************************************
* Function Name: app_management_callback
********************************************************************************
* Summary:
* This function handles the BT stack events.
*
* Parameters:
*  wiced_bt_management_evt_t: event code
*  p_event_data: Pointer to the event data
*
* Return:
*  wiced_result_t: Result
*
*******************************************************************************/
wiced_result_t app_management_callback(wiced_bt_management_evt_t event,
                                       wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = {0};
    wiced_bt_dev_ble_io_caps_req_t *pairing_io_caps = &(p_event_data->
                                           pairing_io_capabilities_ble_request);

    printf("Bluetooth Management Event: %s\n", get_bt_event_name(event));

    switch (event)
    {
    case BTM_ENABLED_EVT:

        /* Bluetooth is enabled */
        wiced_bt_set_local_bdaddr((uint8_t *)cy_bt_device_address,
                                   BLE_ADDR_PUBLIC);

        /* Read and print the BD address */
        wiced_bt_dev_read_local_addr(bda);
        printf("Local Bluetooth Address: ");
        print_bd_address(bda);
        printf("\n");

        application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

        /* Print passkey to the screen so that the user can enter it. */
    case BTM_PASSKEY_NOTIFICATION_EVT:
        printf( "********************************************************\r\n");
        printf( "Passkey Notification\r\n");
        printf("PassKey: %" PRIu32 "\r\n",
        p_event_data->user_passkey_notification.passkey );
        printf( "***********************************************************\r\n");
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        pairing_io_caps->local_io_cap = BTM_IO_CAPABILITIES_NONE; //BTM_IO_CAPABILITIES_DISPLAY_ONLY;

        pairing_io_caps->oob_data = BTM_OOB_NONE;

        pairing_io_caps->auth_req = BTM_LE_AUTH_REQ_SC;

        pairing_io_caps->max_key_size = MAX_KEY_SIZE;

        pairing_io_caps->init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID |
                                    BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;

        pairing_io_caps->resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID |
                                    BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        if (WICED_SUCCESS == p_event_data->
                            pairing_complete.pairing_complete_info.ble.status)
        {
            printf("Pairing Complete: SUCCESS\n");
        }
        else /* Pairing Failed */
        {
            printf("Pairing Complete: FAILED\n");
        }
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Paired Device Link Keys update */
        result = WICED_SUCCESS;
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        result = WICED_BT_ERROR;
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Local identity Keys Update */
        result = WICED_SUCCESS;
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* Local identity Keys Request */
        result = WICED_BT_ERROR;
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        if (WICED_SUCCESS == p_event_data->encryption_status.result)
        {
            printf("Encryption Status Event: SUCCESS\n");
        }
        else /* Encryption Failed */
        {
            printf("Encryption Status Event: FAILED\n");
        }
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                    WICED_BT_SUCCESS);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        printf("\n");
        printf("Advertisement state changed to %s\n", get_bt_advert_mode_name(
                                       p_event_data->ble_advert_state_changed));
        break;

    default:
        break;
    }

    return result;
}

/*******************************************************************************
 * Function Name: gpio_interrupt_handler
 *******************************************************************************/
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
   /* Notify the WiFi task to restart BLE advertisement */
   xTaskNotifyFromISR( g_data_store_task_handle,
                       NOTIF_RESTART_BT_ADVERT,
                       eSetValueWithOverwrite,
                       pdFALSE);
}
#endif


/*-- Public Functions -------------------------------------------------*/

bool ble_button_init(void)
{
#if (FEATURE_BLE_APP == ENABLE_FEATURE)
  /* Initialize the user button.*/
  cyhal_gpio_init(USER_BUTTON_TURN_ON_BLE_ADVERTISEMENT, CYHAL_GPIO_DIR_INPUT,
                      CYHAL_GPIO_DRIVE_NONE, CYBSP_BTN_OFF);

  /* Configure GPIO interrupt */
  // was cyhal_gpio_register_callback(USER_BUTTON_TURN_ON_BLE_ADVERTISEMENT, gpio_interrupt_handler, NULL);
  cyhal_gpio_register_callback(USER_BUTTON_TURN_ON_BLE_ADVERTISEMENT, &s_cb_data);

#endif
  return true;
}

bool ble_init(void)
{
#if (FEATURE_BLE_APP == ENABLE_FEATURE)
  /* Configure platform specific settings for Bluetooth */
  cybt_platform_config_init(&cybsp_bt_platform_cfg);

  /* Initialize the Bluetooth stack with a callback function and stack
   * configuration structure */
  if (WICED_SUCCESS != wiced_bt_stack_init(app_management_callback, &wiced_bt_cfg_settings))
  {
    //CY_LOGE(TAG, "Error initializing BT stack\n");
    //CY_ASSERT(0);
    return false;
  }

#endif
  return true;
}

/*******************************************************************************
* Function Name: app_get_attribute
********************************************************************************
* Summary:
* This function searches through the GATT DB to point to the attribute
* corresponding to the given handle
*
* Parameters:
*  uint16_t handle: Handle to search for in the GATT DB
*
* Return:
*  gatt_db_lookup_table_t *: Pointer to the correct attribute in the GATT DB
*
*******************************************************************************/
gatt_db_lookup_table_t * app_get_attribute(uint16_t handle)
{
  /* Search for the given handle in the GATT DB and return the pointer to the
  correct attribute */
  uint8_t array_index = 0;

  for (array_index = 0; array_index < app_gatt_db_ext_attr_tbl_size; array_index++)
  {
    if (app_gatt_db_ext_attr_tbl[array_index].handle == handle)
    {
      return (&app_gatt_db_ext_attr_tbl[array_index]);
    }
  }
  return NULL;
}

/* [] END OF FILE */
