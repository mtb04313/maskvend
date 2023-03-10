/******************************************************************************
* File Name:   mqtt_client_config.c
*
* Description: This file contains the configuration structures used by
*              the MQTT client for MQTT connect operation.
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

#include <stdio.h>
#include "feature_config.h"
#include "mqtt_client_config.h"
#include "cy_mqtt_api.h"
#include "flash_eeprom.h"
#include "cy_debug.h"
#include "cy_string.h"

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
#include "optiga_trust_helpers.h"

/* We don't use CLIENT_CERTIFICATE memory but instead allocate a buffer and
 * populate it with teh certificate form the Secure Element */
char CERTIFICATE[1200];
uint16_t CERTIFICATE_SIZE = 1200;
#endif

/******************************************************************************
* Global Variables
*******************************************************************************/
/* MQTT Broker/Server details */
cy_mqtt_broker_info_t broker_info =
{
    .hostname = MQTT_BROKER_ADDRESS,
    .hostname_len = sizeof(MQTT_BROKER_ADDRESS) - 1,
    .port = MQTT_PORT
};

#if (MQTT_SECURE_CONNECTION)
/* MQTT client credentials to be used in case of a secure connection. */
static cy_awsport_ssl_credentials_t credentials =
{
    /* Configure the client certificate. */
#ifdef CLIENT_CERTIFICATE
    .client_cert = (const char *)CLIENT_CERTIFICATE,
    .client_cert_size = sizeof(CLIENT_CERTIFICATE),
#else
    .client_cert = NULL,
    .client_cert_size = 0,
#endif

    /* Configure the client private key. */
#ifdef CLIENT_PRIVATE_KEY
    .private_key = (const char *)CLIENT_PRIVATE_KEY,
    .private_key_size = sizeof(CLIENT_PRIVATE_KEY),
#else
    .private_key = NULL,
    .private_key_size = 0,
#endif

    /* Configure the Root CA certificate of the MQTT Broker/Server. */
#ifdef ROOT_CA_CERTIFICATE
    .root_ca = (const char *)ROOT_CA_CERTIFICATE,
    .root_ca_size = sizeof(ROOT_CA_CERTIFICATE),
#else
    .root_ca = NULL,
    .root_ca_size = 0,
#endif

    /* Application Layer Protocol Negotiation (ALPN) is used to implement
     * MQTT with TLS Client Authentication from client devices.
     */
#ifdef MQTT_ALPN_PROTOCOL_NAME
    .alpnprotos = (const char *)MQTT_ALPN_PROTOCOL_NAME,
    .alpnprotoslen = sizeof(MQTT_ALPN_PROTOCOL_NAME),
#else
    .alpnprotos = NULL,
    .alpnprotoslen = 0,
#endif

    /* Server Name Indication (SNI) hostname to be sent as a part of the
     * Client Hello message sent during TLS handshake as specified by the
     * MQTT Broker.
     */
#ifdef MQTT_SNI_HOSTNAME
    .sni_host_name = (const char *)MQTT_SNI_HOSTNAME,
    .sni_host_name_size = sizeof(MQTT_SNI_HOSTNAME)
#else
    .sni_host_name = NULL,
    .sni_host_name_size = 0
#endif
};

/* Pointer to the security details of the MQTT connection. */
cy_awsport_ssl_credentials_t *security_info = &credentials;

#else
/* Pointer to the security details of the MQTT connection. */
struct cy_awsport_ssl_credentials_t *security_info = NULL;
#endif /* #if (MQTT_SECURE_CONNECTION) */

#if ENABLE_LWT_MESSAGE
static char s_mqtt_will_topic[MQTT_WILL_TOPIC_MAX_LEN] = "";

/* Last Will and Testament (LWT) message structure. The MQTT broker will
 * publish the LWT message if this client disconnects unexpectedly.
 */
static cy_mqtt_publish_info_t s_will_msg_info =
{
    .qos = CY_MQTT_QOS2,
    .topic = s_mqtt_will_topic, //MQTT_WILL_TOPIC_NAME,
    .topic_len = 0, //(uint16_t)(sizeof(MQTT_WILL_TOPIC_NAME) - 1),
    .payload = MQTT_WILL_MESSAGE,
    .payload_len = (size_t)(sizeof(MQTT_WILL_MESSAGE) - 1),
    .retain = false,
    .dup = false
};
#endif /* ENABLE_LWT_MESSAGE */

/* MQTT connection information structure */
cy_mqtt_connect_info_t g_mqtt_connection_info =
{
    .client_id = NULL,
    .client_id_len = 0,
    .username = NULL,
    .username_len = 0,
    .password = NULL,
    .password_len = 0,
    .clean_session = true,
    .keep_alive_sec = MQTT_KEEP_ALIVE_SECONDS,
#if ENABLE_LWT_MESSAGE
    .will_info = &s_will_msg_info
#else
    .will_info = NULL
#endif /* ENABLE_LWT_MESSAGE */
};

/* Check for a valid QoS setting - QoS 0, QoS 1, or QoS 2. */
#if ((MQTT_MESSAGES_QOS != 0) && (MQTT_MESSAGES_QOS != 1) && (MQTT_MESSAGES_QOS != 2))
#error "Invalid QoS setting! MQTT_MESSAGES_QOS must be either 0 or 1."
#endif


void mqtt_set_will_topic(void)
{
#if ENABLE_LWT_MESSAGE
  // initialize the topic once
  if (s_mqtt_will_topic[0] == '\0') {
    int16_t id = DEFAULT_KIOSK_ID;

    // read the kiosk id and set the topic, topic_len
    if (!flash_eeprom_get_kiosk_id(&id, sizeof(id))) {
      DEBUG_PRINT(("flash_eeprom_get_kiosk_id failed\n"));
    }

    SNPRINTF(s_mqtt_will_topic, sizeof(s_mqtt_will_topic), MQTT_WILL_TOPIC_PATTERN, id);
    s_will_msg_info.topic_len = strlen(s_mqtt_will_topic);
  }
#endif
}

#if (FEATURE_TRUST_M == ENABLE_FEATURE)
void use_optiga_certificate(void)
{
    /* This is the place where the certificate is initialized. This is a function
     * which will allow to read it out and populate internal mbedtls settings wit it*/
    read_certificate_from_optiga(0xe0e0, CERTIFICATE, &CERTIFICATE_SIZE);
    printf("Your certificate is:\n%s\n",CERTIFICATE);
#if (MQTT_SECURE_CONNECTION)
    security_info->client_cert = (const char *)CERTIFICATE;
    security_info->client_cert_size = (size_t)CERTIFICATE_SIZE;
#endif
}
#endif

/* [] END OF FILE */
