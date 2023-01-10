/******************************************************************************
* File Name:   atmodem_config.h
*
* Description: This file contains the configuration macros required for the
*              AT modem.
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

#ifndef SOURCE_ATMODEM_CONFIG_H_
#define SOURCE_ATMODEM_CONFIG_H_

#include "cy_atmodem_hw.h"
#include "cycfg_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* modem model */
//#define ATMODEM_HW                ATMODEM_HW_MURATA_1SC
//#define ATMODEM_HW                ATMODEM_HW_SIMCOM_7600G
//#define ATMODEM_HW                ATMODEM_HW_QUECTEL_BG96
//#define ATMODEM_HW                ATMODEM_HW_SIMCOM_A7670E
//#define ATMODEM_HW                ATMODEM_HW_UBLOX_LARA_R280
#define ATMODEM_HW                ATMODEM_HW_UBLOX_SARA_U201
//#define ATMODEM_HW                ATMODEM_HW_SIMCOM_7000G
//#define ATMODEM_HW                ATMODEM_HW_UBLOX_SARA_R412M
//#define ATMODEM_HW                ATMODEM_HW_CINTERION_EXS62W

/* hardware pins */
#if defined (TARGET_APP_CY8CEVAL_062S2_LAI_4373M2) // CY8CEVAL Eval Kit

#define ATMODEM_HW_PIN_UART_RX      CYBSP_MIKROBUS_UART_RX

#define ATMODEM_HW_PIN_UART_TX      CYBSP_MIKROBUS_UART_TX

#if (ATMODEM_HW == ATMODEM_HW_CINTERION_EXS62W)
#define ATMODEM_HW_PIN_UART_RTS     CYBSP_MIKROBUS_INT
#else
#define ATMODEM_HW_PIN_UART_RTS     CYBSP_MIKROBUS_SPI_CS
#endif

#if (ATMODEM_HW == ATMODEM_HW_UBLOX_SARA_R412M)
#define ATMODEM_HW_PIN_POWER_KEY    CYBSP_MIKROBUS_AN
#else
#define ATMODEM_HW_PIN_POWER_KEY    CYBSP_MIKROBUS_RST
#endif

#if (ATMODEM_HW == ATMODEM_HW_MURATA_1SC)
#define ATMODEM_HW_PIN_IO_REF       CYBSP_MIKROBUS_PWM  // ATMODEM_HW_MURATA_1SC on CY8CEVAL Kit
#endif

#elif defined (TARGET_APP_CY8CKIT_062S2_43012) // 62S2 Pioneer Kit

#define ATMODEM_HW_PIN_UART_RX      (P13_4)

#define ATMODEM_HW_PIN_UART_TX      (P13_5)

#undef ATMODEM_HW_PIN_UART_RTS     // not needed (connect to GND)

#define ATMODEM_HW_PIN_POWER_KEY    (P8_0)

#if (ATMODEM_HW == ATMODEM_HW_MURATA_1SC)
#undef ATMODEM_HW_PIN_IO_REF       // not needed ? REVISIT
#endif

#elif defined (TARGET_APP_CY8CPROTO_062_4343W) // WIFI-BT Prototyping Kit

#define ATMODEM_HW_PIN_UART_RX      (P5_4)

#define ATMODEM_HW_PIN_UART_TX      (P5_5)

#undef ATMODEM_HW_PIN_UART_RTS     // not needed (connect to GND)

#define ATMODEM_HW_PIN_POWER_KEY    (P5_7)

#if (ATMODEM_HW == ATMODEM_HW_MURATA_1SC)
#define ATMODEM_HW_PIN_IO_REF       P5_6  // ATMODEM_HW_MURATA_1SC on WIFI-BT Prototyping Kit
#endif

#else
#pragma GCC error "Unsupported TARGET board: " __FILE__
#endif


#ifdef __cplusplus
}
#endif

#endif /* SOURCE_ATMODEM_CONFIG_H_ */

/* [] END OF FILE */
