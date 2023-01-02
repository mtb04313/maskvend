/******************************************************************************
* File Name:   time_config.h
*
* Description: This file has timestamp related configuration
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

/*******************************************************************************
 *  Include guard
 ******************************************************************************/
#ifndef SOURCE_TIME_CONFIG_H_
#define SOURCE_TIME_CONFIG_H_

#include <stdint.h>
#include <cmsis_gcc.h>  // for __PACKED_STRUCT

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Macros
 ******************************************************************************/

/* Configure the timestamp */
#define DEFAULT_TIMESTAMP_SEC      -461020808L // must be int32_t

#define MIN_TIMESTAMP_SEC          -461020808L // 29-Jun 2021 9:08am GMT or 17:08 SGP
#define MAX_TIMESTAMP_SEC          (MIN_TIMESTAMP_SEC + 3600*24*365*10) // +10 years

/* Configure the timezone */
#define DEFAULT_TIMEZONE_DIFF       +8
#define MIN_TIMEZONE_DIFF           -12
#define MAX_TIMEZONE_DIFF           +14

/*******************************************************************************
 * Structures
 ******************************************************************************/

/* Structure to store time related info that goes into EEPROM */
typedef __PACKED_STRUCT
{
    int32_t timestamp;    // e.g. -461020808L
    float timezone_diff;  // e.g. +5.5
} cy_time_info_t;

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_TIME_CONFIG_H_ */

/* [] END OF FILE */
