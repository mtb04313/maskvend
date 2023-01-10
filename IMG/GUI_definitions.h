/******************************************************************************
* File Name:   GUI_definitions.h
*
* Description: This file defines screen parameters
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

#ifndef SOURCE_GUI_DEFINITIONS_H_
#define SOURCE_GUI_DEFINITIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*-- Public Definitions -------------------------------------------------*/

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
#define SCREEN_CENTER_X_COORD       159
#define SCREEN_CENTER_Y_COORD       119

#define TEXT_BOX_X1_COORD           0
#define TEXT_BOX_Y1_COORD           89
#define TEXT_BOX_X2_COORD           319
#define TEXT_BOX_Y2_COORD           149

#define MASK_DISPENSED_MSG_X_COORD  210
#define MASK_DISPENSED_MSG_Y_COORD  100
#define OUT_OF_STOCK_MSG_X_COORD    210
#define OUT_OF_STOCK_MSG_Y_COORD    100

#define SCREEN_X_COORD_0 0
#define SCREEN_Y_COORD_0 0
#define SCREEN_X_COORD_319 319
#define SCREEN_Y_COORD_239 239

#ifdef __cplusplus
}
#endif

#endif // SOURCE_GUI_DEFINITIONS_H_
