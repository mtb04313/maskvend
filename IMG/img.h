/******************************************************************************
* File Name:   img.h
*
* Description: This file declares the image resources in source files
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

#ifndef SOURCE_IMG_H_
#define SOURCE_IMG_H_

#include "feature_config.h"

#include <stdlib.h>
#include "GUI.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

extern GUI_CONST_STORAGE GUI_BITMAP bmmanword2;

extern GUI_CONST_STORAGE GUI_BITMAP bmS_green_tick;
extern GUI_CONST_STORAGE GUI_BITMAP bmS_no_tick;

extern GUI_CONST_STORAGE GUI_BITMAP bmdark_blue_mask_3d;
extern GUI_CONST_STORAGE GUI_BITMAP bmdark_green_mask_3d;
extern GUI_CONST_STORAGE GUI_BITMAP bmdark_grey_mask_3d;
extern GUI_CONST_STORAGE GUI_BITMAP bmdark_red_mask_3d;
extern GUI_CONST_STORAGE GUI_BITMAP bmdark_violet_mask_3d;
extern GUI_CONST_STORAGE GUI_BITMAP bmdark_pink_mask_3d;

extern GUI_CONST_STORAGE GUI_BITMAP bmmask_kiosk;

#if (FEATURE_GUI_MARKETING_SCREEN == ENABLE_FEATURE)
extern GUI_CONST_STORAGE GUI_BITMAP bmchip2;
extern GUI_CONST_STORAGE GUI_BITMAP bmmanword2;
#endif

#ifdef __cplusplus
}
#endif

#endif // SOURCE_IMG_H_

/* [] END OF FILE */
