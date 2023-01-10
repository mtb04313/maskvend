/******************************************************************************
* File Name:   data_store_task.h
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

#ifndef SOURCE_DATA_STORE_TASK_H_
#define SOURCE_DATA_STORE_TASK_H_

#include "feature_config.h"
#include "cyabs_rtos.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
* Macros
********************************************************************************/

#define DATA_STORE_TASK_NAME        "data store task"
#define DATA_STORE_TASK_PRIORITY    CY_RTOS_PRIORITY_LOW
#define DATA_STORE_TASK_STACK_SIZE  (2 * 1024)

#if (FEATURE_DATA_STORE == ENABLE_FEATURE)
extern cy_thread_t g_data_store_task_handle;
#endif

/*******************************************************************************
* Function Prototype
********************************************************************************/
void data_store_task(cy_thread_arg_t arg);

#ifdef __cplusplus
}
#endif

#endif /* SOURCE_DATA_STORE_TASK_H_ */
