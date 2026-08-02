/*
 * FreeRTOSConfig.h
 *
 *  Created on: Aug 1, 2026
 *      Author: vietht-hl
 */

#ifndef INC_FREERTOSCONFIG_H_
#define INC_FREERTOSCONFIG_H_

#include "stm32f446xx.h"

/* scheduler */
#define configUSE_PREEMPTION 1U
#define configMAX_PRIORITIES 7U


/* clock and tick */
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000)

/* Task */
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0

/* Memory */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   ((size_t)(32U * 1024U))

/* Sync */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8

#define vPortSVCHandler       SVC_Handler
#define xPortPendSVHandler    PendSV_Handler
#define xPortSysTickHandler   SysTick_Handler


#endif /* INC_FREERTOSCONFIG_H_ */
