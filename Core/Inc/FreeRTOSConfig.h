/*
 * FreeRTOSConfig.h
 *
 *  Created on: Aug 1, 2026
 *      Author: vietht-hl
 */

/*
 * FreeRTOSConfig.h
 */

#ifndef INC_FREERTOSCONFIG_H_
#define INC_FREERTOSCONFIG_H_

#include "stm32f446xx.h"

/* ================= Scheduler ================= */

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

/* ================= Tick ================= */

#define configCPU_CLOCK_HZ                      (SystemCoreClock)

#define configTICK_RATE_HZ                      ((TickType_t)1000U)

#define configUSE_16_BIT_TICKS                  0

/* ================= Task ================= */

#define configMAX_PRIORITIES                    7U
#define configMINIMAL_STACK_SIZE                ((uint16_t)128U)

/* ================= Dynamic memory ================= */

#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configTOTAL_HEAP_SIZE                   ((size_t)(32U * 1024U))

/* ================= Interrupt priority ================= */

#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5U

#define configMAX_SYSCALL_INTERRUPT_PRIORITY          \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY     \
     << (8U - __NVIC_PRIO_BITS))

/* ================= Debug assertion ================= */

#define configASSERT(condition)                       \
    do                                                \
    {                                                 \
        if ((condition) == 0)                         \
        {                                             \
            __disable_irq();                          \
            for (;;)                                  \
            {                                         \
            }                                         \
        }                                             \
    } while (0)

/* ================= Cortex-M handlers ================= */

#define vPortSVCHandler                       SVC_Handler
#define xPortPendSVHandler                    PendSV_Handler
#define xPortSysTickHandler                   SysTick_Handler
#define INCLUDE_vTaskDelete    1
#define INCLUDE_vTaskDelay 1
#endif /* INC_FREERTOSCONFIG_H_ */
