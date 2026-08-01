/*
 * FreeRTOSConfig.h
 *
 *  Created on: Aug 1, 2026
 *      Author: vietht-hl
 */

#ifndef INC_FREERTOSCONFIG_H_
#define INC_FREERTOSCONFIG_H_

#include "stm32f446xx.h"

//scheduler
#define configUSE_PREEMPTION 1


//clock and tick
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000)



#endif /* INC_FREERTOSCONFIG_H_ */
