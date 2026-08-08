/*
 * retarget.c
 *
 *  Created on: Jun 25, 2026
 *      Author: vietht-hl
 */

#include "main.h"
#include "stdio.h"
#include "unistd.h"

extern UART_HandleTypeDef husart2;

int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&husart2, (uint8_t *)ptr, len, 100);
  return len;
}
