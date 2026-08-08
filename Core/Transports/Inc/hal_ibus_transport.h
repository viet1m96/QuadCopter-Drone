/*
 * hal_ibus_transport.h
 *
 *  Created on: Aug 5, 2026
 *      Author: vietht-hl
 */

#ifndef TRANSPORTS_INC_HAL_IBUS_TRANSPORT_H_
#define TRANSPORTS_INC_HAL_IBUS_TRANSPORT_H_

#include "ibus.h"
#include "stm32f4xx_hal.h"

typedef enum {
  HAL_IBUS_TRANSPORT_OK = 0,
  HAL_IBUS_TRANSPORT_ERR_NULL,
  HAL_IBUS_TRANSPORT_ERR_CONFIG,
  HAL_IBUS_TRANSPORT_ERR_HAL
} HAL_IBUS_TransportStatus_t;

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t rx_buffer[IBUS_FRAME_SIZE];
} HAL_IBUS_Transport_t;

HAL_IBUS_TransportStatus_t
HAL_IBUS_TransportInit(HAL_IBUS_Transport_t *transport,
                       UART_HandleTypeDef *huart);
HAL_IBUS_TransportStatus_t
HAL_IBUS_TransportStart(HAL_IBUS_Transport_t *transport);
HAL_IBUS_TransportStatus_t
HAL_IBUS_TransportStop(HAL_IBUS_Transport_t *transport);

#endif /* TRANSPORTS_INC_HAL_IBUS_TRANSPORT_H_ */
