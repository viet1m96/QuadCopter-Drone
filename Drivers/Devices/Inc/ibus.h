/*
 * ibus.h
 *
 *  Created on: Jul 26, 2026
 *      Author: vietht-hl
 */

#ifndef DEVICES_INC_IBUS_H_
#define DEVICES_INC_IBUS_H_

#include "stddef.h"
#include "stdint.h"

#define IBUS_FRAME_SIZE 32U
#define IBUS_CHANNEL_COUNT 14U

#define IBUS_FRAME_LENGTH_VALUE 0x20U
#define IBUS_COMMAND_CHANNEL_DATA 0x40U
#define IBUS_TIMEOUT_MS 100U

typedef enum {
  IBUS_OK = 0,
  IBUS_ERR_NULL,
  IBUS_ERR_FRAME_SIZE,
  IBUS_ERR_FRAME_HEADER,
  IBUS_ERR_CHECKSUM
} IBUS_Status_t;

typedef struct {
  uint8_t bytes[IBUS_FRAME_SIZE];
} IBUS_RawFrame_t;

typedef struct {
  uint16_t channels[IBUS_CHANNEL_COUNT];
} IBUS_Data_t;

IBUS_Status_t IBUS_DecodeFrame(const uint8_t *raw_frame, size_t frame_size,
                               IBUS_Data_t *decoded_frame);

#endif /* DEVICES_INC_IBUS_H_ */
