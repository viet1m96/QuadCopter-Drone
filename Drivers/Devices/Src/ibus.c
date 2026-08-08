/*
 * ibus.c
 *
 *  Created on: Jul 26, 2026
 *      Author: vietht-hl
 */

#include "ibus.h"
#include "byte_utils.h"

#define IBUS_CHANNEL_DATA_OFFSET 2U
#define IBUS_CHECKSUM_DATA_SIZE 30U
#define IBUS_CHECKSUM_LOW_INDEX 30U
#define IBUS_CHECKSUM_HIGH_INDEX 31U
#define IBUS_CHANNEL_BYTE_INDEX(i) (((i) * 2U) + 2U)

static uint16_t ibus_calculate_checksum(const uint8_t *raw_frame) {
  uint16_t sum = 0U;

  for (size_t i = 0U; i < IBUS_CHECKSUM_DATA_SIZE; i++) {

    sum = (uint16_t)(sum + raw_frame[i]);
  }

  return (uint16_t)(0xFFFFU - sum);
}

static uint16_t ibus_get_received_checksum(const uint8_t *raw_frame) {
  return (uint16_t)raw_frame[IBUS_CHECKSUM_LOW_INDEX] |
         ((uint16_t)raw_frame[IBUS_CHECKSUM_HIGH_INDEX] << 8U);
}

IBUS_Status_t IBUS_DecodeFrame(const uint8_t *raw_frame, size_t frame_size,
                               IBUS_Data_t *decoded_frame) {
  if (raw_frame == NULL || decoded_frame == NULL)
    return IBUS_ERR_NULL;
  if (frame_size != IBUS_FRAME_SIZE)
    return IBUS_ERR_FRAME_SIZE;
  if ((raw_frame[0] != IBUS_FRAME_LENGTH_VALUE) ||
      raw_frame[1] != IBUS_COMMAND_CHANNEL_DATA) {
    return IBUS_ERR_FRAME_HEADER;
  }

  if (ibus_get_received_checksum(raw_frame) !=
      ibus_calculate_checksum(raw_frame)) {
    return IBUS_ERR_CHECKSUM;
  }
  for (size_t i = 0U; i < IBUS_CHANNEL_COUNT; i++) {
    uint32_t idx = IBUS_CHANNEL_BYTE_INDEX(i);
    decoded_frame->channels[i] =
        byte_utils_u16_from_be(raw_frame[idx + 1], raw_frame[idx]);
  }

  return IBUS_OK;
}
