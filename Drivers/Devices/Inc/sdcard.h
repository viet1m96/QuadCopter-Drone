/*
 * sdcard.h
 *
 * Blocking SDHC/SDXC driver using SPI mode.
 */

#ifndef DEVICES_INC_SDCARD_H_
#define DEVICES_INC_SDCARD_H_

#include <stdint.h>

#define SD_SECTOR_SIZE 512U

typedef enum {
  SD_OK = 0,
  SD_ERR_NULL,
  SD_ERR_NOT_INITIALIZED,
  SD_ERR_TIMEOUT,
  SD_ERR_RESPONSE,
  SD_ERR_UNSUPPORTED_CARD,
  SD_ERR_TOKEN,
  SD_ERR_WRITE
} SD_Status_t;

typedef enum { SD_CARDTYPE_UNKNOWN = 0, SD_CARDTYPE_SDHC_SDXC } SD_CardType_t;

SD_Status_t SD_Init(void);

SD_Status_t SD_ReadSector(uint32_t sector, uint8_t *buffer);

SD_Status_t SD_WriteSector(uint32_t sector, const uint8_t *buffer);

SD_CardType_t SD_GetCardType(void);

#endif /* DEVICES_INC_SDCARD_H_ */
