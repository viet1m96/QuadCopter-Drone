/*
 * sdcard.c
 *
 * SPI2 pins:
 *   PB13 - SCK  (AF5)
 *   PB14 - MISO (AF5)
 *   PB15 - MOSI (AF5)
 *   PB6  - CS   (GPIO output)
 */

#include "sdcard.h"

#include <stddef.h>

#include "peripherals.h"
#include "stm32f4xx.h"

#define SD_SPI_BYTE_TIMEOUT_US 1000U
#define SD_COMMAND_READY_TIMEOUT_US 10000U
#define SD_CMD0_TIMEOUT_US 500000U
#define SD_INIT_TIMEOUT_US 2000000U
#define SD_READ_TOKEN_TIMEOUT_US 200000U
#define SD_WRITE_BUSY_TIMEOUT_US 500000U

#define SD_CMD0 0U
#define SD_CMD8 8U
#define SD_CMD17 17U
#define SD_CMD24 24U
#define SD_CMD55 55U
#define SD_ACMD41 41U
#define SD_CMD58 58U

#define SD_R1_READY 0x00U
#define SD_R1_IDLE 0x01U
#define SD_R1_ILLEGAL_COMMAND 0x04U

#define SD_TOKEN_START_BLOCK 0xFEU
#define SD_DATA_ACCEPTED 0x05U

static uint8_t sd_initialized = 0U;
static SD_CardType_t sd_card_type = SD_CARDTYPE_UNKNOWN;

static uint8_t SD_TimeoutExpired(uint32_t start_us, uint32_t timeout_us) {
  return (uint32_t)(PrecisionTimer_GetUs() - start_us) >= timeout_us;
}

static void SD_DelayUs(uint32_t delay_us) {
  const uint32_t start_us = PrecisionTimer_GetUs();

  while (!SD_TimeoutExpired(start_us, delay_us)) {
  }
}

static void SD_Select(void) { GPIOB->BSRR = GPIO_BSRR_BR6; }

static void SD_Deselect(void) { GPIOB->BSRR = GPIO_BSRR_BS6; }

static void SD_GPIO_Init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  (void)RCC->AHB1ENR;

  SD_Deselect();

  GPIOB->MODER &=
      ~(GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
  GPIOB->MODER |=
      GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;

  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL13 | GPIO_AFRH_AFSEL14 | GPIO_AFRH_AFSEL15);
  GPIOB->AFR[1] |= (5U << GPIO_AFRH_AFSEL13_Pos) |
                   (5U << GPIO_AFRH_AFSEL14_Pos) |
                   (5U << GPIO_AFRH_AFSEL15_Pos);

  GPIOB->MODER &= ~GPIO_MODER_MODER6;
  GPIOB->MODER |= GPIO_MODER_MODER6_0;

  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT14 |
                     GPIO_OTYPER_OT15);

  GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED6 | GPIO_OSPEEDR_OSPEED13 |
                    GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15;

  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD14 |
                    GPIO_PUPDR_PUPD15);
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0 | GPIO_PUPDR_PUPD14_0;
}

static void SD_SPI_InitSlow(void) {
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  (void)RCC->APB1ENR;

  SPI2->CR1 = 0U;
  SPI2->CR2 = 0U;

  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_2 |
              SPI_CR1_BR_1 | SPI_CR1_BR_0;
  SPI2->CR1 |= SPI_CR1_SPE;
}

static SD_Status_t SD_WaitSPIIdle(void) {
  const uint32_t start_us = PrecisionTimer_GetUs();

  while ((SPI2->SR & SPI_SR_BSY) != 0U) {
    if (SD_TimeoutExpired(start_us, SD_SPI_BYTE_TIMEOUT_US)) {
      return SD_ERR_TIMEOUT;
    }
  }

  return SD_OK;
}

static SD_Status_t SD_SPISetFast(void) {
  SD_Status_t status = SD_WaitSPIIdle();

  if (status != SD_OK) {
    return status;
  }

  SPI2->CR1 &= ~SPI_CR1_SPE;
  SPI2->CR1 &= ~SPI_CR1_BR;
  SPI2->CR1 |= SPI_CR1_BR_1;
  SPI2->CR1 |= SPI_CR1_SPE;

  return SD_OK;
}

static SD_Status_t SD_SPITransfer(uint8_t tx, uint8_t *rx) {
  const uint32_t start_us = PrecisionTimer_GetUs();

  if (rx == NULL) {
    return SD_ERR_NULL;
  }

  while ((SPI2->SR & SPI_SR_TXE) == 0U) {
    if (SD_TimeoutExpired(start_us, SD_SPI_BYTE_TIMEOUT_US)) {
      return SD_ERR_TIMEOUT;
    }
  }

  *((volatile uint8_t *)&SPI2->DR) = tx;

  while ((SPI2->SR & SPI_SR_RXNE) == 0U) {
    if (SD_TimeoutExpired(start_us, SD_SPI_BYTE_TIMEOUT_US)) {
      return SD_ERR_TIMEOUT;
    }
  }

  *rx = *((volatile uint8_t *)&SPI2->DR);
  return SD_OK;
}

static SD_Status_t SD_WriteByte(uint8_t value) {
  uint8_t ignored;
  return SD_SPITransfer(value, &ignored);
}

static SD_Status_t SD_ReadByte(uint8_t *value) {
  return SD_SPITransfer(0xFFU, value);
}

static SD_Status_t SD_ReadBytes(uint8_t *buffer, uint32_t length) {
  for (uint32_t i = 0U; i < length; ++i) {
    SD_Status_t status = SD_ReadByte(&buffer[i]);

    if (status != SD_OK) {
      return status;
    }
  }

  return SD_OK;
}

static SD_Status_t SD_WaitReady(uint32_t timeout_us) {
  const uint32_t start_us = PrecisionTimer_GetUs();
  uint8_t value;

  do {
    SD_Status_t status = SD_ReadByte(&value);

    if (status != SD_OK) {
      return status;
    }

    if (value == 0xFFU) {
      return SD_OK;
    }
  } while (!SD_TimeoutExpired(start_us, timeout_us));

  return SD_ERR_TIMEOUT;
}

static SD_Status_t SD_EndTransaction(void) {
  SD_Status_t status = SD_WaitSPIIdle();

  SD_Deselect();

  if (status != SD_OK) {
    return status;
  }

  return SD_WriteByte(0xFFU);
}

static SD_Status_t SD_Finish(SD_Status_t operation_status) {
  SD_Status_t end_status = SD_EndTransaction();
  return operation_status != SD_OK ? operation_status : end_status;
}

static SD_Status_t SD_SendCommand(uint8_t command, uint32_t argument,
                                  uint8_t crc, uint8_t *r1) {
  const uint8_t packet[6] = {
      (uint8_t)(0x40U | command), (uint8_t)(argument >> 24),
      (uint8_t)(argument >> 16),  (uint8_t)(argument >> 8),
      (uint8_t)argument,          crc};

  if (r1 == NULL) {
    return SD_ERR_NULL;
  }

  SD_Select();

  SD_Status_t status = SD_WaitReady(SD_COMMAND_READY_TIMEOUT_US);

  for (uint32_t i = 0U; (i < sizeof(packet)) && (status == SD_OK); ++i) {
    status = SD_WriteByte(packet[i]);
  }

  if (status != SD_OK) {
    SD_Deselect();
    return status;
  }

  for (uint32_t i = 0U; i < 10U; ++i) {
    status = SD_ReadByte(r1);

    if (status != SD_OK) {
      SD_Deselect();
      return status;
    }

    if ((*r1 & 0x80U) == 0U) {
      return SD_OK;
    }
  }

  SD_Deselect();
  return SD_ERR_TIMEOUT;
}

SD_Status_t SD_Init(void) {
  SD_Status_t status;
  uint8_t r1 = 0xFFU;
  uint8_t response[4];
  uint8_t ignored;
  uint32_t start_us;

  sd_initialized = 0U;
  sd_card_type = SD_CARDTYPE_UNKNOWN;

  SD_GPIO_Init();
  SD_SPI_InitSlow();
  SD_Deselect();
  SD_DelayUs(2000U);

  for (uint32_t i = 0U; i < 10U; ++i) {
    status = SD_SPITransfer(0xFFU, &ignored);

    if (status != SD_OK) {
      return status;
    }
  }

  start_us = PrecisionTimer_GetUs();
  do {
    status = SD_SendCommand(SD_CMD0, 0U, 0x95U, &r1);

    if (status == SD_OK) {
      status = SD_EndTransaction();

      if (status != SD_OK) {
        return status;
      }

      if (r1 == SD_R1_IDLE) {
        break;
      }
    }
  } while (!SD_TimeoutExpired(start_us, SD_CMD0_TIMEOUT_US));

  if (r1 != SD_R1_IDLE) {
    return SD_ERR_RESPONSE;
  }

  status = SD_SendCommand(SD_CMD8, 0x000001AAU, 0x87U, &r1);

  if (status != SD_OK) {
    return status;
  }

  status = SD_ReadBytes(response, sizeof(response));
  status = SD_Finish(status);

  if ((status != SD_OK) || ((r1 & SD_R1_ILLEGAL_COMMAND) != 0U)) {
    return SD_ERR_UNSUPPORTED_CARD;
  }

  if ((r1 != SD_R1_IDLE) || ((response[2] & 0x0FU) != 0x01U) ||
      (response[3] != 0xAAU)) {
    return SD_ERR_RESPONSE;
  }

  /* CMD55 + ACMD41(HCS): wait until the card leaves idle state. */
  start_us = PrecisionTimer_GetUs();
  do {
    status = SD_SendCommand(SD_CMD55, 0U, 0xFFU, &r1);
    status = SD_Finish(status);

    if ((status != SD_OK) || ((r1 & (uint8_t)~SD_R1_IDLE) != 0U)) {
      return SD_ERR_RESPONSE;
    }

    status = SD_SendCommand(SD_ACMD41, 0x40000000U, 0xFFU, &r1);
    status = SD_Finish(status);

    if (status != SD_OK) {
      return status;
    }

    if (r1 == SD_R1_READY) {
      break;
    }

    if (r1 != SD_R1_IDLE) {
      return SD_ERR_RESPONSE;
    }
  } while (!SD_TimeoutExpired(start_us, SD_INIT_TIMEOUT_US));

  if (r1 != SD_R1_READY) {
    return SD_ERR_TIMEOUT;
  }

  status = SD_SendCommand(SD_CMD58, 0U, 0xFFU, &r1);

  if (status != SD_OK) {
    return status;
  }

  status = SD_ReadBytes(response, sizeof(response));
  status = SD_Finish(status);

  if (status != SD_OK) {
    return status;
  }

  if ((r1 != SD_R1_READY) || ((response[0] & 0x80U) == 0U)) {
    return SD_ERR_RESPONSE;
  }

  if ((response[0] & 0x40U) == 0U) {
    return SD_ERR_UNSUPPORTED_CARD;
  }

  sd_card_type = SD_CARDTYPE_SDHC_SDXC;
  status = SD_SPISetFast();

  if (status != SD_OK) {
    sd_card_type = SD_CARDTYPE_UNKNOWN;
    return status;
  }

  sd_initialized = 1U;
  return SD_OK;
}

SD_Status_t SD_ReadSector(uint32_t sector, uint8_t *buffer) {
  SD_Status_t status;
  uint8_t r1;
  uint8_t ignored;
  uint32_t start_us;

  if (buffer == NULL) {
    return SD_ERR_NULL;
  }

  if (sd_initialized == 0U) {
    return SD_ERR_NOT_INITIALIZED;
  }

  status = SD_SendCommand(SD_CMD17, sector, 0xFFU, &r1);

  if (status != SD_OK) {
    return status;
  }

  if (r1 != SD_R1_READY) {
    return SD_Finish(SD_ERR_RESPONSE);
  }

  start_us = PrecisionTimer_GetUs();
  do {
    status = SD_ReadByte(&ignored);

    if (status != SD_OK) {
      return SD_Finish(status);
    }

    if (ignored == SD_TOKEN_START_BLOCK) {
      break;
    }

    if (ignored != 0xFFU) {
      return SD_Finish(SD_ERR_TOKEN);
    }
  } while (!SD_TimeoutExpired(start_us, SD_READ_TOKEN_TIMEOUT_US));

  if (ignored != SD_TOKEN_START_BLOCK) {
    return SD_Finish(SD_ERR_TIMEOUT);
  }

  status = SD_ReadBytes(buffer, SD_SECTOR_SIZE);

  if (status == SD_OK) {
    status = SD_ReadByte(&ignored);
  }
  if (status == SD_OK) {
    status = SD_ReadByte(&ignored);
  }

  return SD_Finish(status);
}

SD_Status_t SD_WriteSector(uint32_t sector, const uint8_t *buffer) {
  SD_Status_t status;
  uint8_t r1;
  uint8_t response = 0xFFU;

  if (buffer == NULL) {
    return SD_ERR_NULL;
  }

  if (sd_initialized == 0U) {
    return SD_ERR_NOT_INITIALIZED;
  }

  status = SD_SendCommand(SD_CMD24, sector, 0xFFU, &r1);

  if (status != SD_OK) {
    return status;
  }

  if (r1 != SD_R1_READY) {
    return SD_Finish(SD_ERR_RESPONSE);
  }

  status = SD_WriteByte(0xFFU);
  if (status == SD_OK) {
    status = SD_WriteByte(SD_TOKEN_START_BLOCK);
  }

  for (uint32_t i = 0U; (i < SD_SECTOR_SIZE) && (status == SD_OK); ++i) {
    status = SD_WriteByte(buffer[i]);
  }

  if (status == SD_OK) {
    status = SD_WriteByte(0xFFU);
  }
  if (status == SD_OK) {
    status = SD_WriteByte(0xFFU);
  }

  for (uint32_t i = 0U; (i < 10U) && (status == SD_OK); ++i) {
    status = SD_ReadByte(&response);

    if (response != 0xFFU) {
      break;
    }
  }

  if (status != SD_OK) {
    return SD_Finish(status);
  }

  if ((response & 0x1FU) != SD_DATA_ACCEPTED) {
    return SD_Finish(SD_ERR_WRITE);
  }

  status = SD_WaitReady(SD_WRITE_BUSY_TIMEOUT_US);
  return SD_Finish(status);
}

SD_CardType_t SD_GetCardType(void) { return sd_card_type; }
