#include "board_i2c1.h"

#include "main.h"
#include "peripherals.h"
#include "stm32f4xx_hal.h"

#define I2C1_SCL_PIN GPIO_PIN_8
#define I2C1_SDA_PIN GPIO_PIN_9
#define I2C1_GPIO_PORT GPIOB
#define I2C1_RECOVERY_CLOCK_PULSES 9U
#define I2C1_RECOVERY_DELAY_MS 1U

static void I2C1_ResetPeripheral(void) {
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
}

static uint8_t I2C1_RecoverLines(void) {
  GPIO_InitTypeDef gpio = {0};
  uint8_t recovery_ok = 1U;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN, GPIO_PIN_SET);

  gpio.Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(I2C1_GPIO_PORT, &gpio);

  HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN, GPIO_PIN_SET);
  HAL_Delay(I2C1_RECOVERY_DELAY_MS);

  if (HAL_GPIO_ReadPin(I2C1_GPIO_PORT, I2C1_SCL_PIN) == GPIO_PIN_RESET) {
    recovery_ok = 0U;
  }

  for (uint32_t pulse = 0U;
       pulse < I2C1_RECOVERY_CLOCK_PULSES && recovery_ok != 0U; ++pulse) {
    if (HAL_GPIO_ReadPin(I2C1_GPIO_PORT, I2C1_SDA_PIN) == GPIO_PIN_SET) {
      break;
    }

    HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
    HAL_Delay(I2C1_RECOVERY_DELAY_MS);

    HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(I2C1_RECOVERY_DELAY_MS);

    if (HAL_GPIO_ReadPin(I2C1_GPIO_PORT, I2C1_SCL_PIN) == GPIO_PIN_RESET) {
      recovery_ok = 0U;
    }
  }

  if (recovery_ok != 0U) {
    HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(I2C1_RECOVERY_DELAY_MS);

    HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(I2C1_RECOVERY_DELAY_MS);

    if (HAL_GPIO_ReadPin(I2C1_GPIO_PORT, I2C1_SCL_PIN) == GPIO_PIN_RESET) {
      recovery_ok = 0U;
    }
  }

  if (recovery_ok != 0U) {
    HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(I2C1_RECOVERY_DELAY_MS);

    recovery_ok =
        HAL_GPIO_ReadPin(I2C1_GPIO_PORT, I2C1_SCL_PIN) == GPIO_PIN_SET &&
        HAL_GPIO_ReadPin(I2C1_GPIO_PORT, I2C1_SDA_PIN) == GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(I2C1_GPIO_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN, GPIO_PIN_SET);
  HAL_GPIO_DeInit(I2C1_GPIO_PORT, I2C1_SCL_PIN | I2C1_SDA_PIN);

  return recovery_ok;
}

static uint8_t I2C1_InitPeripheral(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = I2C_CLOCK_SPEED_FM;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.OwnAddress2 = 0U;

  return HAL_I2C_Init(&hi2c1) == HAL_OK;
}

uint8_t I2C1_Recover(void) {
  HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

  HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
  HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);

  hi2c1.Instance = I2C1;

  if (hi2c1.State != HAL_I2C_STATE_RESET) {
    if (HAL_I2C_DeInit(&hi2c1) != HAL_OK) {
      return 0U;
    }
  }

  I2C1_ResetPeripheral();

  if (I2C1_RecoverLines() == 0U) {
    return 0U;
  }

  if (I2C1_InitPeripheral() == 0U) {
    (void)HAL_I2C_DeInit(&hi2c1);
    I2C1_ResetPeripheral();
    return 0U;
  }

  return 1U;
}
