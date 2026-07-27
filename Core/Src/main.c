#include "stm32f4xx.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_uart.h"

#include "main.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "msp.h"

#include <math.h>
#include <stdio.h>


#define HMC5883L_CALIBRATION_TIME_MS    30000U


I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef husart2;

MPU6050_Handle_t mpu6050;

HMC5883L_Handle_t hmc5883l;
HMC5883L_RawData_t hmc_raw;
HMC5883L_Data_t hmc_scaled;
HMC5883L_Data_t hmc_calibrated;
HMC5883L_CalibrationSession_t hmc_cal_session;




/*
 * ISR chỉ tăng số sự kiện DRDY.
 * Không đọc I2C trong callback ngắt.
 */
static volatile uint32_t hmc_drdy_pending = 0U;
static volatile uint32_t hmc_drdy_total = 0U;


void SystemClockConfig(void)
{
    /* Dùng clock mặc định sau HAL_Init(). */
}


void I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed = I2C_CLOCK_SPEED_SM;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0U;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        while (1) {
        }
    }
}


void USART2_UART_Init(void)
{
    husart2.Instance = USART2;

    husart2.Init.BaudRate = 115200U;
    husart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    husart2.Init.WordLength = UART_WORDLENGTH_8B;
    husart2.Init.StopBits = UART_STOPBITS_1;
    husart2.Init.Parity = UART_PARITY_NONE;
    husart2.Init.Mode = UART_MODE_TX;
    husart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&husart2) != HAL_OK) {
        while (1) {
        }
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1) {
        hmc_drdy_pending++;
        hmc_drdy_total++;
    }
}





int main(void)
{


    HAL_Init();

    SystemClockConfig();
    I2C1_Init();
    USART2_UART_Init();


    Sensor_EXTI_Init();



    while (1) {

    }
}
