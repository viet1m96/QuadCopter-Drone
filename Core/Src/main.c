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


static void HMC5883L_DRDY_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    hmc_drdy_pending = 0U;
    hmc_drdy_total = 0U;

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

    if (primask == 0U) {
        __enable_irq();
    }
}


static uint32_t HMC5883L_DRDY_TakePending(void)
{
    uint32_t pending;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    pending = hmc_drdy_pending;
    hmc_drdy_pending = 0U;

    if (primask == 0U) {
        __enable_irq();
    }

    return pending;
}


static uint8_t MPU6050_BypassSetup(void)
{
    MPU6050_Status_t status;

    status = MPU6050_Init(
            &mpu6050,
            &hi2c1,
            MPU6050_ADDRESS,
            MPU6050_CLKSRC_PLL_X,
            MPU6050_DLPF_CFG_3,
            MPU6050_GYRO_CONFIG_FS_250DPS,
            MPU6050_ACCEL_CONFIG_AFS_2G,
            9U);

    if (status != MPU6050_OK) {
        printf(
                "MPU6050 init failed: %s\r\n",
                MPU6050_ConvertStatusToString(status));

        return 0U;
    }

    status = MPU6050_EnableBypass(&mpu6050);

    if (status != MPU6050_OK) {
        printf(
                "MPU6050 bypass failed: %s\r\n",
                MPU6050_ConvertStatusToString(status));

        return 0U;
    }

    printf("MPU6050 bypass enabled\r\n");

    return 1U;
}


static uint8_t HMC5883L_Setup(void)
{
    HMC5883L_Config_t config = {
        .meas_mode = HMC5883L_MEAS_MODE_NORMAL,
        .output_rate = HMC5883L_OUTPUT_RATE_15,
        .sample_avg = HMC5883L_SAMPLE_AVG_8,
        .device_gain = HMC5883L_DEVICE_GAIN_1_3,
        .mode = HMC5883L_MODE_CONTINUOUS_MEAS
    };

    HMC5883L_Status_t status = HMC5883L_Init(
            &hmc5883l,
            &hi2c1,
            HMC5883L_ADDRESS,
            &config);

    if (status != HMC5883L_OK) {
        HMC5883L_NotiStatus(
                status,
                "HMC5883L init failed");

        return 0U;
    }

    printf(
            "HMC5883L initialized, scale=%.1f LSB/G\r\n",
            hmc5883l.lsb_per_gauss);

    return 1U;
}


static void HMC5883L_PrintCalibration(void)
{
    printf(
            "Hard-iron bias [G]: X=%.5f Y=%.5f Z=%.5f\r\n",
            hmc5883l.calibration.hard_iron_bias_g.x,
            hmc5883l.calibration.hard_iron_bias_g.y,
            hmc5883l.calibration.hard_iron_bias_g.z);

    printf("Soft-iron matrix:\r\n");

    printf(
            "[ %.5f %.5f %.5f ]\r\n",
            hmc5883l.calibration.soft_iron_matrix[0][0],
            hmc5883l.calibration.soft_iron_matrix[0][1],
            hmc5883l.calibration.soft_iron_matrix[0][2]);

    printf(
            "[ %.5f %.5f %.5f ]\r\n",
            hmc5883l.calibration.soft_iron_matrix[1][0],
            hmc5883l.calibration.soft_iron_matrix[1][1],
            hmc5883l.calibration.soft_iron_matrix[1][2]);

    printf(
            "[ %.5f %.5f %.5f ]\r\n",
            hmc5883l.calibration.soft_iron_matrix[2][0],
            hmc5883l.calibration.soft_iron_matrix[2][1],
            hmc5883l.calibration.soft_iron_matrix[2][2]);

    printf(
            "Calibration flags: 0x%02X\r\n\r\n",
            hmc5883l.calibration.valid_flags);
}


static HMC5883L_Status_t HMC5883L_RunCalibration(
        uint32_t duration_ms)
{
    HMC5883L_Status_t status;
    uint32_t start_tick;
    uint32_t missed_events = 0U;
    uint32_t read_errors = 0U;

    status = HMC5883L_CalibrationBegin(
            &hmc_cal_session);

    if (status != HMC5883L_OK) {
        return status;
    }

    HMC5883L_DRDY_Reset();

    status = HMC5883L_StartContinuousMeasurement(
            &hmc5883l);

    if (status != HMC5883L_OK) {
        return status;
    }

    printf("\r\nHMC5883L calibration started\r\n");
    printf(
            "Calibration time: %lu seconds\r\n",
            (unsigned long)(duration_ms / 1000U));
    printf("Rotate the sensor slowly through all orientations\r\n");

    start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) < duration_ms) {
        uint32_t events = HMC5883L_DRDY_TakePending();

        if (events == 0U) {
            continue;
        }

        /*
         * HMC5883L chỉ giữ bộ dữ liệu mới nhất.
         * Nếu events > 1 thì main đã xử lý chậm và mất một số conversion.
         */
        if (events > 1U) {
            missed_events += events - 1U;
        }

        status = HMC5883L_ReadRawData(
                &hmc5883l,
                &hmc_raw);

        if (status == HMC5883L_DATA_OVERFLOW) {
            hmc_cal_session.rejected_samples++;
            continue;
        }

        if (status != HMC5883L_OK) {
            read_errors++;
            continue;
        }

        status = HMC5883L_ConvertRawToScaled(
                &hmc5883l,
                &hmc_raw,
                &hmc_scaled);

        if (status != HMC5883L_OK) {
            read_errors++;
            continue;
        }

        status = HMC5883L_CalibrationAddSample(
                &hmc_cal_session,
                &hmc_scaled);

        if (status != HMC5883L_OK) {
            hmc_cal_session.rejected_samples++;
        }
    }

    HMC5883L_StopMeasurement(&hmc5883l);

    printf(
            "DRDY events=%lu accepted=%lu rejected=%lu "
            "missed=%lu read_errors=%lu\r\n",
            (unsigned long)hmc_drdy_total,
            (unsigned long)hmc_cal_session.accepted_samples,
            (unsigned long)hmc_cal_session.rejected_samples,
            (unsigned long)missed_events,
            (unsigned long)read_errors);

    status = HMC5883L_CalibrationFinish(
            &hmc5883l,
            &hmc_cal_session);

    if (status != HMC5883L_OK) {
        return status;
    }

    printf("HMC5883L calibration completed\r\n");
    HMC5883L_PrintCalibration();

    return HMC5883L_OK;
}


static HMC5883L_Status_t HMC5883L_StartLiveTest(void)
{
    HMC5883L_DRDY_Reset();

    HMC5883L_Status_t status =
            HMC5883L_StartContinuousMeasurement(
                    &hmc5883l);

    if (status == HMC5883L_OK) {
        printf("HMC5883L DRDY live test started\r\n");
        printf("Expected dt at 15 Hz: about 66-67 ms\r\n\r\n");
    }

    return status;
}


int main(void)
{
    HMC5883L_Status_t status;

    uint32_t last_sample_tick = 0U;
    uint32_t missed_events = 0U;
    uint8_t has_previous_sample = 0U;

    HAL_Init();

    SystemClockConfig();
    I2C1_Init();
    USART2_UART_Init();

    /*
     * PB1/EXTI1 đã được cấu hình trong msp.c.
     * EXTI1_IRQHandler đã được định nghĩa trong it.c.
     */
    Sensor_EXTI_Init();

    printf("\r\nBOOT\r\n");

    if (MPU6050_BypassSetup() == 0U) {
        printf("Cannot access HMC5883L without bypass\r\n");

        while (1) {
            HAL_Delay(1000U);
        }
    }

    if (HMC5883L_Setup() == 0U) {
        while (1) {
            HAL_Delay(1000U);
        }
    }

    status = HMC5883L_RunCalibration(
            HMC5883L_CALIBRATION_TIME_MS);

    if (status != HMC5883L_OK) {
        HMC5883L_NotiStatus(
                status,
                "HMC5883L calibration failed");

        printf("Continue with default calibration\r\n\r\n");
    }

    status = HMC5883L_StartLiveTest();

    if (status != HMC5883L_OK) {
        HMC5883L_NotiStatus(
                status,
                "Cannot start HMC5883L continuous measurement");

        while (1) {
            HAL_Delay(1000U);
        }
    }

    while (1) {
        uint32_t events = HMC5883L_DRDY_TakePending();

        if (events == 0U) {
            continue;
        }

        if (events > 1U) {
            missed_events += events - 1U;
        }

        status = HMC5883L_ReadRawData(
                &hmc5883l,
                &hmc_raw);

        if (status != HMC5883L_OK) {
            HMC5883L_NotiStatus(
                    status,
                    "HMC5883L read failed");
            continue;
        }

        status = HMC5883L_ConvertRawToScaled(
                &hmc5883l,
                &hmc_raw,
                &hmc_scaled);

        if (status != HMC5883L_OK) {
            HMC5883L_NotiStatus(
                    status,
                    "HMC5883L conversion failed");
            continue;
        }

        status = HMC5883L_ApplyCalibration(
                &hmc5883l,
                &hmc_scaled,
                &hmc_calibrated);

        if (status != HMC5883L_OK) {
            HMC5883L_NotiStatus(
                    status,
                    "HMC5883L apply calibration failed");
            continue;
        }

        uint32_t now = HAL_GetTick();
        uint32_t dt_ms = 0U;

        if (has_previous_sample != 0U) {
            dt_ms = now - last_sample_tick;
        } else {
            has_previous_sample = 1U;
        }

        last_sample_tick = now;

        float magnitude_g = sqrtf(
                hmc_calibrated.x_g * hmc_calibrated.x_g +
                hmc_calibrated.y_g * hmc_calibrated.y_g +
                hmc_calibrated.z_g * hmc_calibrated.z_g);

        printf(
                "DRDY=%lu dt=%lu ms missed=%lu "
                "Raw=[%d %d %d] "
                "Uncal=[%.4f %.4f %.4f]G "
                "Cal=[%.4f %.4f %.4f]G "
                "|B|=%.4fG\r\n",

                (unsigned long)hmc_drdy_total,
                (unsigned long)dt_ms,
                (unsigned long)missed_events,

                (int)hmc_raw.x,
                (int)hmc_raw.y,
                (int)hmc_raw.z,

                hmc_scaled.x_g,
                hmc_scaled.y_g,
                hmc_scaled.z_g,

                hmc_calibrated.x_g,
                hmc_calibrated.y_g,
                hmc_calibrated.z_g,

                magnitude_g);
    }
}
