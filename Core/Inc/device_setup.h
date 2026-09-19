#ifndef CORE_INC_DEVICE_SETUP_H_
#define CORE_INC_DEVICE_SETUP_H_

#include <stdint.h>

#include "device_IO.h"
#include "hal_ibus_transport.h"
#include "motor_pwm.h"
#include "mpu6050.h"
#include "rc_input.h"

uint8_t IBUS_Setup(HAL_IBUS_Transport_t *transport);
uint8_t RCInput_Setup(RCInput_Handle_t *rc_input);
uint8_t MotorPWM_Setup(MotorPWM_Handle_t *motor_pwm);
uint8_t MPU6050_Setup(MPU6050_Handle_t *mpu, DeviceIO_t *device_io);
uint8_t MPU6050_EnableDataReadyInterrupt(MPU6050_Handle_t *mpu);

#endif
