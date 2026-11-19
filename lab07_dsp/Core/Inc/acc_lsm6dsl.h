/**
  ******************************************************************************
  * @file    acc_lsm6dsl.h
  * @brief   Minimal LSM6DSL accelerometer driver over I2C2 (STM32 IoT node)
  ******************************************************************************
  */
#ifndef ACC_LSM6DSL_H
#define ACC_LSM6DSL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* Output data rates supported by CTRL1_XL (value = ODR_XL[3:0]) */
typedef enum
{
  ACC_ODR_OFF    = 0x0,
  ACC_ODR_12_5HZ = 0x1,
  ACC_ODR_26HZ   = 0x2,
  ACC_ODR_52HZ   = 0x3,
  ACC_ODR_104HZ  = 0x4,
  ACC_ODR_208HZ  = 0x5,
  ACC_ODR_416HZ  = 0x6,
  ACC_ODR_833HZ  = 0x7,
  ACC_ODR_1660HZ = 0x8,
} AccOdr_t;

/* Initialize the sensor (+-2g full scale, 104 Hz). Returns HAL_OK on success. */
HAL_StatusTypeDef ACC_Init(I2C_HandleTypeDef *hi2c);

/* LAB07: free a stuck I2C bus (SDA held low after an MCU reset) and re-init the I2C peripheral */
void ACC_BusRecover(I2C_HandleTypeDef *hi2c);

/* Change the sensor's internal output data rate. */
HAL_StatusTypeDef ACC_SetOdr(AccOdr_t odr);

/* Read the 3-axis acceleration in mg. */
HAL_StatusTypeDef ACC_ReadXYZ(int16_t xyz_mg[3]);

/* Enable the embedded significant motion detection, signalled on INT1 (PD11). */
HAL_StatusTypeDef ACC_EnableSigMotion(void);

/* Read (and clear) the event source; returns 1 if significant motion was detected. */
uint8_t ACC_SigMotionDetected(void);

#ifdef __cplusplus
}
#endif

#endif /* ACC_LSM6DSL_H */
