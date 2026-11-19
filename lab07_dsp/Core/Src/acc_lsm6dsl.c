/**
  ******************************************************************************
  * @file    acc_lsm6dsl.c
  * @brief   Minimal LSM6DSL accelerometer driver over I2C2 (STM32 IoT node)
  *
  *          The LSM6DSL on B-L475E-IOT01A sits on I2C2 (PB10 SCL / PB11 SDA).
  ******************************************************************************
  */
#include "acc_lsm6dsl.h"

/* 8-bit I2C addresses (SA0 = 0 / SA0 = 1) */
#define LSM6DSL_ADDR_LOW     0xD4
#define LSM6DSL_ADDR_HIGH    0xD6

/* Registers */
#define LSM6DSL_WHO_AM_I     0x0F
#define LSM6DSL_CTRL1_XL     0x10
#define LSM6DSL_CTRL3_C      0x12
#define LSM6DSL_OUTX_L_XL    0x28
#define LSM6DSL_INT1_CTRL    0x0D
#define LSM6DSL_CTRL10_C     0x19
#define LSM6DSL_FUNC_SRC1    0x53

#define LSM6DSL_WHO_AM_I_VAL 0x6A
#define CTRL3_C_BDU_IF_INC   0x44     /* block data update + register auto-increment */
#define FS_XL_2G             0x00     /* CTRL1_XL FS_XL[1:0] = 00 -> +-2g */
#define SENSITIVITY_2G       0.061f   /* mg / LSB at +-2g */

#define INT1_SIGN_MOT        0x40     /* INT1_CTRL: route significant motion to INT1 */
#define CTRL10_FUNC_EN       0x04     /* CTRL10_C: enable embedded functions */
#define CTRL10_SIGN_MOTION   0x01     /* CTRL10_C: enable significant motion detection */
#define FUNC_SRC1_SIGN_MOT   0x40     /* FUNC_SRC1: significant motion event */

#define I2C_TIMEOUT_MS       100

static I2C_HandleTypeDef *acc_i2c;
static uint16_t acc_addr;

static HAL_StatusTypeDef acc_write(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(acc_i2c, acc_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef acc_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(acc_i2c, acc_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT_MS);
}

/**
  * @brief  LAB07: I2C bus recovery. If the MCU is reset in the middle of a read, the
  *         LSM6DSL may still hold SDA low, waiting for more clocks; then every transfer
  *         fails. Clock SCL (PB10) by hand until SDA (PB11) is released, send a STOP
  *         and re-initialize the I2C peripheral.
  */
void ACC_BusRecover(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef g = {0};

  HAL_I2C_DeInit(hi2c);
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_SET);
  g.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  for (int i = 0; i < 16 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 20; d++) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    for (volatile int d = 0; d < 20; d++) {}
  }
  /* STOP: SDA low -> high while SCL is high */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  for (volatile int d = 0; d < 20; d++) {}
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
  for (volatile int d = 0; d < 20; d++) {}

  HAL_I2C_Init(hi2c);                      /* MspInit puts PB10/PB11 back to I2C2 AF */
  HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(hi2c, 0);
}

HAL_StatusTypeDef ACC_Init(I2C_HandleTypeDef *hi2c)
{
  const uint16_t candidates[] = {LSM6DSL_ADDR_LOW, LSM6DSL_ADDR_HIGH};
  uint8_t id = 0;

  acc_i2c = hi2c;
  acc_addr = 0;
  for (uint32_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
  {
    acc_addr = candidates[i];
    if (acc_read(LSM6DSL_WHO_AM_I, &id, 1) == HAL_OK && id == LSM6DSL_WHO_AM_I_VAL)
    {
      break;
    }
    acc_addr = 0;
  }
  if (acc_addr == 0)
  {
    return HAL_ERROR;
  }

  if (acc_write(LSM6DSL_CTRL3_C, CTRL3_C_BDU_IF_INC) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return ACC_SetOdr(ACC_ODR_104HZ);
}

HAL_StatusTypeDef ACC_SetOdr(AccOdr_t odr)
{
  return acc_write(LSM6DSL_CTRL1_XL, (uint8_t)((odr << 4) | FS_XL_2G));
}

HAL_StatusTypeDef ACC_ReadXYZ(int16_t xyz_mg[3])
{
  uint8_t raw[6];

  if (acc_read(LSM6DSL_OUTX_L_XL, raw, sizeof(raw)) != HAL_OK)
  {
    return HAL_ERROR;
  }
  for (int i = 0; i < 3; i++)
  {
    int16_t lsb = (int16_t)((raw[2 * i + 1] << 8) | raw[2 * i]);
    xyz_mg[i] = (int16_t)(lsb * SENSITIVITY_2G);
  }
  return HAL_OK;
}

HAL_StatusTypeDef ACC_EnableSigMotion(void)
{
  uint8_t src;

  /* embedded functions need ODR >= 26 Hz; ACC_Init() already set 104 Hz */
  if (acc_write(LSM6DSL_CTRL10_C, CTRL10_FUNC_EN | CTRL10_SIGN_MOTION) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (acc_write(LSM6DSL_INT1_CTRL, INT1_SIGN_MOT) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* clear any pending event */
  return acc_read(LSM6DSL_FUNC_SRC1, &src, 1);
}

uint8_t ACC_SigMotionDetected(void)
{
  uint8_t src = 0;

  if (acc_read(LSM6DSL_FUNC_SRC1, &src, 1) != HAL_OK)
  {
    return 0;
  }
  return (src & FUNC_SRC1_SIGN_MOT) ? 1 : 0;
}
