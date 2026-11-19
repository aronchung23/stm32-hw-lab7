/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "hci.h"
#include "hci_tl_interface.h"
#include "sensor.h"
#include "acc_lsm6dsl.h"
#include "bluenrg_def.h"
#include "dsp_service.h"
#include "dsp_filter.h"
#include "lab07.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAW_QUEUE_LEN   32U    /* TASK_SENSOR -> TASK_DSP : 0.5 s of samples */
#define TX_QUEUE_LEN    64U    /* TASK_DSP    -> TASK_BLE : 1 s of samples   */
#define BLE_RETRY_MAX   20U    /* notify retries while the BlueNRG TX buffers are full */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* Definitions for TASK_SENSOR */
osThreadId_t TASK_SENSORHandle;
const osThreadAttr_t TASK_SENSOR_attributes = {
  .name = "TASK_SENSOR",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TASK_DSP */
osThreadId_t TASK_DSPHandle;
const osThreadAttr_t TASK_DSP_attributes = {
  .name = "TASK_DSP",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TASK_BLE */
osThreadId_t TASK_BLEHandle;
const osThreadAttr_t TASK_BLE_attributes = {
  .name = "TASK_BLE",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osMessageQueueId_t rawQueueHandle;   /* TASK_SENSOR -> TASK_DSP */
osMessageQueueId_t txQueueHandle;    /* TASK_DSP    -> TASK_BLE */

/* statistics */
static volatile uint32_t rawDropped = 0;
static volatile uint32_t txDropped = 0;
static volatile uint32_t bleSent = 0;
static volatile uint32_t bleFailed = 0;

/* BLE connection state, owned by sensor.c */
extern __IO uint8_t set_connectable;
extern __IO uint32_t connected;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
void StartTaskSensor(void *argument);
void StartTaskDSP(void *argument);
void StartTaskBLE(void *argument);

/* USER CODE BEGIN PFP */
static void SamplingTimer_Start(uint32_t fs_hz);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  DSP_CycleCounterInit();
  printf("\r\n##### HW-Lab7: CMSIS-DSP FIR low-pass on the LSM6DSL accelerometer #####\r\n");
  printf("SYSCLK = %lu Hz, CMSIS-RTOS2 (FreeRTOS), BLE name \"LAB7DSP\"\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  rawQueueHandle = osMessageQueueNew(RAW_QUEUE_LEN, sizeof(SensorSample_t), NULL);
  txQueueHandle  = osMessageQueueNew(TX_QUEUE_LEN, sizeof(DspSample_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TASK_SENSOR */
  TASK_SENSORHandle = osThreadNew(StartTaskSensor, NULL, &TASK_SENSOR_attributes);

  /* creation of TASK_DSP */
  TASK_DSPHandle = osThreadNew(StartTaskDSP, NULL, &TASK_DSP_attributes);

  /* creation of TASK_BLE */
  TASK_BLEHandle = osThreadNew(StartTaskBLE, NULL, &TASK_BLE_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  HCI_TL_SPI_Send() with the BlueNRG IRQ (EXTI9_5) masked, so the ISR's SPI read
  *         cannot interleave with a task's SPI write. An IRQ raised meanwhile stays pending
  *         and is served as soon as it is unmasked.
  */
int32_t HCI_TL_SPI_Send_Guarded(uint8_t* buffer, uint16_t size)
{
  HAL_NVIC_DisableIRQ(HCI_TL_SPI_EXTI_IRQn);
  int32_t ret = HCI_TL_SPI_Send(buffer, size);
  HAL_NVIC_EnableIRQ(HCI_TL_SPI_EXTI_IRQn);
  return ret;
}

/**
  * @brief  TIM2 = sampling clock. Update event every 1/fs s -> TIM2_IRQHandler (stm32l4xx_it.c)
  *         -> thread flag to TASK_SENSOR.
  *         TIM2 clock = PCLK1 (APB1 prescaler 1) = 4 MHz (MSI)
  *         PSC = 4 MHz / 1 MHz - 1 = 3, ARR = 1 MHz / 64 Hz - 1 = 15624  -> 64.000 Hz
  */
static void SamplingTimer_Start(uint32_t fs_hz)
{
  uint32_t timclk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    timclk *= 2U;   /* timer clock is 2 x PCLK1 when APB1 is divided */
  }

  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->CR1  = 0;
  TIM2->PSC  = timclk / 1000000U - 1U;     /* 1 MHz counter */
  TIM2->ARR  = 1000000U / fs_hz - 1U;
  TIM2->EGR  = TIM_EGR_UG;                 /* load PSC/ARR */
  TIM2->SR   = 0;
  TIM2->DIER = TIM_DIER_UIE;
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);   /* = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  TIM2->CR1  = TIM_CR1_CEN;

  printf("TIM2: clk = %lu Hz, PSC = %lu, ARR = %lu -> fs = %lu Hz\r\n",
         (unsigned long)timclk, (unsigned long)TIM2->PSC, (unsigned long)TIM2->ARR,
         (unsigned long)(timclk / (TIM2->PSC + 1U) / (TIM2->ARR + 1U)));
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskSensor */
/**
  * @brief  TASK_SENSOR (highest priority): every TIM2 tick, read the LSM6DSL
  *         accelerometer over I2C2 and pass the sample to TASK_DSP.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskSensor */
void StartTaskSensor(void *argument)
{
  /* USER CODE BEGIN 5 */
  SensorSample_t s;
  uint16_t seq = 0;

  /* the board may have been reset in the middle of an I2C read: recover the bus and retry */
  int tries = 0;
  while (ACC_Init(&hi2c2) != HAL_OK)
  {
    if (++tries >= 5)
    {
      printf("LSM6DSL init failed!\r\n");
      osThreadSuspend(osThreadGetId());
    }
    printf("LSM6DSL not answering, I2C bus recovery (try %d)\r\n", tries);
    ACC_BusRecover(&hi2c2);
    osDelay(10);
  }
  printf("LSM6DSL ready (ODR 104 Hz, +-2 g), TASK_SENSOR waits for TIM2\r\n");

  for(;;)
  {
    /* released by TIM2_IRQHandler at DSP_FS_HZ */
    osThreadFlagsWait(SENSOR_FLAG_SAMPLE, osFlagsWaitAny, osWaitForever);

    if (ACC_ReadXYZ(s.xyz) == HAL_OK)
    {
      s.seq = seq++;
      if (osMessageQueuePut(rawQueueHandle, &s, 0, 0) != osOK)
      {
        rawDropped++;
      }
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskDSP */
/**
  * @brief  TASK_DSP: 1) known-signal tests of the CMSIS-DSP FIR,
  *                   2) collects 16 samples, filters the 3 axes with arm_fir_f32(),
  *                      passes raw + filtered samples to TASK_BLE.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskDSP */
void StartTaskDSP(void *argument)
{
  /* USER CODE BEGIN StartTaskDSP */
  static int16_t in[DSP_BLOCK_SIZE][DSP_NUM_AXES];
  static int16_t out[DSP_BLOCK_SIZE][DSP_NUM_AXES];
  static uint16_t seqs[DSP_BLOCK_SIZE];
  SensorSample_t s;
  DspSample_t d;
  uint32_t block = 0;

  /* ---- Basic problem: verify the program with known signals first ---- */
  int ok1 = DSP_SelfTest_ArmFirExample();
  int ok2 = DSP_SelfTest_AccLpf();
  printf("\r\n===== Known-signal tests: Test 1 %s, Test 2 %s =====\r\n\r\n",
         ok1 ? "SUCCESS" : "FAILURE", ok2 ? "SUCCESS" : "FAILURE");

  /* ---- then process the real accelerometer data ---- */
  DSP_FilterInit();
  SamplingTimer_Start(DSP_FS_HZ);
  printf("Real-time: fs = %u Hz, FIR %u taps (fc = 4 Hz), block = %u samples; one line per block:\r\n",
         (unsigned)DSP_FS_HZ, (unsigned)DSP_NUM_TAPS, (unsigned)DSP_BLOCK_SIZE);

  for(;;)
  {
    for (uint32_t n = 0; n < DSP_BLOCK_SIZE; n++)
    {
      osMessageQueueGet(rawQueueHandle, &s, NULL, osWaitForever);
      seqs[n] = s.seq;
      in[n][0] = s.xyz[0];
      in[n][1] = s.xyz[1];
      in[n][2] = s.xyz[2];
    }

    uint32_t cycles = DSP_FilterBlock(in, out);

    for (uint32_t n = 0; n < DSP_BLOCK_SIZE; n++)
    {
      d.seq = seqs[n];
      for (uint32_t a = 0; a < DSP_NUM_AXES; a++)
      {
        d.raw[a]  = in[n][a];
        d.filt[a] = out[n][a];
      }
      if (osMessageQueuePut(txQueueHandle, &d, 0, 0) != osOK)
      {
        txDropped++;
      }
    }

    const uint32_t L = DSP_BLOCK_SIZE - 1U;
    printf("[%5lu] seq %5u raw X=%5d Y=%5d Z=%5d | filt X=%5d Y=%5d Z=%5d mg | FIR %lu cyc | BLE %s sent=%lu drop=%lu\r\n",
           (unsigned long)block++, seqs[L], in[L][0], in[L][1], in[L][2], out[L][0], out[L][1], out[L][2],
           (unsigned long)cycles, (connected && DspService_NotifyEnabled()) ? "on " : "off",
           (unsigned long)bleSent, (unsigned long)(rawDropped + txDropped + bleFailed));
  }
  /* USER CODE END StartTaskDSP */
}

/* USER CODE BEGIN Header_StartTaskBLE */
/**
  * @brief  TASK_BLE: BLE event handling (connect / disconnect / CCCD writes) and
  *         sends every raw + filtered sample to the PC as a GATT notification.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskBLE */
void StartTaskBLE(void *argument)
{
  /* USER CODE BEGIN StartTaskBLE */
  DspSample_t d;

  for(;;)
  {
    /* dispatches queued HCI events to user_notify() in sensor.c */
    hci_user_evt_proc();
    if (set_connectable)
    {
      /* (re)start advertising after reset or disconnection */
      Set_DeviceConnectable();
      set_connectable = FALSE;
    }

    if (osMessageQueueGet(txQueueHandle, &d, NULL, 5) == osOK)
    {
      if (connected && DspService_NotifyEnabled())
      {
        uint8_t ret;
        uint32_t tries = 0;
        while ((ret = DspService_Notify(d.seq, d.raw, d.filt)) == BLE_STATUS_INSUFFICIENT_RESOURCES
               && tries++ < BLE_RETRY_MAX)
        {
          /* BlueNRG TX buffers full: let the radio send, then retry */
          hci_user_evt_proc();
          osDelay(2);
        }
        if (ret == BLE_STATUS_SUCCESS) bleSent++; else bleFailed++;
      }
      /* not connected: the sample is just discarded */
    }
  }
  /* USER CODE END StartTaskBLE */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
