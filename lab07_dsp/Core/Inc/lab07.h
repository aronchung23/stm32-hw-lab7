/**
  ******************************************************************************
  * @file    lab07.h
  * @brief   HW-Lab7: objects shared by the three RTOS tasks and the TIM2 ISR
  *
  *   TIM2 ISR (64 Hz) --flag--> TASK_SENSOR (High)        : read LSM6DSL
  *                                  | rawQueue (SensorSample_t)
  *                                  v
  *                              TASK_DSP (AboveNormal)    : CMSIS-DSP FIR LPF, 16-sample blocks
  *                                  | txQueue (DspSample_t: raw + filtered)
  *                                  v
  *                              TASK_BLE (Normal)         : BLE notify to the PC + HCI events
  ******************************************************************************
  */
#ifndef LAB07_H
#define LAB07_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"

/* thread flag: TIM2 ISR -> TASK_SENSOR */
#define SENSOR_FLAG_SAMPLE   0x0001U

typedef struct
{
  uint16_t seq;          /* sample number                 */
  int16_t  xyz[3];       /* acceleration [mg]             */
} SensorSample_t;

typedef struct
{
  uint16_t seq;
  int16_t  raw[3];       /* before the filter [mg]        */
  int16_t  filt[3];      /* after the FIR low-pass [mg]   */
} DspSample_t;

extern osThreadId_t TASK_SENSORHandle;

#ifdef __cplusplus
}
#endif

#endif /* LAB07_H */
