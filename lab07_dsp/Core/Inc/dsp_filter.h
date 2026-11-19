/**
  ******************************************************************************
  * @file    dsp_filter.h
  * @brief   HW-Lab7: CMSIS-DSP FIR low-pass filter for the LSM6DSL accelerometer
  *
  *  - fs = 64 Hz (TIM2 update interrupt), 29-tap FIR low-pass, fc = 4 Hz
  *    coefficients = MATLAB fir1(28, 4/32)  (Hamming window, linear phase,
  *    group delay = 14 samples = 219 ms)
  *  - processed block by block with arm_fir_f32(), one FIR instance per axis
  ******************************************************************************
  */
#ifndef DSP_FILTER_H
#define DSP_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DSP_FS_HZ          64U     /* sampling frequency of the accelerometer   */
#define DSP_NUM_TAPS       29U     /* FIR length (order 28)                     */
#define DSP_BLOCK_SIZE     16U     /* samples per arm_fir_f32() call (250 ms)   */
#define DSP_NUM_AXES       3U

/* Known-signal tests, printed on the UART. Return 1 = PASS, 0 = FAIL */
int DSP_SelfTest_ArmFirExample(void);  /* CMSIS arm_fir_example: 1 kHz + 15 kHz @ 48 kHz vs MATLAB reference */
int DSP_SelfTest_AccLpf(void);         /* our accelerometer LPF: 1 Hz + 16 Hz @ 64 Hz, checked by SNR + FFT    */

/* Real-time filter (one FIR state per axis) */
void     DSP_FilterInit(void);
/* in/out: [DSP_BLOCK_SIZE][3] samples in mg. Returns the CPU cycles spent in arm_fir_f32(). */
uint32_t DSP_FilterBlock(const int16_t in[][DSP_NUM_AXES], int16_t out[][DSP_NUM_AXES]);

/* DWT cycle counter helpers */
void     DSP_CycleCounterInit(void);
uint32_t DSP_Cycles(void);

/* printf helper without -u _printf_float: writes v with 'dec' decimals into buf */
const char *DSP_FloatStr(char *buf, float v, int dec);

#ifdef __cplusplus
}
#endif

#endif /* DSP_FILTER_H */
