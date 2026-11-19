/**
  ******************************************************************************
  * @file    dsp_filter.c
  * @brief   HW-Lab7: CMSIS-DSP programming
  *
  *   1) DSP_SelfTest_ArmFirExample(): the CMSIS-DSP arm_fir_example ported to
  *      CubeIDE (input 1 kHz + 15 kHz, fs = 48 kHz, fir1(28, 6/24)); the output
  *      is compared with the MATLAB reference output by arm_snr_f32().
  *      Also benchmarks arm_fir_f32() against a plain C FIR loop (DWT cycles).
  *   2) DSP_SelfTest_AccLpf(): the filter we use for the accelerometer, tested
  *      with a known signal x[n] = sin(2*pi*1*n/64) + 0.5*sin(2*pi*16*n/64):
  *        - time domain: output vs ideal output |H(1 Hz)|*sin(2*pi*1*(n-14)/64)
  *        - frequency domain: 256-point FFT (arm_cfft_f32) of input and output
  *   3) DSP_FilterInit() / DSP_FilterBlock(): real-time FIR on the 3 axes.
  ******************************************************************************
  */
#include <stdio.h>
#include <math.h>
#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "math_helper.h"          /* arm_snr_f32() from the CMSIS example */
#include "dsp_filter.h"

/* ========================================================================== */
/*  Helpers                                                                   */
/* ========================================================================== */
void DSP_CycleCounterInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DSP_Cycles(void)
{
  return DWT->CYCCNT;
}

const char *DSP_FloatStr(char *buf, float v, int dec)
{
  static const uint32_t pow10[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
  if (dec < 0) dec = 0;
  if (dec > 6) dec = 6;
  const char *sign = (v < 0.0f) ? "-" : "+";
  if (v < 0.0f) v = -v;
  uint32_t scaled = (uint32_t)(v * (float)pow10[dec] + 0.5f);
  uint32_t ip = scaled / pow10[dec];
  uint32_t fp = scaled % pow10[dec];
  if (dec == 0)
    sprintf(buf, "%s%lu", sign, (unsigned long)ip);
  else
    sprintf(buf, "%s%lu.%0*lu", sign, (unsigned long)ip, dec, (unsigned long)fp);
  return buf;
}

static float to_db(float ratio)
{
  return 20.0f * log10f(ratio + 1e-12f);
}

/* ========================================================================== */
/*  Test 1: CMSIS-DSP arm_fir_example (Drivers/CMSIS/DSP/Examples/ARM/...)    */
/* ========================================================================== */
#define EX_TEST_LENGTH_SAMPLES  320
#define EX_SNR_THRESHOLD_F32    140.0f
#define EX_BLOCK_SIZE           32
#define EX_NUM_TAPS             29

/* input signal and MATLAB reference output: arm_fir_data.c (copied from the example) */
extern float32_t testInput_f32_1kHz_15kHz[EX_TEST_LENGTH_SAMPLES];
extern float32_t refOutput[EX_TEST_LENGTH_SAMPLES];

static float32_t exOutput[EX_TEST_LENGTH_SAMPLES];
static float32_t exOutputC[EX_TEST_LENGTH_SAMPLES];
static float32_t exState[EX_BLOCK_SIZE + EX_NUM_TAPS - 1];

/* fir1(28, 6/24): 6 kHz cutoff at fs = 48 kHz */
static const float32_t exCoeffs[EX_NUM_TAPS] = {
  -0.0018225230f, -0.0015879294f, +0.0000000000f, +0.0036977508f, +0.0080754303f, +0.0085302217f, -0.0000000000f, -0.0173976984f,
  -0.0341458607f, -0.0333591565f, +0.0000000000f, +0.0676308395f, +0.1522061835f, +0.2229246956f, +0.2504960933f, +0.2229246956f,
  +0.1522061835f, +0.0676308395f, +0.0000000000f, -0.0333591565f, -0.0341458607f, -0.0173976984f, -0.0000000000f, +0.0085302217f,
  +0.0080754303f, +0.0036977508f, +0.0000000000f, -0.0015879294f, -0.0018225230f
};

/* plain C direct-form FIR (y[n] = sum h[k] x[n-k]), used as a cross-check and benchmark */
static void fir_plain_c(const float *h, int taps, const float *x, float *y, int len)
{
  for (int n = 0; n < len; n++)
  {
    float acc = 0.0f;
    for (int k = 0; k < taps; k++)
    {
      if (n - k >= 0)
      {
        acc += h[k] * x[n - k];
      }
    }
    y[n] = acc;
  }
}

int DSP_SelfTest_ArmFirExample(void)
{
  arm_fir_instance_f32 S;
  char b1[16], b2[16], b3[16];
  uint32_t numBlocks = EX_TEST_LENGTH_SAMPLES / EX_BLOCK_SIZE;

  printf("\r\n================ Test 1: CMSIS-DSP arm_fir_example ================\r\n");
  printf("input = sin(2*pi*1kHz*t) + 0.5*sin(2*pi*15kHz*t + pi/4), fs = 48 kHz, %d samples\r\n", EX_TEST_LENGTH_SAMPLES);
  printf("filter = fir1(28, 6/24): %d-tap low-pass, fc = 6 kHz, block size %d\r\n", EX_NUM_TAPS, EX_BLOCK_SIZE);

  /* --- CMSIS-DSP --- */
  uint32_t t0 = DSP_Cycles();
  arm_fir_init_f32(&S, EX_NUM_TAPS, (float32_t *)&exCoeffs[0], &exState[0], EX_BLOCK_SIZE);
  for (uint32_t i = 0; i < numBlocks; i++)
  {
    arm_fir_f32(&S, testInput_f32_1kHz_15kHz + (i * EX_BLOCK_SIZE), exOutput + (i * EX_BLOCK_SIZE), EX_BLOCK_SIZE);
  }
  uint32_t cyc_cmsis = DSP_Cycles() - t0;

  /* --- plain C --- */
  t0 = DSP_Cycles();
  fir_plain_c(exCoeffs, EX_NUM_TAPS, testInput_f32_1kHz_15kHz, exOutputC, EX_TEST_LENGTH_SAMPLES);
  uint32_t cyc_c = DSP_Cycles() - t0;

  /* compare with the MATLAB reference */
  float32_t snr = arm_snr_f32(&refOutput[0], &exOutput[0], EX_TEST_LENGTH_SAMPLES);
  float32_t maxdiff = 0.0f;
  for (int n = 0; n < EX_TEST_LENGTH_SAMPLES; n++)
  {
    float d = fabsf(exOutput[n] - exOutputC[n]);
    if (d > maxdiff) maxdiff = d;
  }

  printf("   n      input     arm_fir_f32   MATLAB ref\r\n");
  for (int n = 0; n < EX_TEST_LENGTH_SAMPLES; n += 16)
  {
    printf(" %3d  %10s  %12s  %11s\r\n", n,
           DSP_FloatStr(b1, testInput_f32_1kHz_15kHz[n], 6),
           DSP_FloatStr(b2, exOutput[n], 6),
           DSP_FloatStr(b3, refOutput[n], 6));
  }
  printf("SNR(arm_fir_f32 vs MATLAB refOutput) = %s dB  (threshold %d dB)\r\n",
         DSP_FloatStr(b1, snr, 2), (int)EX_SNR_THRESHOLD_F32);
  printf("max |arm_fir_f32 - plain C FIR|     = %s\r\n", DSP_FloatStr(b1, maxdiff, 6));
  printf("cycles for 320 samples: arm_fir_f32 = %lu, plain C = %lu  (%s x faster)\r\n",
         (unsigned long)cyc_cmsis, (unsigned long)cyc_c,
         DSP_FloatStr(b1, (float)cyc_c / (float)cyc_cmsis, 2));

  int pass = (snr >= EX_SNR_THRESHOLD_F32);
  printf("Test 1 result: %s\r\n", pass ? "SUCCESS" : "FAILURE");
  return pass;
}

/* ========================================================================== */
/*  Accelerometer low-pass filter: fs = 64 Hz, fc = 4 Hz, 29 taps             */
/* ========================================================================== */
/* scipy.signal.firwin(29, 4/32) == MATLAB fir1(28, 4/32) (Hamming window).
 * arm_fir_f32() expects the coefficients in time-reversed order; this filter is
 * symmetric (linear phase), so fliplr(h) == h. */
static const float32_t lpfCoeffs[DSP_NUM_TAPS] = {
  -0.0012874998f, -0.0020727679f, -0.0033338852f, -0.0048267758f, -0.0057047920f, -0.0046121574f,
  +0.0000000000f, +0.0094066633f, +0.0241219388f, +0.0435446244f, +0.0658632997f, +0.0882803947f,
  +0.1075242551f, +0.1205318951f, +0.1251296142f, +0.1205318951f, +0.1075242551f, +0.0882803947f,
  +0.0658632997f, +0.0435446244f, +0.0241219388f, +0.0094066633f, +0.0000000000f, -0.0046121574f,
  -0.0057047920f, -0.0048267758f, -0.0033338852f, -0.0020727679f, -0.0012874998f
};
#define LPF_DELAY   ((DSP_NUM_TAPS - 1U) / 2U)   /* 14 samples */

/* |H(f)| of the FIR, evaluated directly from the coefficients */
static float lpf_gain(float f_hz)
{
  float re = 0.0f, im = 0.0f;
  for (uint32_t k = 0; k < DSP_NUM_TAPS; k++)
  {
    float w = 2.0f * PI * f_hz * (float)k / (float)DSP_FS_HZ;
    re += lpfCoeffs[k] * arm_cos_f32(w);
    im -= lpfCoeffs[k] * arm_sin_f32(w);
  }
  float mag;
  arm_sqrt_f32(re * re + im * im, &mag);
  return mag;
}

/* ---------------- Test 2: known signal through the accelerometer LPF ---------------- */
#define T2_FFT_LEN   256U
#define T2_SKIP      32U                       /* skip the FIR start-up transient (> 28 samples) */
#define T2_LEN       (T2_SKIP + T2_FFT_LEN)    /* 288 = 18 blocks of 16 */
#define T2_F1        1.0f                      /* pass-band tone  -> FFT bin 4  */
#define T2_F2        16.0f                     /* stop-band tone  -> FFT bin 64 */
#define T2_A2        0.5f

static float32_t t2x[T2_LEN];
static float32_t t2y[T2_LEN];
static float32_t t2ideal[T2_LEN];
static float32_t t2state[DSP_BLOCK_SIZE + DSP_NUM_TAPS - 1];
static float32_t fftBuf[2 * T2_FFT_LEN];
static float32_t fftMag[T2_FFT_LEN];

/* amplitude spectrum (single-sided) of 256 real samples */
static void amp_spectrum(const float32_t *x)
{
  for (uint32_t i = 0; i < T2_FFT_LEN; i++)
  {
    fftBuf[2 * i]     = x[i];
    fftBuf[2 * i + 1] = 0.0f;
  }
  arm_cfft_f32(&arm_cfft_sR_f32_len256, fftBuf, 0, 1);
  arm_cmplx_mag_f32(fftBuf, fftMag, T2_FFT_LEN);
  arm_scale_f32(fftMag, 2.0f / (float)T2_FFT_LEN, fftMag, T2_FFT_LEN);
}

int DSP_SelfTest_AccLpf(void)
{
  arm_fir_instance_f32 S;
  char b1[16], b2[16], b3[16], b4[16];
  const uint32_t bin1 = (uint32_t)(T2_F1 * T2_FFT_LEN / DSP_FS_HZ);   /* 4  */
  const uint32_t bin2 = (uint32_t)(T2_F2 * T2_FFT_LEN / DSP_FS_HZ);   /* 64 */

  printf("\r\n========== Test 2: accelerometer LPF with a known signal ==========\r\n");
  printf("filter = fir1(28, 4/32): %u taps, fc = 4 Hz, fs = %u Hz, delay = %u samples\r\n",
         (unsigned)DSP_NUM_TAPS, (unsigned)DSP_FS_HZ, (unsigned)LPF_DELAY);
  printf("input  = sin(2*pi*1*n/64) + 0.5*sin(2*pi*16*n/64), %u samples, block size %u\r\n",
         (unsigned)T2_LEN, (unsigned)DSP_BLOCK_SIZE);

  /* generate the known signal and the ideal output */
  float g1 = lpf_gain(T2_F1), g2 = lpf_gain(T2_F2);
  for (uint32_t n = 0; n < T2_LEN; n++)
  {
    t2x[n] = arm_sin_f32(2.0f * PI * T2_F1 * (float)n / DSP_FS_HZ)
           + T2_A2 * arm_sin_f32(2.0f * PI * T2_F2 * (float)n / DSP_FS_HZ);
    t2ideal[n] = g1 * arm_sin_f32(2.0f * PI * T2_F1 * ((float)n - (float)LPF_DELAY) / DSP_FS_HZ);
  }

  /* filter block by block, exactly like the real-time task */
  arm_fir_init_f32(&S, DSP_NUM_TAPS, (float32_t *)lpfCoeffs, t2state, DSP_BLOCK_SIZE);
  uint32_t t0 = DSP_Cycles();
  for (uint32_t i = 0; i < T2_LEN / DSP_BLOCK_SIZE; i++)
  {
    arm_fir_f32(&S, t2x + i * DSP_BLOCK_SIZE, t2y + i * DSP_BLOCK_SIZE, DSP_BLOCK_SIZE);
  }
  uint32_t cyc = DSP_Cycles() - t0;

  printf("theory: |H(1 Hz)| = %s (%s dB), |H(16 Hz)| = %s (%s dB)\r\n",
         DSP_FloatStr(b1, g1, 4), DSP_FloatStr(b2, to_db(g1), 2),
         DSP_FloatStr(b3, g2, 6), DSP_FloatStr(b4, to_db(g2), 2));

  /* time domain */
  printf("   n      x[n]      y[n]   ideal y[n]\r\n");
  for (uint32_t n = T2_SKIP; n < T2_SKIP + 64; n += 4)
  {
    printf(" %3lu  %8s  %8s  %8s\r\n", (unsigned long)n,
           DSP_FloatStr(b1, t2x[n], 4), DSP_FloatStr(b2, t2y[n], 4), DSP_FloatStr(b3, t2ideal[n], 4));
  }
  float32_t snr = arm_snr_f32(t2ideal + T2_SKIP, t2y + T2_SKIP, T2_FFT_LEN);
  printf("SNR(y vs ideal 1 Hz output, n = %u..%u) = %s dB\r\n",
         (unsigned)T2_SKIP, (unsigned)(T2_LEN - 1), DSP_FloatStr(b1, snr, 2));

  /* frequency domain: 256-point FFT, bin spacing 64/256 = 0.25 Hz */
  float32_t inPeak, outPeak, in1, in2, out1, out2;
  uint32_t inPeakBin, outPeakBin;
  amp_spectrum(t2x + T2_SKIP);
  in1 = fftMag[bin1]; in2 = fftMag[bin2];
  arm_max_f32(fftMag + 1, T2_FFT_LEN / 2 - 1, &inPeak, &inPeakBin); inPeakBin += 1;
  amp_spectrum(t2y + T2_SKIP);
  out1 = fftMag[bin1]; out2 = fftMag[bin2];
  arm_max_f32(fftMag + 1, T2_FFT_LEN / 2 - 1, &outPeak, &outPeakBin); outPeakBin += 1;

  printf("FFT (arm_cfft_f32, N = 256, 0.25 Hz/bin)   input      output    gain\r\n");
  printf("   bin %2lu ( 1 Hz) amplitude             %8s  %8s  %s dB\r\n", (unsigned long)bin1,
         DSP_FloatStr(b1, in1, 4), DSP_FloatStr(b2, out1, 4), DSP_FloatStr(b3, to_db(out1 / in1), 2));
  printf("   bin %2lu (16 Hz) amplitude             %8s  %8s  %s dB\r\n", (unsigned long)bin2,
         DSP_FloatStr(b1, in2, 4), DSP_FloatStr(b2, out2, 6), DSP_FloatStr(b3, to_db(out2 / in2), 2));
  printf("   largest bins: input -> bin %lu (%s Hz), output -> bin %lu (%s Hz)\r\n",
         (unsigned long)inPeakBin, DSP_FloatStr(b1, inPeakBin * (float)DSP_FS_HZ / T2_FFT_LEN, 2),
         (unsigned long)outPeakBin, DSP_FloatStr(b2, outPeakBin * (float)DSP_FS_HZ / T2_FFT_LEN, 2));
  printf("cycles: arm_fir_f32 on %u samples = %lu (%lu per sample)\r\n",
         (unsigned)T2_LEN, (unsigned long)cyc, (unsigned long)(cyc / T2_LEN));

  int pass = (snr > 40.0f) && (to_db(out2 / in2) < -40.0f) && (fabsf(to_db(out1 / in1)) < 0.5f)
          && (outPeakBin == bin1);
  printf("Test 2 result: %s  (need SNR > 40 dB, 16 Hz < -40 dB, 1 Hz within 0.5 dB)\r\n",
         pass ? "SUCCESS" : "FAILURE");
  return pass;
}

/* ---------------- real-time filter, one FIR instance per axis ---------------- */
static arm_fir_instance_f32 accFir[DSP_NUM_AXES];
static float32_t accState[DSP_NUM_AXES][DSP_BLOCK_SIZE + DSP_NUM_TAPS - 1];
static float32_t accIn[DSP_BLOCK_SIZE];
static float32_t accOut[DSP_BLOCK_SIZE];

void DSP_FilterInit(void)
{
  for (uint32_t a = 0; a < DSP_NUM_AXES; a++)
  {
    arm_fir_init_f32(&accFir[a], DSP_NUM_TAPS, (float32_t *)lpfCoeffs, accState[a], DSP_BLOCK_SIZE);
  }
}

uint32_t DSP_FilterBlock(const int16_t in[][DSP_NUM_AXES], int16_t out[][DSP_NUM_AXES])
{
  uint32_t cycles = 0;

  for (uint32_t a = 0; a < DSP_NUM_AXES; a++)
  {
    for (uint32_t n = 0; n < DSP_BLOCK_SIZE; n++)
    {
      accIn[n] = (float32_t)in[n][a];
    }
    uint32_t t0 = DSP_Cycles();
    arm_fir_f32(&accFir[a], accIn, accOut, DSP_BLOCK_SIZE);
    cycles += DSP_Cycles() - t0;
    for (uint32_t n = 0; n < DSP_BLOCK_SIZE; n++)
    {
      out[n][a] = (int16_t)lroundf(accOut[n]);
    }
  }
  return cycles;
}
