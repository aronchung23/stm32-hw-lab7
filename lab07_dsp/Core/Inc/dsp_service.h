/**
  ******************************************************************************
  * @file    dsp_service.h
  * @brief   HW-Lab7 custom GATT service: accelerometer data before / after the
  *          CMSIS-DSP FIR low-pass filter
  *
  *  Service      a0070000-8e22-4541-9d4c-21edae82ed19
  *   DATA char   a0070001-...   READ | NOTIFY   14 bytes, little endian:
  *                 uint16 seq, int16 raw_x, raw_y, raw_z, int16 filt_x, filt_y, filt_z  [mg]
  ******************************************************************************
  */
#ifndef DSP_SERVICE_H
#define DSP_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Add the service and its characteristic to the GATT database. Returns 0 on success. */
uint8_t DspService_Add(void);

/* Send one sample (raw + filtered) as a notification. Returns the BlueNRG status (0 = OK). */
uint8_t DspService_Notify(uint16_t seq, const int16_t raw[3], const int16_t filt[3]);

/* 1 if the client has enabled notifications on the DATA characteristic */
uint8_t DspService_NotifyEnabled(void);

/* Called from user_notify() (sensor.c) on EVT_BLUE_GATT_ATTRIBUTE_MODIFIED / disconnection */
void DspService_AttributeModified(uint16_t attr_handle, uint8_t data_length, const uint8_t *att_data);
void DspService_Disconnected(void);

#ifdef __cplusplus
}
#endif

#endif /* DSP_SERVICE_H */
