/**
  ******************************************************************************
  * @file    dsp_service.c
  * @brief   HW-Lab7 custom GATT service (see dsp_service.h)
  ******************************************************************************
  */
#include <stdio.h>
#include "dsp_service.h"
#include "bluenrg_def.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_aci_const.h"

#define DSP_DATA_LEN  14U

/* UUIDs written in the usual string order (MSB first); the stack wants LSB first */
static const uint8_t DSP_SERVICE_UUID[16] = {0xa0,0x07,0x00,0x00, 0x8e,0x22, 0x45,0x41, 0x9d,0x4c, 0x21,0xed,0xae,0x82,0xed,0x19};
static const uint8_t DSP_DATA_UUID[16]    = {0xa0,0x07,0x00,0x01, 0x8e,0x22, 0x45,0x41, 0x9d,0x4c, 0x21,0xed,0xae,0x82,0xed,0x19};

static uint16_t dspServHandle;
static uint16_t dspDataCharHandle;
static volatile uint8_t dspNotifyEnabled = 0;

static void uuid_to_le(const uint8_t be[16], uint8_t le[16])
{
  for (int i = 0; i < 16; i++)
  {
    le[i] = be[15 - i];
  }
}

uint8_t DspService_Add(void)
{
  uint8_t uuid[16];
  tBleStatus ret;

  /* 1 service + (declaration, value, CCCD) */
  uuid_to_le(DSP_SERVICE_UUID, uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 1 + 3, &dspServHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("DspService: add service failed 0x%02x\r\n", ret);
    return ret;
  }

  uuid_to_le(DSP_DATA_UUID, uuid);
  ret = aci_gatt_add_char(dspServHandle, UUID_TYPE_128, uuid, DSP_DATA_LEN,
                          CHAR_PROP_READ | CHAR_PROP_NOTIFY,
                          ATTR_PERMISSION_NONE, GATT_DONT_NOTIFY_EVENTS,
                          16, 0, &dspDataCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf("DspService: add DATA char failed 0x%02x\r\n", ret);
    return ret;
  }

  printf("DspService added (DATA value handle %u)\r\n", dspDataCharHandle + 1);
  return BLE_STATUS_SUCCESS;
}

uint8_t DspService_Notify(uint16_t seq, const int16_t raw[3], const int16_t filt[3])
{
  uint8_t buf[DSP_DATA_LEN];

  buf[0] = (uint8_t)seq;
  buf[1] = (uint8_t)(seq >> 8);
  for (int i = 0; i < 3; i++)
  {
    buf[2 + 2 * i]     = (uint8_t)(raw[i]);
    buf[3 + 2 * i]     = (uint8_t)(raw[i] >> 8);
    buf[8 + 2 * i]     = (uint8_t)(filt[i]);
    buf[9 + 2 * i]     = (uint8_t)(filt[i] >> 8);
  }
  return aci_gatt_update_char_value(dspServHandle, dspDataCharHandle, 0, sizeof(buf), buf);
}

uint8_t DspService_NotifyEnabled(void)
{
  return dspNotifyEnabled;
}

void DspService_AttributeModified(uint16_t attr_handle, uint8_t data_length, const uint8_t *att_data)
{
  (void)data_length;
  if (attr_handle == dspDataCharHandle + 2)
  {
    /* Client Characteristic Configuration Descriptor */
    dspNotifyEnabled = (att_data[0] & 0x01) ? 1 : 0;
    printf("DSP notifications %s\r\n", dspNotifyEnabled ? "enabled" : "disabled");
  }
}

void DspService_Disconnected(void)
{
  dspNotifyEnabled = 0;
}
