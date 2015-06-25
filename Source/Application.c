#include "Application.h"

#include "att.h"
#include "hal_i2c.h"
#include "hal_sensor.h"
#include "hal_uart.h"
#include "npi.h"
#include "TCS3414CS.h"
#include "eeprom.h"

static uint8 somedata1[] =
{
  0x61,   // 'a'
  0x70,   // 'p'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
};

bool writeTestPaperId(uint8* buf, uint8 length){
  i2c_eeprom_write_page(EEPROM_ADDR, 0, buf, length);
  ST_HAL_DELAY(125);
  //HalI2CDisable();
}

bool sendReadBuf(attHandleValueNoti_t* ptrNoti, uint8* buf, uint8 length, uint8 preamble){
  int i;
  ptrNoti->handle = 0x2E;
  ptrNoti->len = length + 1;
  ptrNoti->value[0] = preamble;
  for(i = 0; i < length; i++){
    ptrNoti->value[i+1] = buf[i];
  }
  GATT_Notification(0, ptrNoti, FALSE);
}