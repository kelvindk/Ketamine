#include "eeprom.h"

void i2c_eeprom_write_byte(uint8 deviceaddress, uint8 eeaddress, uint8 data){
    HalI2CInit(deviceaddress, i2cClock_267KHZ);
    HalSensorWriteReg(eeaddress, &data, sizeof(data));
    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);
}
void i2c_eeprom_write_page(uint8 deviceaddress, uint8 eeaddresspage, uint8* data, uint8 length){
    HalI2CInit(deviceaddress, i2cClock_267KHZ);
    HalSensorWriteReg(eeaddresspage, data, length);
    // Wait for measurement ready (appx. 14.5 ms)
    ST_HAL_DELAY(1800);
}
uint8 i2c_eeprom_read_byte(uint8 deviceaddress, uint8 eeaddress){
    HalI2CInit(deviceaddress, i2cClock_267KHZ);
    HalSensorWriteReg(eeaddress, NULL, 0);

    uint8 buf;
    HalI2CInit(deviceaddress, i2cClock_267KHZ);
    HalSensorReadReg(eeaddress, &buf, 1);

    return buf;
}
bool i2c_eeprom_read_buffer(uint8 deviceaddress, uint8 eeaddresspage, uint8* buffer, uint8 length){
    HalI2CInit(deviceaddress, i2cClock_267KHZ);
    HalSensorWriteReg(eeaddresspage, NULL, 0);

    HalI2CInit(deviceaddress, i2cClock_267KHZ);
    return HalSensorReadReg(eeaddresspage, buffer, length);
}
