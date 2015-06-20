#include "hal_i2c.h"
#include "hal_sensor.h"

#define EEPROM_ADDR 0x50

void i2c_eeprom_write_byte(uint8 deviceaddress, uint8 eeaddress, uint8 data);
void i2c_eeprom_write_page(uint8 deviceaddress, uint8 eeaddresspage, uint8* data, uint8 length);
uint8 i2c_eeprom_read_byte(uint8 deviceaddress, uint8 eeaddress);
bool i2c_eeprom_read_buffer(uint8 deviceaddress, uint8 eeaddress, uint8* buffer, uint8 length);
