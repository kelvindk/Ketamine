#include "TCS3414CS.h"

void HalColorInit(uint8 addr){
    setTimingReg(addr, INTEG_MODE_FREE);//Set trigger mode.Including free mode,manually mode,single synchronizition mode or so.
    setInterruptSourceReg(addr, INT_SOURCE_CLEAR); //Set interrupt source
    setInterruptControlReg(addr, INTR_LEVEL|INTR_PERSIST_EVERY);//Set interrupt mode
    setGain(addr, GAIN_1|PRESCALER_4);//Set gain value and prescaler value
    setEnableADC(addr);//Start ADC of the color sensor
}
void setReadReg(uint8 addr){
    HalI2CInit(addr, i2cClock_267KHZ);
    HalSensorWriteReg(REG_BLOCK_READ, NULL, 0);
}
struct RGBC ReadRGB(uint8 addr){
    struct RGBC color;
    uint8 buf[20];
    
    //HalI2CInit(addr, i2cClock_267KHZ);
    //HalSensorWriteReg(REG_BLOCK_READ, NULL, 0);
    
    HalI2CInit(addr, i2cClock_267KHZ);
    HalSensorReadReg(addr, buf, 8);
    
    color.green = buf[1]*256 + buf[0];
    color.red = buf[3]*256 + buf[2];
    color.blue = buf[5]*256 + buf[4];
    color.clear = buf[7]*256 + buf[6];
    
    setDisableColor(addr);
    
    return color;
}
void calculateCoordinate(uint8 addr);

// Helper functions
void setTimingReg(uint8 addr, uint8 x){
    HalI2CInit(addr, i2cClock_267KHZ);
    HalSensorWriteReg(REG_TIMING, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    //ST_HAL_DELAY(180);
}
void setInterruptSourceReg(uint8 addr, uint8 x){
    HalI2CInit(addr, i2cClock_267KHZ);
    HalSensorWriteReg(REG_INT_SOURCE, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    //ST_HAL_DELAY(180);
}
void setInterruptControlReg(uint8 addr, uint8 x){
    HalI2CInit(addr, i2cClock_267KHZ);
    HalSensorWriteReg(REG_INT, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    //ST_HAL_DELAY(180);
}
void setGain(uint8 addr, uint8 x){
    HalI2CInit(addr, i2cClock_267KHZ);
    HalSensorWriteReg(REG_GAIN, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    //ST_HAL_DELAY(180);
}
void setEnableADC(uint8 addr){
    HalI2CInit(addr, i2cClock_267KHZ);
    uint8 instruction = CTL_DAT_INIITIATE;
    HalSensorWriteReg(REG_CTL, &instruction, 1);
    // Wait for measurement ready (appx. 1.45 ms)
    //ST_HAL_DELAY(180);
}
void setDisableColor(uint8 addr){
    HalI2CInit(addr, i2cClock_267KHZ);
    uint8 instruction = 0;
    HalSensorWriteReg(REG_CTL, &instruction, 1);
}
void clearInterrupt(uint8 addr){
    HalI2CInit(addr, i2cClock_267KHZ);
    HalSensorWriteReg(CLR_INT, NULL, 0);
    // Wait for measurement ready (appx. 1.45 ms)
    //ST_HAL_DELAY(180);
}
