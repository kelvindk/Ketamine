#include <TCS3414CS.h>

void HalColorInit(void){
	setTimingReg(INTEG_MODE_FREE);//Set trigger mode.Including free mode,manually mode,single synchronizition mode or so.
    setInterruptSourceReg(INT_SOURCE_CLEAR); //Set interrupt source
    setInterruptControlReg(INTR_LEVEL|INTR_PERSIST_EVERY);//Set interrupt mode
    setGain(GAIN_1|PRESCALER_4);//Set gain value and prescaler value
    setEnableADC();//Start ADC of the color sensor
}
struct RGBC ReadRGB(void){
    struct RGBC color;
    uint8 buf[20];
    
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    HalSensorWriteReg(REG_BLOCK_READ, NULL, 0);
    
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    HalSensorReadReg(COLOR_SENSOR_ADDR, buf, 8);

    color.green = buf[1]*256 + buf[0];
    color.red = buf[3]*256 + buf[2];
    color.blue = buf[5]*256 + buf[4];
    color.clear = buf[7]*256 + buf[6];
    
    return color;
}
void calculateCoordinate(void);

// Helper functions

void setTimingReg(uint8 x){
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    HalSensorWriteReg(REG_TIMING, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);
}
void setInterruptSourceReg(uint8 x){
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    HalSensorWriteReg(REG_INT_SOURCE, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);
}
void setInterruptControlReg(uint8 x){
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    HalSensorWriteReg(REG_INT, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);
}
void setGain(uint8 x){
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    HalSensorWriteReg(REG_GAIN, &x, sizeof(x));
    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);
}
void setEnableADC(void){
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    uint8 instruction = CTL_DAT_INIITIATE;
    HalSensorWriteReg(REG_CTL, &instruction, 1);
    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);
}
void clearInterrupt(void){
    HalI2CInit(COLOR_SENSOR_ADDR, i2cClock_267KHZ);
    HalSensorWriteReg(CLR_INT, NULL, 0);
    // Wait for measurement ready (appx. 1.45 ms)
    ST_HAL_DELAY(180);
}
