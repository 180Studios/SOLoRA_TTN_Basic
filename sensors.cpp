// LIS3DETR Accelerometer and LM75 temperature sensor drivers
#include "sensors.h"
#include <Wire.h>


static uint8_t readI2Creg(uint8_t i2caddr, uint8_t reg) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom(i2caddr, 1);

  return Wire.read();
 
}

static void writeI2Creg(uint8_t i2caddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)val);
  Wire.endTransmission(); 
}


float getTemp(void){
    float temperature;

    uint16_t val;

    // send the device address then the register pointer byte
    Wire.beginTransmission(LM75_I2CADDR);
    Wire.write(LM75_TEMP);
    Wire.endTransmission(false);

    // resend device address then get the 2 returned bytes
    Wire.requestFrom(LM75_I2CADDR, (uint8_t)2);

    // data is returned as 2 bytes big endian
    val = Wire.read() << 8;
    val |= Wire.read();

    temperature = (float)val * 0.125/32.0;

    return temperature;
}




void getAccel(int8_t *accelVal) {

    accelVal[0] = (int8_t)readI2Creg(LIS3DH_ADDRESS, LIS3DH_REG_OUT_X);
    accelVal[1] = (int8_t)readI2Creg(LIS3DH_ADDRESS, LIS3DH_REG_OUT_Y);
    accelVal[2] = (int8_t)readI2Creg(LIS3DH_ADDRESS, LIS3DH_REG_OUT_Z);
      
}

void initTemp(void) {


}

void initAccel(void) {

   // set data rate
  uint8_t ctl1 = readI2Creg(LIS3DH_ADDRESS,LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (1 << 4); // 1Hz
  writeI2Creg(LIS3DH_ADDRESS,LIS3DH_REG_CTRL1, ctl1);


  // DRDY on INT1
//  writeI2Creg(LIS3DH_ADDRESS, LIS3DH_REG_CTRL3, 0x10);


}
