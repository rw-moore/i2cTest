//
// Created by RogerMoore on 24/05/2019.
//

/*
 * Implements the code to interface to the ADXL355 Accelerometer
 */

#include <stdio.h>

#include "adxl355.h"
#include "stm32h7xx_hal_def.h"
#include "main.h"


/*
 * Initializes global devices
 */
void ADXL355_Initialize(void) {
  // Configure the chip
  adxl355.i2cBus = &I2C_Bus2;
  adxl355.i2cAddress = ADXL355_I2C_ADDRESS << 1;
  adxl355.range = ADXL355_RANGE_2G;
  adxl355.standby = true;
  adxl355.enableTemp = true;
  ADXL355_Configure(&adxl355);
}

/*
 * Configures the ADXL355 chip to the supplied settings
 */
HAL_StatusTypeDef ADXL355_Configure(ADXL355_Handle_t *dev) {
  uint8_t tmpvalue;
  /* First check that we can communicate with the device by checking its ID value */
  tmpvalue=I2C_Read8(dev->i2cBus, dev->i2cAddress, ADXL355_REG_DEVID_AD);
  if(tmpvalue != ADXL355_DEVICE_ID) {
    I2C_ErrorHandler("ADXL355Configure",dev->i2cAddress,ADXL355_REG_DEVID_AD,"Invalid ADXL355 ID value");
    return HAL_ERROR;
  }
  /* Now that communication has been established reset the device and then put it into standby mode
   * so that we can configure it. */
  ADXL355_Reset(dev);
  ADXL355_StandbyOn(dev);
  /* Set the dynamic range of the device */
  ADXL355_SetRange(dev,dev->range);
  /* If the device handle indicates that standby mode is not enabled then turn it off now that
   * we have finished configuring the device. */
  if(!dev->standby) ADXL355_StandbyOff(dev);
  return HAL_OK;
}

/*
 * Puts the ADXL355 into standby mode for configuration.
 */
void ADXL355_StandbyOn(ADXL355_Handle_t *dev) {
  /* Read the current value of the register so that we do not change other settings */
  uint8_t value=ADXL355_ReadRegister(dev,ADXL355_REG_POWER_CTL);
  /* Set the standby mode bit and ensure all reserved bits are masked out "just in case" */
  value = (value & 0x07) | 1;
  /* Write the new value back to the device */
  ADXL355_WriteRegister(dev,ADXL355_REG_POWER_CTL,value);
  /* Update the state of the device handler */
  dev->standby = true;
}

/*
 * Puts the ADXL355 out of standby mode for data collection.
 */
void ADXL355_StandbyOff(ADXL355_Handle_t *dev) {
  /* Read the current value of the register so that we do not change other settings */
  uint8_t value=ADXL355_ReadRegister(dev,ADXL355_REG_POWER_CTL);
  /* Clear the standby mode bit and ensure all reserved bits are also cleared "just in case" */
  value &= 0x06;
  /* Write the new value back to the device */
  ADXL355_WriteRegister(dev,ADXL355_REG_POWER_CTL,value);
  /* Update the state of the device handler */
  dev->standby = false;
}

/*
 * Sets the ADXL355's dynamic range.
 */
void ADXL355_SetRange(ADXL355_Handle_t *dev, ADXL355_Range_t range) {
  /* Check that we are in standby mode, if not then set the device into standby */
  if(!dev->standby) ADXL355_StandbyOn(dev);
  /* Read the current value of the register so that we do not change other settings */
  uint8_t value=ADXL355_ReadRegister(dev,ADXL355_REG_RANGE);
  /* Clear the range and reserved bits and then OR the given range value. */
  value = (value & 0xc0) | range;
  /* Write the new value back to the device */
  ADXL355_WriteRegister(dev,ADXL355_REG_RANGE,value);
  /* Update the state of the device handler */
  dev->range = range;
  /* If we are not supposed to be in standby mode then turn it off again */
  if(!dev->standby) ADXL355_StandbyOff(dev);

}

/*
 * Reads the current, raw temperature data.
 */
uint16_t ADXL355_RawTemperature(ADXL355_Handle_t *dev) {
  /* Read the raw temperature from the device */
  uint16_t temp=I2C_Read16(dev->i2cBus, dev->i2cAddress, ADXL355_REG_TEMP2);
  /* Mask out the 4 reserved bits */
  return temp & 0x0fff;
}

/*
 * Reads the x component acceleration data.
 */
int32_t ADXL355_XAcceleration(ADXL355_Handle_t *dev) {
  /* Read the acceleration data from the device */
  int32_t accel=(int32_t)I2C_Read24(dev->i2cBus, dev->i2cAddress, ADXL355_REG_XDATA3);
  /* Right shift by 4 bits to get rid of the reserved bits but use division since we have
   * a signed integer and bitwise operations on these are undefined */
  return accel / (1 << 4);
}

/*
 * Reads the y component acceleration data.
 */
int32_t ADXL355_YAcceleration(ADXL355_Handle_t *dev) {
  /* Read the acceleration data from the device */
  int32_t accel=(int32_t)I2C_Read24(dev->i2cBus, dev->i2cAddress, ADXL355_REG_YDATA3);
  /* Right shift by 4 bits to get rid of the reserved bits but use division since we have
   * a signed integer and bitwise operations on these are undefined */
  return accel / (1 << 4);
}

/*
 * Reads the z component acceleration data.
 */
int32_t ADXL355_ZAcceleration(ADXL355_Handle_t *dev) {
  /* Read the acceleration data from the device */
  int32_t accel=(int32_t)I2C_Read24(dev->i2cBus, dev->i2cAddress, ADXL355_REG_ZDATA3);
  /* Right shift by 4 bits to get rid of the reserved bits but use division since we have
   * a signed integer and bitwise operations on these are undefined */
  return accel / (1 << 4);
}

/*
 * Reads the x trim value which is subtracted from the X acceleration data.
 * The 16 bits read correspond to the most significant 16 bits of the acceleration data and this
 * function shifts them to give a trim value consistent with the acceleration data. Note that the
 * value is signed.
 */
int32_t ADXL355_XTrim(ADXL355_Handle_t *dev) {
  /* Read the trim data from the device */
  int16_t trim=(int16_t)I2C_Read16(dev->i2cBus, dev->i2cAddress, ADXL355_REG_OFFSET_X_H);
  /* Left shift by 4 bits so the return value looks the same as the acceleration data,
   * but use multiplication since bitwise operators on signed ints is undefined */
  return ((int32_t) trim) * (1 << 4);
}

/*
 * Reads the x trim value which is subtracted from the X acceleration data.
 * The 16 bits read correspond to the most significant 16 bits of the acceleration data and this
 * function shifts them to give a trim value consistent with the acceleration data. Note that the
 * value is signed.
 */
int32_t ADXL355_YTrim(ADXL355_Handle_t *dev) {
  /* Read the trim data from the device */
  int16_t trim=(int16_t)I2C_Read16(dev->i2cBus, dev->i2cAddress, ADXL355_REG_OFFSET_Y_H);
  /* Left shift by 4 bits so the return value looks the same as the acceleration data,
   * but use multiplication since bitwise operators on signed ints is undefined */
  return ((int32_t) trim) * (1 << 4);
}

/*
 * Reads the x trim value which is subtracted from the X acceleration data.
 * The 16 bits read correspond to the most significant 16 bits of the acceleration data and this
 * function shifts them to give a trim value consistent with the acceleration data. Note that the
 * value is signed.
 */
int32_t ADXL355_ZTrim(ADXL355_Handle_t *dev) {
  /* Read the trim data from the device */
  int16_t trim=(int16_t)I2C_Read16(dev->i2cBus, dev->i2cAddress, ADXL355_REG_OFFSET_Z_H);
  /* Left shift by 4 bits so the return value looks the same as the acceleration data,
   * but use multiplication since bitwise operators on signed ints is undefined */
  return ((int32_t) trim) * (1 << 4);
}

/*
 * Sets the x trim value which is subtracted from the X acceleration data.
 * The chip only subtracts trim from the 16 MSBs so this function masks out the 4 LSBs from the
 * value given to give the 16 MSBs which are then written to the chip.
 */
void ADXL355_SetXTrim(ADXL355_Handle_t *dev,int32_t value) {
  /* Convert to unsigned int so we can use bitwise operators to right shift by 4 bits and mask
   * out all but the least significant 16 bits. */
  uint32_t val=(((uint32_t)value) >> 4) & 0xffff;
  /* Write the trim data to the device */
  I2C_Write16(dev->i2cBus, dev->i2cAddress, ADXL355_REG_OFFSET_X_H, (uint16_t)val);
}

/*
 * Sets the y trim value which is subtracted from the X acceleration data.
 * The chip only subtracts trim from the 16 MSBs so this function masks out the 4 LSBs from the
 * value given to give the 16 MSBs which are then written to the chip.
 */
void ADXL355_SetYTrim(ADXL355_Handle_t *dev,int32_t value) {
  /* Convert to unsigned int so we can use bitwise operators to right shift by 4 bits and mask
   * out all but the least significant 16 bits. */
  uint32_t val=(((uint32_t)value) >> 4) & 0xffff;
  /* Write the trim data to the device */
  I2C_Write16(dev->i2cBus, dev->i2cAddress, ADXL355_REG_OFFSET_Y_H, (uint16_t)val);
}

/*
 * Sets the z trim value which is subtracted from the X acceleration data.
 * The chip only subtracts trim from the 16 MSBs so this function masks out the 4 LSBs from the
 * value given to give the 16 MSBs which are then written to the chip.
 */
void ADXL355_SetZTrim(ADXL355_Handle_t *dev,int32_t value) {
  /* Convert to unsigned int so we can use bitwise operators to right shift by 4 bits and mask
   * out all but the least significant 16 bits. */
  uint32_t val=(((uint32_t)value) >> 4) & 0xffff;
  /* Write the trim data to the device */
  I2C_Write16(dev->i2cBus, dev->i2cAddress, ADXL355_REG_OFFSET_Z_H, (uint16_t)val);
}



