//
// Created by RogerMoore on 30/08/2019.
//

/*
 * Implements the code to interface to the LIS3MDL Magnetometer sensor
 */

#include <stdio.h>
#include <limits.h>

#include "lis3mdl.h"
#include "stm32h7xx_hal_def.h"
#include "main.h"

/*
 * Initializes global devices
 */
void LIS3MDL_Initialize(void) {
  /* Configure the chip */
  lis3mdl.i2cBus = &I2C_Bus2;
  lis3mdl.i2cAddress = LIS3MDL_I2C_ADDRESS;
  lis3mdl.range = LIS3MDL_RANGE_4G;
  lis3mdl.enableTemp=true;
  LIS3MDL_Configure(&lis3mdl);
}

/*
 * Configures the LIS3MDL chip to the supplied settings
 */
HAL_StatusTypeDef LIS3MDL_Configure(LIS3MDL_Handle_t *dev) {
  uint8_t tmpvalue;
  /* First check that we can communicate with the device */
  tmpvalue = LIS3MDL_ReadRegister(dev,LIS3MDL_REG_WHO_AM_I);
  if(tmpvalue != LIS3MDL_DEVICE_ID) {
    I2C_ErrorHandler("LIS3MDL_Configure",dev->i2cAddress,LIS3MDL_REG_WHO_AM_I,"Invalid LIS3MDL ID value");
    return HAL_ERROR;
  }
  /* Set the dynamic range */
  LIS3MDL_SetRange(dev,dev->range);
  /* Enable (or disable) the temperature sensor */
  if(dev->enableTemp) {
    LIS3MDL_EnableTemperature(dev);
  } else {
    LIS3MDL_DisableTemperature(dev);
  }
  return HAL_OK;
}

/*
 * Enables the temperature sensor
 */
void LIS3MDL_EnableTemperature(LIS3MDL_Handle_t *dev) {
  /* Read the current value of the register so that we do not change other settings */
  uint8_t value=LIS3MDL_ReadRegister(dev,LIS3MDL_REG_CTRL_REG1);
  /* Set the temperature mode bit (bit 7)*/
  value |= 0x80;
  /* Write the new value back to the device */
  LIS3MDL_WriteRegister(dev,LIS3MDL_REG_CTRL_REG1,value);
  /* Update the state of the device handler */
  dev->enableTemp = true;
}

/*
 * Disables the temperature sensor
 */
void LIS3MDL_DisableTemperature(LIS3MDL_Handle_t *dev) {
  /* Read the current value of the register so that we do not change other settings */
  uint8_t value=LIS3MDL_ReadRegister(dev,LIS3MDL_REG_CTRL_REG1);
  /* Clear the temperature mode bit (bit 7) */
  value &= 0x7f;
  /* Write the new value back to the device */
  LIS3MDL_WriteRegister(dev,LIS3MDL_REG_CTRL_REG1,value);
  /* Update the state of the device handler */
  dev->enableTemp = false;
}

/*
 * Sets the LIS3MDL's dynamic range.
 */
void LIS3MDL_SetRange(LIS3MDL_Handle_t *dev, LIS3MDL_Range_t range) {
  /* Only the range bits should be set, everything else must be zero, because this register
   * contains the range and two reset bits */
  uint8_t value=(range & 0x03) << 5;
  /* Write the value to the device */
  LIS3MDL_WriteRegister(dev,LIS3MDL_REG_CTRL_REG2,value);
  /* Update the state of the device handler */
  dev->range = range;
}

/*
 * Reads the calibrated x component magnetic field data and returns
 * a value in SI units of tesla.
 */
float_t LIS3MDL_XBField(LIS3MDL_Handle_t *dev) {
  int16_t raw=LIS3MDL_RawXBField(dev);
  /* Calculate the value in tesla by first dividing by the maximum 16-bit value (USHRT_MAX)
   * to get the fraction of full scale observed, then multiplying by the full-scale value
   * in gauss and finally converting from gauss to tesla.
   */
  float_t value=((float_t)raw/USHRT_MAX)*(dev->range+1)*4*1e-4;
  return value;
}

/*
 * Reads the calibrated x component magnetic field data and returns
 * a value in SI units of tesla.
 */
float_t LIS3MDL_YBField(LIS3MDL_Handle_t *dev) {
  int16_t raw=LIS3MDL_RawYBField(dev);
  /* Calculate the value in tesla by first dividing by the maximum 16-bit value (USHRT_MAX)
   * to get the fraction of full scale observed, then multiplying by the full-scale value
   * in gauss and finally converting from gauss to tesla.
   */
  float_t value=((float_t)raw/USHRT_MAX)*(dev->range+1)*4*1e-4;
  return value;
}

/*
 * Reads the calibrated x component magnetic field data and returns
 * a value in SI units of tesla.
 */
float_t LIS3MDL_ZBField(LIS3MDL_Handle_t *dev) {
  int16_t raw=LIS3MDL_RawZBField(dev);
  /* Calculate the value in tesla by first dividing by the maximum 16-bit value (USHRT_MAX)
   * to get the fraction of full scale observed, then multiplying by the full-scale value
   * in gauss and finally converting from gauss to tesla.
   */
  float_t value=((float_t)raw/USHRT_MAX)*(dev->range+1)*4*1e-4;
  return value;
}

