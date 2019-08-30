//
// Created by RogerMoore on 29/05/2019.
//

/*
 * Implements the code to interface to the LPS22HB pressure sensor
 */

#include <stdio.h>

#include "lps22hb.h"
#include "stm32h7xx_hal_def.h"
#include "main.h"


/*
 * Initializes global devices
 */
void LPS22HB_Initialize(void) {
  /* Configure the chip */
  lps22hb.i2cBus = &I2C_Bus2;
  lps22hb.i2cAddress = LPS22HB_I2C_ADDRESS;
  LPS22HB_Configure(&lps22hb);
}

/*
 * Configures the LPS22HB chip to the supplied settings
 */
HAL_StatusTypeDef LPS22HB_Configure(LPS22HB_Handle_t *dev) {
  uint8_t tmpvalue;
  /* First check that we can communicate with the device */
  tmpvalue = I2C_Read8(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_WHO_AM_I);
  if(tmpvalue != LPS22HB_DEVICE_ID) {
    I2C_ErrorHandler("LPS22HB_Configure",dev->i2cAddress,LPS22HB_REG_WHO_AM_I,"Invalid LPS22HB ID value");
    return HAL_ERROR;
  }
  return HAL_OK;
}

/*
 * Reads the reference pressure.
 * Note this is only used when the AUTOZERO or AUTORIFP modes are enabled.
 */
int32_t LPS22HB_ReadRefPressure(LPS22HB_Handle_t *dev) {
  /* Read the reference pressure from the device casting the data into a signed integer
   * since it uses two's complement */
  int32_t ref=(int32_t)I2C_Read24BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_REF_P_XL);
  return ref;
}

/*
 * Writes the reference pressure.
 * Note this is only used when the AUTOZERO or AUTORIFP modes are enabled.
 */
void LPS22HB_SetRefPressure(LPS22HB_Handle_t *dev,int32_t ref) {
  /* Convert to an unsigned int and then mask the first 9 bits to give a 24 bit value. */
  uint32_t val=((uint32_t)ref) & 0x7fffff;
  /* Look at the sign of the reference and, if it is less than zero, set the 24th bit
   * since we are using two's complement */
  if(ref<0) val |= 0x800000;
  /* Write the reference to the device */
  I2C_Write24BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_REF_P_XL, val);
}

/*
 * Reads the current pressure value.
 * This method will read and return the current pressure value. If AUTOZERO mode is enabled
 * this value will have the reference pressure subtract from it and if AUTORIFP mode is
 * enabled it will have the offset pressure subtracted from it.
 */
int32_t LPS22HB_ReadPressure(LPS22HB_Handle_t *dev) {
  /* Read the pressure from the device casting the data into a signed integer since it uses two's complement */
  int32_t pressure=(int32_t)I2C_Read24BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_PRESS_OUT_XL);
  return pressure;
}

/*
 * Sets the offset pressure.
 * If the AUTORIFP mode is set this is subtracted from the measured pressure when a readout is requested. The
 * manual is unclear whether this 16-bit value corresponds to the least or most significant 16-bits of the
 * readout pressure but this code currently assumes LSB.
 */
void LPS22HB_SetOffsetPressure(LPS22HB_Handle_t *dev, int32_t value) {
  /* Convert to an unsigned int and then mask the first 16 bits to give a 16 bit value. */
  uint16_t val=((uint32_t)value) & 0x7fff;
  /* Look at the sign of the value and, if it is less than zero, set the 16th bit
   * since we are using two's complement */
  if(value<0) val |= 0x8000;
  /* Write the offset to the device */
  I2C_Write16BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_RPDS_L, val);
}

/*
 * Reads the offset pressure.
 * If the AUTORIFP mode is set this is subtracted from the measured pressure when a readout is requested. The
 * manual is unclear whether this 16-bit value corresponds to the least or most significant 16-bits of the
 * readout pressure but this code currently assumes LSB.
 */
int32_t LPS22HB_ReadOffsetPressure(LPS22HB_Handle_t *dev) {
  /* Read the offset pressure from the device casting the data into a signed integer
   * since it uses two's complement */
  int32_t offset=(int32_t)I2C_Read16BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_RPDS_L);
  return offset;
}

/*
 * Turns on the AUTOZERO mode of the device.
 * This method will set the device into its AUTOZERO mode. The device will measure the correct pressure
 * and set the reference pressure registers to this value. It will then subtract this value from the
 * pressure readout sensors before returning the value.
 */
void LPS22HB_AutoZeroOn(LPS22HB_Handle_t *dev) {
  /* Check to see if anything needs to be done. */
  if(dev->autoZeroMode) {
    I2C_ErrorHandler("LPS22HB_AutoZeroOn",dev->i2cAddress,LPS22HB_REG_INTERRUPT_CFG,"Autozero already enabled");
    return;
  }
  /* Set the bit in the INTERRUPT_CFG register to enable AUTOZERO. This will cause the device to read the current
   * pressure and use it as the reference pressure.
   * We do this by first reading the register, then setting the correct bit and then writing the new value
   * back to the register. The set bit autoclears once the mode is enabled. */
  uint8_t regval=LPS22HB_ReadRegister(dev,LPS22HB_REG_INTERRUPT_CFG);
  regval=regval | (1 << 5);
  LPS22HB_WriteRegister(dev,LPS22HB_REG_INTERRUPT_CFG,regval);
  dev->autoZeroMode=true;
}

/*
 * Turns off the AUTOZERO mode of the device.
 */
void LPS22HB_AutoZeroOff(LPS22HB_Handle_t *dev) {
  /* Check to see if anything needs to be done. */
  if(!dev->autoZeroMode) {
    I2C_ErrorHandler("LPS22HB_AutoZeroOff", dev->i2cAddress, LPS22HB_REG_INTERRUPT_CFG, "Autozero already disabled");
    return;
  }
  /* Set the bit in the INTERRUPT_CFG register to disable AUTOZERO.
   * We do this by first reading the register, then setting the correct bit and then writing the new value
   * back to the register. The set bit autoclears once the mode is disabled. */
  uint8_t regval=LPS22HB_ReadRegister(dev,LPS22HB_REG_INTERRUPT_CFG);
  regval=regval | (1 << 4);
  LPS22HB_WriteRegister(dev,LPS22HB_REG_INTERRUPT_CFG,regval);
  dev->autoZeroMode=false;
}

/*
 * Reads the temperature.
 */
int16_t LPS22HB_ReadTemperature(LPS22HB_Handle_t *dev) {
  /* Read the temperature from the device casting the data into a signed integer
   * since it uses two's complement */
  int16_t temp=(int16_t)I2C_Read16BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_TEMP_OUT_L);
  return temp;
}

