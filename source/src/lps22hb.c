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
  HAL_StatusTypeDef status;
  /* First check that we can communicate with the device */
  tmpvalue = I2C_Read8(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_WHO_AM_I);
  if(tmpvalue != LPS22HB_DEVICE_ID) {
    I2C_ErrorHandler("LPS22HB_Configure",dev->i2cAddress,LPS22HB_REG_WHO_AM_I,"Invalid LPS22HB ID value");
    return HAL_ERROR;
  }
  return HAL_OK;
}

/*
 * Reads a register on the LPS22HB and returns the value.
 */
uint8_t LPS22HB_ReadRegister(LPS22HB_Handle_t *dev, LPS22HB_Register_t reg) {
  uint8_t data;
  if (HAL_I2C_Mem_Read(dev->i2cBus, dev->i2cAddress,
                       reg, sizeof(uint8_t), &data, 1, I2C_TIMEOUT) != HAL_OK) {
    Error_Handler();
  }
  return data;
}

/*
 * Writes to a register on the LPS22HB.
 */
void LPS22HB_WriteRegister(LPS22HB_Handle_t *dev, LPS22HB_Register_t reg, uint8_t value) {
  if (HAL_I2C_Mem_Write(dev->i2cBus, dev->i2cAddress,
                        reg, sizeof(uint8_t), &value, 1, I2C_TIMEOUT) != HAL_OK) {
    Error_Handler();
  }
}

/*
 * Reads the reference pressure
 */
int32_t LPS22HB_ReadRefPressure(LPS22HB_Handle_t *dev) {
/* Read the reference pressure from the device casting the data into a signed integer
 * since it uses two's complement */
  int32_t ref=(int32_t)I2C_Read24BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_REF_P_XL);
  return ref;
}

/*
 * Writes the reference pressure
 */
void LPS22HB_SetRefPressure(LPS22HB_Handle_t *dev,int32_t ref) {
  /* Convert to an unsigned int and then mask the first 9 bits to give a 23 bit value. */
  uint32_t val=((uint32_t)ref) & 0x7fffff;
  /* Look at the sign of the reference and, if it is less than zero, set the 24th bit
   * since we are using two's complement */
  if(ref<0) val |= 0x800000;
  /* Write the reference to the device */
  I2C_Write16BE(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_REF_P_XL, val);
}

