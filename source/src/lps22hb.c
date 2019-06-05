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
void LPS22HBInit(void) {
  // Configure the chip
  lps22hb.i2cBus = &I2C_Bus2;
  lps22hb.i2cAddress = LPS22HB_I2C_ADDRESS;
}

/*
 * Configures the ADXL355 chip to the supplied settings
 */
HAL_StatusTypeDef LPS22HBConfigure(LPS22HBHandle_t *dev) {
  uint8_t data[4];
  HAL_StatusTypeDef status;
  /* First check that we can communicate with the device */
  status = HAL_I2C_Mem_Read(dev->i2cBus, dev->i2cAddress, LPS22HB_REG_WHO_AM_I,
                            sizeof(uint8_t), data, 1, 1000);
  if (status != HAL_OK) {
    fprintf(stderr,"LPS22HB: Reading device ID failed\n");
    Error_Handler();
  }
  return HAL_OK;
}

/*
 * Reads a register on the LPS22HB and returns the value.
 */
uint8_t LPS22HBReadRegister(LPS22HBHandle_t *dev, LPS22HBRegister_t reg) {
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
void LPS22HBWriteRegister(LPS22HBHandle_t *dev, LPS22HBRegister_t reg, uint8_t value) {
  if (HAL_I2C_Mem_Write(dev->i2cBus, dev->i2cAddress,
                        reg, sizeof(uint8_t), &value, 1, I2C_TIMEOUT) != HAL_OK) {
    Error_Handler();
  }
}

