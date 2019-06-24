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
void ADXL355Init(void) {
  // Configure the chip
  adxl355.i2cBus = &I2C_Bus2;
  adxl355.i2cAddress = ADXL355_I2C_ADDRESS;
  adxl355.range = ADXL355_RANGE_2G;
  adxl355.standby = false;
  adxl355.enableTemp = true;
}

/*
 * Configures the ADXL355 chip to the supplied settings
 */
HAL_StatusTypeDef ADXL355Configure(ADXL355Handle_t *dev) {
  uint8_t data[4];
  HAL_StatusTypeDef status;
  /* First check that we can communicate with the device */
  status = HAL_I2C_Mem_Read(dev->i2cBus, dev->i2cAddress, ADXL355_REG_DEVID_AD,
          sizeof(uint8_t), data, 1, 1000);
  HAL_I2C_IsDeviceReady(dev->i2cBus, dev->i2cAddress,1,1000);
  if (status != HAL_OK) {
    fprintf(stderr,"ADXL355: Reading device ID failed\n");
    Error_Handler();
  }
  return HAL_OK;
}

/*
 * Reads a register on the ADXL355 and returns the value.
 */
uint8_t ADXL355ReadRegister(ADXL355Handle_t *dev, ADXL355Register_t reg) {
  uint8_t data;
  if (HAL_I2C_Mem_Read(dev->i2cBus, dev->i2cAddress,
          reg, sizeof(uint8_t), &data, 1, I2C_TIMEOUT) != HAL_OK) {
    Error_Handler();
  }
  return data;
}

/*
 * Writes to a register on the ADXL355.
 */
void ADXL355WriteRegister(ADXL355Handle_t *dev, ADXL355Register_t reg, uint8_t value) {
  if (HAL_I2C_Mem_Write(dev->i2cBus, dev->i2cAddress,
                        reg, sizeof(uint8_t), &value, 1, I2C_TIMEOUT) != HAL_OK) {
    Error_Handler();
  }
}

