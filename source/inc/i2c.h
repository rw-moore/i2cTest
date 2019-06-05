//
// Created by RogerMoore on 24/05/2019.
//

#ifndef I2CTEST_I2C_H
#define I2CTEST_I2C_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_144.h"

/* Defines a default time out value for I2C read commands (units are ms) */
#define I2C_TIMEOUT 1000

typedef uint16_t I2CAddress_t;

// Structures to contain I2C bus configuration data
I2C_HandleTypeDef I2C_Bus1;
I2C_HandleTypeDef I2C_Bus2;
I2C_HandleTypeDef I2C_Bus3;
I2C_HandleTypeDef I2C_Bus4;

/* I2C Function prototypes */
void I2CInit(void);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c);

/* Functions to read and write multi-byte values */
void I2CWrite16(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress, uint8_t memAddress,
    uint16_t value);
uint16_t I2CRead16(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress, uint8_t memAddress);
void I2CWrite24(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress, uint8_t memAddress,
    uint32_t value);
uint32_t I2CRead24(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress, uint8_t memAddress);

#endif //I2CTEST_I2C_H
