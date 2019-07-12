//
// Created by RogerMoore on 24/05/2019.
//

#ifndef I2CTEST_I2C_H
#define I2CTEST_I2C_H

#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_144.h"

/* Defines a default time out value for I2C read commands (units are ms) */
#define I2C_TIMEOUT 1000

typedef uint16_t I2C_Address_t;

// Structures to contain I2C bus configuration data
I2C_HandleTypeDef I2C_Bus1;
I2C_HandleTypeDef I2C_Bus2;
I2C_HandleTypeDef I2C_Bus3;
I2C_HandleTypeDef I2C_Bus4;

/* I2C Function prototypes */
void I2C_Initialize(void);
void I2C_ErrorHandler(char *func,I2C_Address_t i2cAddress, uint8_t memAddress,char *msg);

/* Functions to read and write register values */
void I2C_Write8(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress,
                uint8_t value);
uint8_t I2C_Read8(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress);
void I2C_Write16LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress,
                   uint16_t value);
uint16_t I2C_Read16LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress);
void I2C_Write16BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress,
                   uint16_t value);
uint16_t I2C_Read16BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress);
void I2C_Write24LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress,
                   uint32_t value);
uint32_t I2C_Read24LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress);
void I2C_Write24BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress,
                   uint32_t value);
uint32_t I2C_Read24BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress);

#endif //I2CTEST_I2C_H
