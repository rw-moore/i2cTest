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

// I2C Function prototypes
void I2CInit(void);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c);

#endif //I2CTEST_I2C_H
