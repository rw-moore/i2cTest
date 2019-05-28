//
// Created by RogerMoore on 24/05/2019.
//

#include "i2c.h"

extern void Error_Handler(void);

#define I2C_ADDRESS        0x0

/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 100 MHz */
/* This example use TIMING to 0x00901954 to reach 400 kHz speed (Rise time = 100 ns, Fall time = 10 ns) */
//#define I2C_TIMING      0x00901954
/* New value from Jim Braun to set bus to 100kHz clock speed */
#define I2C_TIMING      0x10909CEC

// Configure I2C buses for sensor I/O
void I2CInit(void) {
  /* Configure I2C bus 2*/
  I2C_Bus2.Instance = I2C2;
  I2C_Bus2.Init.Timing = I2C_TIMING;
  I2C_Bus2.Init.OwnAddress1 = I2C_ADDRESS;
  I2C_Bus2.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  I2C_Bus2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2C_Bus2.Init.OwnAddress2 = 0x0;
  I2C_Bus2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2C_Bus2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2C_Bus2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  /* Initialize the I2C2 bus and check the return status */
  if (HAL_I2C_Init(&I2C_Bus2) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }
  /* Enable the Analog I2C Filter which can filter out noise spikes of up to 50ns*/
  if (HAL_I2CEx_ConfigAnalogFilter(&I2C_Bus2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  };
  /* Configure the digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&I2C_Bus2, 0) != HAL_OK) {
    Error_Handler();
  }
}

/**
* I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C2) {
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /* I2C2 GPIO Configuration
        PF0     ------> I2C2_SDA
        PF1     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  }
}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
  if(hi2c->Instance==I2C2) {
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0 | GPIO_PIN_1);
  }
}


