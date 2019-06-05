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

/* Writes a 16-bit value to the given memory address.
 * Since I2C is an 8-bit protocol this will write the LSB to the address and the
 * MSB to the next address. Little endian format it used since I2C devices use it.
 */
void I2CMemWrite16(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress,
                uint8_t memAddress, uint16_t value){
  /* Single byte variable to store the byte being written */
  uint8_t data;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<2; i++) {
    /* Right shift the value by an increasing multiple of 8 so that the LSB is
     * written first (i=0) then mask out all but the least significant 8 bits.
     */
    data = (value >> (8*i)) & 0xff;
    /* Write the extracted byte to the I2C bus remembering to increase the memory
     * address in the device for each successive byte.
     */
    if (HAL_I2C_Mem_Write(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                           &data, 1, I2C_TIMEOUT) != HAL_OK) {
      Error_Handler();
    }
  }
};

/* Reads a 16-bit value from the given address.
 * Since I2C is an 8-bit protocol this will read the LSB from the address and the
 * MSB from the next address.
 */
uint16_t I2CRead16(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress, uint8_t memAddress){
  /* Single byte variable to store the byte being read */
  uint8_t data;
  /* Variable to store the final multi-byte value */
  uint16_t value=0;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<2; i++) {
    /* Read a byte from the I2C bus remembering to increase the memory
     * address in the device for each successive byte.
     */
    if (HAL_I2C_Mem_Read(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                          &data, 1, I2C_TIMEOUT) != HAL_OK) {
      Error_Handler();
    }
    /* Left shift the byte read and or it into the final value. The casting to a
     * 16 bit variable is to ensure that there are bits to left shift into.
     */
    value |= (((uint16_t) data) << (i*8));
  }
  /* Return the final assembled value */
  return value;
};

/* Writes a 24-bit value to the given address.
 * Since I2C is an 8-bit protocol this will write the LSB to the address and the
 * other two bytes to the next two addresses so that the MSB is at (address+2).
 */
void I2CWrite24(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress,
                uint8_t memAddress,uint32_t value) {
  /* Single byte variable to store the byte being written */
  uint8_t data;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<3; i++) {
    /* Right shift the value by an increasing multiple of 8 so that the LSB is
     * written first (i=0) then mask out all but the least significant 8 bits.
     */
    data = (value >> (8*i)) & 0xff;
    /* Write the extracted byte to the I2C bus remembering to increase the memory
     * address in the device for each sucecssive byte.
     */
    if (HAL_I2C_Mem_Write(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                          &data, 1, I2C_TIMEOUT) != HAL_OK) {
      Error_Handler();
    }
  }
};

/* Reads a 24-bit value from the given address.
 * Since I2C is an 8-bit protocol this will read the LSB from the address and the
 * nest two bytes from the next two addresses i.e. the MSB is read from (address+2).
 */
uint32_t I2CRead24(I2C_HandleTypeDef *i2cBus, I2CAddress_t i2cAddress, uint8_t memAddress){
  /* Single byte variable to store the byte being read */
  uint8_t data;
  /* Variable to store the final multi-byte value */
  uint32_t value=0;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<3; i++) {
    /* Read a byte from the I2C bus remembering to increase the memory
     * address in the device for each successive byte.
     */
    if (HAL_I2C_Mem_Read(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                         &data, 1, I2C_TIMEOUT) != HAL_OK) {
      Error_Handler();
    }
    /* Left shift the byte read and or it into the final value. The casting to a
     * 16 bit variable is to ensure that there are bits to left shift into.
     */
    value |= (((uint32_t) data) << (i*8));
  }
  /* Return the final assembled value */
  return value;
};
