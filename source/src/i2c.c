//
// Created by RogerMoore on 24/05/2019.
//

#include "i2c.h"
#include <stdio.h>

extern void Error_Handler(void);

#define I2C_ADDRESS        0x0

/* I2C TIMING Register define when I2C clock source is APB1 (SYSCLK/4) */
/* I2C TIMING is calculated in case of the I2C Clock source is the APB1CLK = 100 MHz */
/* This example use TIMING to 0x00901954 to reach 400 kHz speed (Rise time = 100 ns, Fall time = 10 ns) */
//#define I2C_TIMING      0x00901954
/* New value from Jim Braun to set bus to 100kHz clock speed */
//#define I2C_TIMING      0x10909CEC
#define I2C_TIMING      0x10707DBC  /* 100 kHz from STM32cube */

// Configure I2C buses for sensor I/O
void I2C_Initialize(void) {
  /* Configure I2C bus 2*/
  __I2C2_CLK_ENABLE();
  I2C_Bus2.Instance = I2C2;
  I2C_Bus2.Init.Timing = I2C_TIMING;
  I2C_Bus2.Init.OwnAddress1 = I2C_ADDRESS;
  I2C_Bus2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
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
    printf("Configuring I2C2 pins...\n");
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /* I2C2 GPIO Configuration
        PF0     ------> I2C2_SDA
        PF1     ------> I2C2_SCL
    */
    GPIO_InitStruct.Pin       = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
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

/* Writes an 8-bit value to the given memory address.
 * This code handles the storage and address pointers and error handling needed by the HAL routines.
 */
void I2C_Write8(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress,
                uint8_t memAddress, uint8_t value){
  /* Write the value byte to the I2C bus remembering to increase the memory
   * address in the device for each successive byte.
   */
  if (HAL_I2C_Mem_Write(i2cBus, i2cAddress, memAddress, sizeof(uint8_t),
      &value, 1, I2C_TIMEOUT) != HAL_OK) {
    I2C_ErrorHandler("I2CWrite8",i2cAddress,memAddress,"I2C error on write");
  }
};

/* Reads an 8-bit value from the given address.
 * This wraps the HAL routines to provide a simple call that returns the value. It also
 * handles errors raised by the HAL.
 */
uint8_t I2C_Read8(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress){
  /* Variable to store the final value */
  uint8_t value=0;
  /* Read a byte from the I2C bus remembering to increase the memory
   * address in the device for each successive byte.
   */
  if (HAL_I2C_Mem_Read(i2cBus, i2cAddress, memAddress, sizeof(uint8_t),
      &value, 1, I2C_TIMEOUT) != HAL_OK) {
    I2C_ErrorHandler("I2CRead8",i2cAddress,memAddress,"I2C error on read");
  }
  /* Return the final value */
  return value;
};


/* Writes a 16-bit value to the given memory address in little-endian format.
 * Since I2C is an 8-bit protocol this will write the MSB to the address and the
 * LSB to the next address.
 */
void I2C_Write16LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress,
                uint8_t memAddress, uint16_t value){
  /* Single byte variable to store the byte being written */
  uint8_t data;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<2; i++) {
    /* Right shift the value by a decreasing multiple of 8 so that the MSB is
     * written first (i=0) then mask out all but 8 bits.
     */
    data = (value >> (8*(1-i))) & 0xff;
    /* Write the extracted byte to the I2C bus remembering to increase the memory
     * address in the device for each successive byte.
     */
    if (HAL_I2C_Mem_Write(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                           &data, 1, I2C_TIMEOUT) != HAL_OK) {
      I2C_ErrorHandler("I2CWrite16",i2cAddress,memAddress+i,"I2C error on write");
    }
  }
};

/* Reads a 16-bit value from the given address in little-endian format.
 * Since I2C is an 8-bit protocol this will read the MSB from the address and the
 * LSB from the next address.
 */
uint16_t I2C_Read16LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress){
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
      I2C_ErrorHandler("I2CRead16",i2cAddress,memAddress+i,"I2C error on read");
    }
    /* Left shift the value by 8 bits to make room for the new data to be OR-ed in. */
    value = (value << 8) | data;
  }
  /* Return the final assembled value */
  return value;
};

/* Writes a 16-bit value to the given memory address in big-endian format.
 * Since I2C is an 8-bit protocol this will write the LSB to the address and the
 * MSB to the next address.
 */
void I2C_Write16BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress,
                   uint8_t memAddress, uint16_t value){
  /* Single byte variable to store the byte being written */
  uint8_t data;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<2; i++) {
    /* Right shift the value by a increasing multiple of 8 so that the LSB is
     * written first (i=0) then mask out all but 8 bits.
     */
    data = (value >> (8*i)) & 0xff;
    /* Write the extracted byte to the I2C bus remembering to increase the memory
     * address in the device for each successive byte.
     */
    if (HAL_I2C_Mem_Write(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                          &data, 1, I2C_TIMEOUT) != HAL_OK) {
      I2C_ErrorHandler("I2CWrite16",i2cAddress,memAddress+i,"I2C error on write");
    }
  }
};

/* Reads a 16-bit value from the given address in big-endian format.
 * Since I2C is an 8-bit protocol this will read the LSB from the address and the
 * MSB from the next address.
 */
uint16_t I2C_Read16BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress){
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
      I2C_ErrorHandler("I2CRead16",i2cAddress,memAddress+i,"I2C error on read");
    }
    /* Left shift the data by 8 bits and OR it to the value. */
    value |= (data << (8*i));
  }
  /* Return the final assembled value */
  return value;
};

/* Writes a 24-bit value to the given address in little-endian format.
 * Since I2C is an 8-bit protocol this will write the MSB to the address and the
 * other two bytes to the next two addresses so that the LSB is at (address+2).
 */
void I2C_Write24LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress,
                uint8_t memAddress,uint32_t value) {
  /* Single byte variable to store the byte being written */
  uint8_t data;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<3; i++) {
    /* Right shift the value by a decreasing multiple of 8 so that the MSB is
     * written first (i=0) then mask out all but 8 bits.
     */
    data = (value >> (8*(2-i))) & 0xff;
    /* Write the extracted byte to the I2C bus remembering to increase the memory
     * address in the device for each sucecssive byte.
     */
    if (HAL_I2C_Mem_Write(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                          &data, 1, I2C_TIMEOUT) != HAL_OK) {
      I2C_ErrorHandler("I2C_Write24LE",i2cAddress,memAddress+i,"I2C error on write");
    }
  }
};

/* Reads a 24-bit value from the given address in little-endian format.
 * Since I2C is an 8-bit protocol this will read the MSB from the address and the
 * nest two bytes from the next two addresses i.e. the LSB is read from (address+2).
 */
uint32_t I2C_Read24LE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress){
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
      I2C_ErrorHandler("I2C_Read24LE",i2cAddress,memAddress+i,"I2C error on read");
    }
    /* Left shift the value by 8 bits to make room for the new data to be OR-ed in. */
    value = (value << 8) | data;
  }
  /* Return the final assembled value */
  return value;
};

/* Writes a 24-bit value to the given address in big-endian format.
 * Since I2C is an 8-bit protocol this will write the MSB to the address and the
 * other two bytes to the next two addresses so that the LSB is at (address+2).
 */
void I2C_Write24BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress,
                   uint8_t memAddress,uint32_t value) {
  /* Single byte variable to store the byte being written */
  uint8_t data;
  /* Loop over the bytes extracting and then writing each one */
  for (int i=0; i<3; i++) {
    /* Right shift the value by a increasing multiple of 8 so that the LSB is
     * written first (i=0) then mask out all but 8 bits.
     */
    data = (value >> (8*i)) & 0xff;
    /* Write the extracted byte to the I2C bus remembering to increase the memory
     * address in the device for each sucecssive byte.
     */
    if (HAL_I2C_Mem_Write(i2cBus, i2cAddress, memAddress+i, sizeof(uint8_t),
                          &data, 1, I2C_TIMEOUT) != HAL_OK) {
      I2C_ErrorHandler("I2C_Write24BE",i2cAddress,memAddress+i,"I2C error on write");
    }
  }
};

/* Reads a 24-bit value from the given address in big-endian format.
 * Since I2C is an 8-bit protocol this will read the MSB from the address and the
 * nest two bytes from the next two addresses i.e. the LSB is read from (address+2).
 */
uint32_t I2C_Read24BE(I2C_HandleTypeDef *i2cBus, I2C_Address_t i2cAddress, uint8_t memAddress){
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
      I2C_ErrorHandler("I2C_Read24BE",i2cAddress,memAddress+i,"I2C error on read");
    }
    /* Left shift the data by 8 bits and OR it to the value. */
    value |= (data << (8*i));
  }
  /* Return the final assembled value */
  return value;
};

/* Handles I2C errors.
 * Function which reports on I2C errors which have been detected usually by the HAL code.
 */
void I2C_ErrorHandler(char *func,I2C_Address_t i2cAddress, uint8_t memAddress,char *msg) {
  printf("%s (dev=0x%02x,addr=0c%02x): %s",func,i2cAddress,memAddress,msg);
  return;
}
