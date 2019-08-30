//
// Created by RogerMoore on 28/05/2019.
//

#ifndef I2CTEST_LIS3MDL_H
#define I2CTEST_LIS3MDL_H

#include <stdbool.h>
#include "i2c.h"

/* Default I2C address of all LIS3MDL chips */
static const I2C_Address_t LIS3MDL_I2C_ADDRESS     = 0x1c;
/* Alternate I2C address of all LIS3MDL chips used if SDO/SA1 pin is high*/
static const I2C_Address_t LIS3MDL_I2C_ADDRESS_ALT = 0x1e;

/* Device ID stored in WHO_AM_I register */
static const uint8_t LIS3MDL_DEVICE_ID           = 0x3d;

/* LIS3MDL Registers (text taken from userguide)
 * Note that reserved registers must never be written to and it could damage the
 * device if they are written to (warning from user guide).
 */
typedef enum {
  /* 0x00 - 0x0e are reserved */
  /*
   * DEVICE ID
   * Device identification register.
   */
  LIS3MDL_REG_WHO_AM_I = 0x0f,
  /* 0x10 - 0x1f are reserved */
  /*
   * CONTROL REGISTER 1
   * bit 0: Self-test enable. Default value: 0 (0: self-test disabled; 1: self-test enabled)
   * bit 1: FAST_ODR enables data rates higher than 80Hz when set
   * bit 2-4: Output data rate selection. Default value: 100
   *    000 = 0.625 Hz
   *    001 = 1.25 Hz
   *    010 = 2.5 Hz
   *    011 = 5 Hz
   *    100 = 20 Hz
   *    110 = 40 Hz
   *    111 = 80 Hz
   * bit 5-6: X and Y axes operative mode selection. Default value: 00
   *    00 = Low-power mode (1000 Hz w/FAST_ODR)
   *    01 = Medium-performance mode (560 Hz w/FAST_ODR)
   *    10 = High-performance mode (300 Hz w/FAST_ODR)
   *    11 = Ultra-high-performance mode (155 Hz w/FAST_ODR)
   *    Note: the rates do not seem to make sense but they are in use guide this way...
   * bit 7: Temperature sensor enable. Default value: 0 (0: temperature sensor disabled;
   *        1: temperature sensor enabled)
   */
  LIS3MDL_REG_CTRL_REG1 = 0x20,
  /*
   * CONTROL REGISTER 2
   */
  LIS3MDL_REG_CTRL_REG2 = 0x21,
  /*
   * CONTROL REGISTER 3
   */
  LIS3MDL_REG_CTRL_REG3 = 0x22,
  /*
   * CONTROL REGISTER 4
   */
  LIS3MDL_REG_CTRL_REG4 = 0x23,
  /*
   * CONTROL REGISTER 5
   */
  LIS3MDL_REG_CTRL_REG5 = 0x24,
  /* 0x25 - 0x26 are reserved */
  /*
   * STATUS REGISTER
   */
  LIS3MDL_REG_STATUS_REG = 0x27,
  /*
   * X AXIS FIELD DATA
   * X-axis data output. The value of magnetic field is expressed as a 16-bit value
   * using two’s complement.
   */
  LIS3MDL_REG_OUT_X_L = 0x28,
  LIS3MDL_REG_OUT_X_H = 0x29,
  /*
   * Y AXIS FIELD DATA
   * Y-axis data output. The value of magnetic field is expressed as a 16-bit value
   * using two’s complement.
   */
  LIS3MDL_REG_OUT_Y_L = 0x2a,
  LIS3MDL_REG_OUT_Y_H = 0x2b,
  /*
   * Z AXIS FIELD DATA
   * Z-axis data output. The value of magnetic field is expressed as a 16-bit value
   * using two’s complement.
   */
  LIS3MDL_REG_OUT_Z_L = 0x2c,
  LIS3MDL_REG_OUT_Z_H = 0x2d,
  /*
   * TEMPERATURE
   * Temperature sensor data. The value of temperature is expressed as a 16-bit value
   * using two’s complement.
   */
  LIS3MDL_REG_TEMP_OUT_L = 0x2e,
  LIS3MDL_REG_TEMP_OUT_H = 0x2f,
  /*
   * INTERRUPT CONFIGURATION
   */
  LIS3MDL_REG_INT_CFG = 0x30,
  /*
   * INTERRUPT SOURCE
   */
  LIS3MDL_REG_INT_SRC = 0x31,
  /*
   * INTERRUPT THRESHOLD
   * Interrupt threshold. Default value: 0. The value is expressed in 16-bit unsigned.
   * Even though the threshold is expressed in absolute value, the device detects both
   * positive and negative thresholds.
   */
  LIS3MDL_REG_INT_THS_L = 0x32,
  LIS3MDL_REG_INT_THS_H = 0x33
} LIS3MDL_Register_t;

/* Dynamic ranges for magnetometer specified in gauss where 1 gauss = 0.1 mT*/
typedef enum {
  /* +/- 2 gauss dynamic range */
      LIS3MDL_RANGE_4G  = 0,
  /* +/- 8 gauss dynamic range */
      LIS3MDL_RANGE_8G  = 1,
  /* +/- 12 gauss dynamic range */
      LIS3MDL_RANGE_12G = 2,
  /* +/- 16 gauss dynamic range */
      LIS3MDL_RANGE_16G = 3
} LIS3MDL_Range_t;

/* Structure to hold information about an LHS22HB device */
typedef struct {
  I2C_HandleTypeDef *i2cBus;          /* I2C bus handler */
  I2C_Address_t      i2cAddress;      /* Address on the I2C bus */
  LIS3MDL_Range_t    range;           /* Configured dynamic range */
  bool               enableTemp;      /* True if temperature measurement is enabled */
} LIS3MDL_Handle_t;

/* Create global structure to contain the configuration data for attached LIS3MDL sensors */
LIS3MDL_Handle_t lis3mdl;

/* LIS3MDL Function prototypes */
void LIS3MDL_Init(void);
HAL_StatusTypeDef LIS3MDL_Configure(LIS3MDL_Handle_t *dev);
void LIS3MDL_EnableTemperature(LIS3MDL_Handle_t *dev);
void LIS3MDL_DisableTemperature(LIS3MDL_Handle_t *dev);
void LIS3MDL_SetRange(LIS3MDL_Handle_t *dev, LIS3MDL_Range_t range);
float_t LIS3MDL_XBField(LIS3MDL_Handle_t *dev);
float_t LIS3MDL_YBField(LIS3MDL_Handle_t *dev);
float_t LIS3MDL_ZBField(LIS3MDL_Handle_t *dev);

// Simple inline functions
/*
 * Reads a register on the LIS3MDL and returns the value.
 */
static inline uint8_t LIS3MDL_ReadRegister(LIS3MDL_Handle_t *dev, LIS3MDL_Register_t reg) {
  return I2C_Read8(dev->i2cBus, dev->i2cAddress, reg);
}

/*
 * Writes to a register on the LIS3MDL.
 */
static inline void LIS3MDL_WriteRegister(LIS3MDL_Handle_t *dev, LIS3MDL_Register_t reg, uint8_t value) {
  I2C_Write8(dev->i2cBus, dev->i2cAddress, reg, value);
}

/*
 * Reads the current, raw temperature data.
 */
static inline uint16_t LIS3MDL_ReadTemperature(LIS3MDL_Handle_t *dev) {
  return I2C_Read16BE(dev->i2cBus, dev->i2cAddress, LIS3MDL_REG_TEMP_OUT_L);
}

/*
 * Reads the raw x component magnetic field data.
 */
static inline int16_t LIS3MDL_RawXBField(LIS3MDL_Handle_t *dev) {
  /* Read and return the magnetic field data from the device */
  return I2C_Read16BE(dev->i2cBus, dev->i2cAddress, LIS3MDL_REG_OUT_X_L);
}

/*
 * Reads the raw y component magnetic field data.
 */
static inline int16_t LIS3MDL_RawYBField(LIS3MDL_Handle_t *dev) {
  /* Read and return the magnetic field data from the device */
  return I2C_Read16BE(dev->i2cBus, dev->i2cAddress, LIS3MDL_REG_OUT_Y_L);
}

/*
 * Reads the raw z component magnetic field data.
 */
static inline int16_t LIS3MDL_RawZBField(LIS3MDL_Handle_t *dev) {
  /* Read and return the magnetic field data from the device */
  return I2C_Read16BE(dev->i2cBus, dev->i2cAddress, LIS3MDL_REG_OUT_Z_L);
}

#endif //I2CTEST_LIS3MDL_H
