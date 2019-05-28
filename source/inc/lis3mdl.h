//
// Created by RogerMoore on 28/05/2019.
//

#ifndef I2CTEST_LIS3MDL_H
#define I2CTEST_LIS3MDL_H

#include <stdbool.h>
#include "i2c.h"

/* Default I2C address of all LIS3MDL chips */
static const I2CAddress_t LIS3MDL_I2C_ADDRESS     = 0x1c;
/* Alternate I2C address of all LIS3MDL chips used if SDO/SA1 pin is high*/
static const I2CAddress_t LIS3MDL_I2C_ADDRESS_ALT = 0x1e;

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
} LIS3MDLRegister_t;

/* Structure to hold information about an LHS22HB device */
typedef struct {
  I2C_HandleTypeDef *i2cBus;          /* I2C bus handler */
  I2CAddress_t       i2cAddress;      /* Address on the I2C bus */
} LIS3MDLHandle_t;

/* Create global structure to contain the configuration data for attached LIS3MDL sensors */
LIS3MDLHandle_t lis3mdl;

/* LIS3MDL Function prototypes */
void LIS3MDLInit(void);
HAL_StatusTypeDef LIS3MDLConfigure(LIS3MDLHandle_t *dev);
uint8_t LIS3MDLReadRegister(LIS3MDLHandle_t *dev,LIS3MDLRegister_t reg);
void LIS3MDLWriteRegister(LIS3MDLHandle_t *dev,LIS3MDLRegister_t reg,uint8_t value);

#endif //I2CTEST_LIS3MDL_H
