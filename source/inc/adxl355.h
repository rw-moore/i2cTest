//
// Created by RogerMoore on 24/05/2019.
//

#ifndef I2CTEST_ADXL355_H
#define I2CTEST_ADXL355_H

#include <stdbool.h>
#include "i2c.h"

/* Default I2C address of all ADXL355 chips */
static const I2CAddress_t ADXL355_I2C_ADDRESS     = 0x1d;
/* Alternate I2C address of all ADXL355 chips used if ASEL pin is high*/
static const I2CAddress_t ADXL355_I2C_ADDRESS_ALT = 0x53;

/* Device IDs stored in registers */
static const uint8_t ADXL355_ANALOGUE_DEVICES_ID = 0xad;
static const uint8_t ADXL355_MEMS_ID             = 0x1d;
static const uint8_t ADXL355_DEVICE_ID           = 0xed;

/* ADXL355 Registers (text from ADXL user guide)*/
typedef enum {
  /*
   * ANALOG DEVICES ID REGISTER
   * This register contains the Analog Devices ID, 0xAD.
   */
  ADXL355_REG_DEVID_AD   = 0x00,
  /* ANALOG DEVICES MEMS ID REGISTER
  * This register contains the Analog Devices MEMS ID, 0x1D.
  */
  ADXL355_REG_DEVID_MST  = 0x01,
  /*
   * DEVICE ID REGISTER
   * This register contains the device ID, 0xED (355 octal).
   */
  ADXL355_REG_PARTID     = 0x02,
  /*
   * PRODUCT REVISION ID REGISTER
   * This register contains the product revision ID, beginning with 0x00 and incrementing for each
   * subsequent revision.
   */
  ADXL355_REG_REVID      = 0x03,
  /*
   * STATUS REGISTER
   * This register includes bits that describe the various conditions of the ADXL355.
   */
  ADXL355_REG_STATUS     = 0x04,
  /*
   * FIFO ENTRIES REGISTER
   * This register indicates the number of valid data samples present in the FIFO buffer.
   * This number ranges from 0 to 96.
   */
  ADXL355_REG_FIFO_ENTRIES= 0x05,
  /*
   * TEMPERATURE DATA REGISTERS
   * These two registers contain the uncalibrated temperature data. The nominal intercept is
   * 1852 LSB at 25°C and the nominal slope is −9.05 LSB/°C. TEMP2 contains the four most significant
   * bits, and TEMP1 contains the eight least significant bits of the 12-bit value.
   */
  ADXL355_REG_TEMP2      = 0x06,
  ADXL355_REG_TEMP1      = 0x07,
  /*
   * X-AXIS DATA REGISTERS
   * These three registers contain the x-axis acceleration data. Data is left justified and
   * formatted as twos complement.
   */
  ADXL355_REG_XDATA3     = 0x08,
  ADXL355_REG_XDATA2     = 0x09,
  ADXL355_REG_XDATA1     = 0x0a,
  /*
   * Y-AXIS DATA REGISTERS
   * These three registers contain the y-axis acceleration data. Data is left justified and
   * formatted as twos complement.
   */
  ADXL355_REG_YDATA3     = 0x0b,
  ADXL355_REG_YDATA2     = 0x0c,
  ADXL355_REG_YDATA1     = 0x0d,
  /*
   * Z-AXIS DATA REGISTERS
   * These three registers contain the z-axis acceleration data. Data is left justified and
   * formatted as twos complement.
   */
  ADXL355_REG_ZDATA3     = 0x0e,
  ADXL355_REG_ZDATA2     = 0x0f,
  ADXL355_REG_ZDATA1     = 0x10,
  /*
   * FIFO ACCESS REGISTER
   * Read this register to access data stored in the FIFO.
   * FIFO data is formatted to 24 bits, 3 bytes, most significant byte first. A read to this
   * address pops an effective three equal byte words of axis data from the FIFO. Two
   * subsequent reads or a multibyte read completes the transaction of this data onto the
   * interface. Continued reading or a sustained multibyte read of this field continues to
   * pop the FIFO every third byte. Multibyte reads to this address do not increment the
   * address pointer. If this address is read due to an autoincrement from the previous
   * address, it does not pop the FIFO. Instead, it returns zeros and increments on to the
   * next address.
   */
  ADXL355_REG_FIFO_DATA  = 0x11,
  /*
   * X-AXIS OFFSET TRIM REGISTERS
   * Offset added to x-axis data after all other signal processing. Data is in twos complement
   * format. The significance of OFFSET_X[15:0] matches the significance of XDATA[19:4].
   */
  ADXL355_REG_OFFSET_X_H = 0x1e,
  ADXL355_REG_OFFSET_X_L = 0x1f,
  /*
   * Y-AXIS OFFSET TRIM REGISTERS
   * Offset added to x-axis data after all other signal processing. Data is in twos complement
   * format. The significance of OFFSET_Y[15:0] matches the significance of YDATA[19:4].
   */
  ADXL355_REG_OFFSET_Y_H = 0x20,
  ADXL355_REG_OFFSET_Y_L = 0x21,
  /*
   * Z-AXIS OFFSET TRIM REGISTERS
   * Offset added to x-axis data after all other signal processing. Data is in twos complement
   * format. The significance of OFFSET_Z[15:0] matches the significance of ZDATA[19:4].
   */
  ADXL355_REG_OFFSET_Z_H = 0x22,
  ADXL355_REG_OFFSET_Z_L = 0x23,
  /********************  NOT YET IMPLEMENTED REGISTERS HERE *************************/
  /*
   * I2C SPEED, INTERRUPT POLARITY, AND RANGE REGISTER
   * Bits 0:1 determine the dynamic range: 01 = +/-2g, 10 = +/-4g, 11 =+/-8g
   */
  ADXL355_REG_RANGE      = 0x2c,
  /*
   * POWER CONTROL REGISTER
   * bit 2 : Set to 1 to force the DRDY output to 0 in modes where it is normally signal data ready.
   * bit 1 : Set to 1 to disable temperature processing. Temperature processing is also disabled
   *         when STANDBY = 1.
   * bit 0 : Standby mode = 1, measurement mode0.
   *         In standby mode, the device is in a low power state, and the
   *         temperature and acceleration datapaths are not operating. In addition, digital
   *         functions, including FIFO pointers, reset. Changes to the configuration setting of the
   *         device must be made when STANDBY1. An exception is a high-pass filter that can
   *         be changed when the device is operating.
   */
  ADXL355_REG_POWER_CTL  = 0x2d,
  /*
   * SELF TEST REGISTER
   * bit 1 : Set to 1 to enable self test force
   * bit 0 : Set to 1 to enable self test mode
   */
  ADXL355_REG_SELF_TEST  = 0x2e,
  /*
   * RESET REGISTER
   * Write Code 0x52 to reset the device, similar to a power-on reset (POR)
   */
  ADXL355_REG_RESET      = 0x2f
} ADXL355_Register_t;

/* Dynamic ranges for accelerometer */
typedef enum {
  /* +/- 2g dynamic range */
      ADXL355_RANGE_2G = 1,
  /* +/- 4g dynamic range */
      ADXL355_RANGE_4G = 2,
  /* +/- 8g dynamic range */
      ADXL355_RANGE_8G = 3
} ADXL355_Range_t;

/* Structure to hold information about an ADXL355 device */
typedef struct {
    I2C_HandleTypeDef *i2cBus;          /* I2C bus handler */
    I2CAddress_t       i2cAddress;      /* Address on the I2C bus */
    ADXL355_Range_t    range;           /* Configured dynamic range */
    bool               standby;         /* True if device is configured to be in standby */
    bool               enableTemp;      /* True if temperature measurement is enabled */
} ADXL355_Handle_t;

/* Create global structure to contain the configuration data for attached ADXL355 sensors */
ADXL355_Handle_t adxl355;

// ADXL355 Function prototypes
void ADXL355_Initialize(void);
HAL_StatusTypeDef ADXL355_Configure(ADXL355_Handle_t *dev);
void ADXL355_StandbyOn(ADXL355_Handle_t *dev);
void ADXL355_StandbyOff(ADXL355_Handle_t *dev);
void ADXL355_EnableTemperature(ADXL355_Handle_t *dev);
void ADXL355_DisableTemperature(ADXL355_Handle_t *dev);
void ADXL355_SetRange(ADXL355_Handle_t *dev, ADXL355_Range_t range);
uint16_t ADXL355_RawTemperature(ADXL355_Handle_t *dev);
int32_t ADXL355_XAcceleration(ADXL355_Handle_t *dev);
int32_t ADXL355_YAcceleration(ADXL355_Handle_t *dev);
int32_t ADXL355_ZAcceleration(ADXL355_Handle_t *dev);
int32_t ADXL355_XTrim(ADXL355_Handle_t *dev);
int32_t ADXL355_YTrim(ADXL355_Handle_t *dev);
int32_t ADXL355_ZTrim(ADXL355_Handle_t *dev);
void ADXL355_SetXTrim(ADXL355_Handle_t *dev,int32_t value);
void ADXL355_SetYTrim(ADXL355_Handle_t *dev,int32_t value);
void ADXL355_SetZTrim(ADXL355_Handle_t *dev,int32_t value);

// Simple inline functions
/*
 * Reads a register on the ADXL355 and returns the value.
 */
static inline uint8_t ADXL355_ReadRegister(ADXL355_Handle_t *dev, ADXL355_Register_t reg) {
  return I2C_Read8(dev->i2cBus, dev->i2cAddress, reg);
}

/*
 * Writes to a register on the ADXL355.
 */
static inline void ADXL355_WriteRegister(ADXL355_Handle_t *dev, ADXL355_Register_t reg, uint8_t value) {
  I2C_Write8(dev->i2cBus, dev->i2cAddress, reg, value);
}

/*
 * Completely resets the ADXL355 to power on status.
 * This is achieved by writing 0x52 to the reset register as per the ADXL355 datasheet.
 */
static inline void ADXL355_Reset(ADXL355_Handle_t *dev) {
  I2C_Write8(dev->i2cBus, dev->i2cAddress, ADXL355_REG_RESET, 0x52);
}

#endif //I2CTEST_ADXL355_H
