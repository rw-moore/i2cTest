//
// Created by RogerMoore on 28/05/2019.
//

#ifndef I2CTEST_LPS22HB_H
#define I2CTEST_LPS22HB_H

#include <stdbool.h>
#include "i2c.h"

/* Default I2C address of all LPS22HB chips */
static const I2CAddress_t LPS22HB_I2C_ADDRESS     = 0x2e;
/* Alternate I2C address of all LPS22HB chips used if SA0 pin is high*/
static const I2CAddress_t LPS22HB_I2C_ADDRESS_ALT = 0x2f;

/* Device ID stored in WHO_AM_I register */
static const uint8_t LPS22HB_DEVICE_ID           = 0xb1;

/* LPS22HB Registers (text taken from userguide)
 * Note that reserved registers must never be written to and it could damage the
 * device if they are written to (warning from user guide).
 */
typedef enum {
  /* 0x00-0x0a are reserved */
  /*
   * INTERRUPT REGISTER
   * Interrupt mode for pressure acquisition configuration.
   * See user guide for definition of each bit.
   */
  LPS22HB_REG_INTERRUPT_CFG   =0x0b,
  /*
   * PRESSURE THRESHOLD REGISTERS
   * User-defined threshold value for pressure interrupt event, 16-bit value.
   * The threshold value for pressure interrupt generation is a 16-bit unsigned
   * right-justified value composed of THS_P_H (0Dh) and THS_P_L (0Ch).The value
   * is expressed as:
   *      Interrupt threshold (hPa) = ±THS_P / 16
   * To enable the interrupt event based on this user-defined threshold, the DIFF_EN bit in
   * INTERRUPT_CFG (0Bh) must be set to '1', the PHE bit or PLE bit (or both bits) in
   * INTERRUPT_CFG (0Bh) has to be enabled.
   */
  LPS22HB_REG_THS_P_L = 0x0c,
  LPS22HB_REG_THS_P_H = 0x0d,
  /* 0x0e is reserved */
  /*
   * DEVICE ID
   * Device Who am I value which serves as a device ID, returns 0b10110001 = 177 = 0xb1
   */
  LPS22HB_REG_WHO_AM_I = 0x0f,
  /*
   * CONTROL REGISTER1
   *
   *  bit 0: SPI Serial Interface Mode selection. Default value: 0
   *          (0: 4-wire interface; 1: 3-wire interface)
   *  bit 1: Block data update. Default value: 0
   *         (0: continuous update;
   *          1: output registers not updated until MSB and LSB have been read)
   *  bit 2: LPFP_CFG: Low-pass configuration register. Default value:0 See guide
   *  bit 3: Enable low-pass filter on pressure data when Continuous mode is used.
   *         Default value: 0 (0: Low-pass filter disabled; 1: Low-pass filter enabled)
   *  bits 4-6: Output data rate selection. Default value: 000 (bits 4;5;6)
   *            000 = Power down/one-shot mode enabled
   *            001 = 1 Hz
   *            010 = 10 Hz
   *            011 = 25 Hz
   *            100 = 50 Hz
   *            101 = 75 Hz
   *            111 = ??? User guide does not say!
   *  bit 7: Must always be zero.
   */
  LPS22HB_REG_CTRL_REG1 = 0x10,
  LPS22HB_REG_CTRL_REG2 = 0x11,
  LPS22HB_REG_CTRL_REG3 = 0x12,
  /* 0x13 is reserved */
  /*
   * FIFO CONFIGURATION
   */
  LPS22HB_REG_FIFO_CTRL = 0x14,
  /*
   * REFERENCE PRESSURE
   */
  LPS22HB_REG_REF_P_XL = 0x15,
  LPS22HB_REG_REF_P_L = 0x16,
  LPS22HB_REG_REF_P_H = 0x17,
  /*
   * PRESSURE OFFSET
   */
  LPS22HB_REG_RPDS_L = 0x18,
  LPS22HB_REG_RPDS_H = 0x19,
  /*
   * RESOLUTION
   */
  LPS22HB_REG_RES_CONF = 0x1a,
  /* 0x1b - 0x24 are reserved */
  /*
   * INTERRUPT
   */
  LPS22HB_REG_INT_SOURCE = 0x25,
  /*
   * FIFO STATUS
   */
  LPS22HB_REG_FIFO_STATUS = 0x26,
  /*
   * STATUS
   */
  LPS22HB_REG_STATUS = 0x27,
  /*
   * PRESSURE OUTPUT
   */
  LPS22HB_REG_PRESS_OUT_XL = 0x28,
  LPS22HB_REG_PRESS_OUT_L = 0x29,
  LPS22HB_REG_PRESS_OUT_H = 0x2a,
  /*
   * TEMPERATURE OUTPUT
   */
  LPS22HB_REG_TEMP_OUT_L = 0x2b,
  LPS22HB_REG_TEMP_OUT_H = 0x2c,
  /* 0x2d - 0x32 are reserved */
  /*
   * FILTER RESET
   */
  LPS22HB_REG_LPFP_RES = 0x33
} LPS22HBRegister_t;

/* Structure to hold information about an LHS22HB device */
typedef struct {
  I2C_HandleTypeDef *i2cBus;          /* I2C bus handler */
  I2CAddress_t       i2cAddress;      /* Address on the I2C bus */
} LPS22HBHandle_t;

/* Create global structure to contain the configuration data for attached ADXL355 sensors */
LPS22HBHandle_t lps22hb;

/* LPS22HB Function prototypes */
void LPS22HBInit(void);
HAL_StatusTypeDef LPS22HBConfigure(LPS22HBHandle_t *dev);
uint8_t LPS22HBReadRegister(LPS22HBHandle_t *dev,LPS22HBRegister_t reg);
void LPS22HBWriteRegister(LPS22HBHandle_t *dev,LPS22HBRegister_t reg,uint8_t value);




#endif //I2CTEST_LPS22HB_H
