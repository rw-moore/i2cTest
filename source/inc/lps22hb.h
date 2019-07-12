//
// Created by RogerMoore on 28/05/2019.
//

#ifndef I2CTEST_LPS22HB_H
#define I2CTEST_LPS22HB_H

#include <stdbool.h>
#include "i2c.h"

/* Default I2C address of all LPS22HB chips */
static const I2C_Address_t LPS22HB_I2C_ADDRESS     = 0x5c;
/* Alternate I2C address of all LPS22HB chips used if SDO/SA0 pin is high*/
static const I2C_Address_t LPS22HB_I2C_ADDRESS_ALT = 0x5d;

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
  LPS22HB_REG_WHO_AM_I    = 0x0f,
  /*
   * CONTROL REGISTER 1
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
  LPS22HB_REG_CTRL_REG1   = 0x10,
  /*
   * CONTROL REGISTER 2
   *
   * bit 0: One-shot enable. Default value: 0 (0: idle mode; 1: a new dataset is acquired)
   * bit 1: 0
   * bit 2: Software reset. Default value: 0 (0: normal mode; 1: software reset).
   *        The bit is self-cleared when the reset is completed.
   * bit 3: Disable I2C interface. Default value: 0(0: I2C enabled;1: I2C disabled)
   * bit 4: Register address automatically incremented during a multiple byte access with a
   *        serial interface (I2C or SPI). Default value: 1 (0: disable; 1 enable)
   * bit 5: Stop on FIFO watermark. Enable FIFO watermark level use. Default value: 0
   *        (0: disable; 1: enable)
   * bit 6: FIFO enable. Default value: 0 (0: disable; 1: enable)
   * bit 7: Reboot memory content. Default value: 0 (0: normal mode; 1: reboot memory content).
   *        The bit is self-cleared when the BOOT is completed.
   */
  LPS22HB_REG_CTRL_REG2   = 0x11,
  /*
   * CONTROL REGISTER 3
   *
   * bit 0-1: Data signal on INT_DRDY pin control bits. Default value: 00
   *            00 = Data signal (in order of priority: DRDY or F_FTH or F_OVR or F_FSS5
   *            01 = Pressure high (P_high)
   *            10 = Pressure low (P_low)
   *            11 = Pressure low OR high
   * bit 2: Data-ready signal on INT_DRDY pin. Default value: 0 (0: disable; 1: enable)
   * bit 3: FIFO overrun interrupt on INT_DRDY pin. Default value: 0 (0: disable; 1: enable)
   * bit 4: FIFO watermark status on INT_DRDY pin. Default value: 0(0: disable; 1: enable)
   * bit 5: FIFO full flag on INT_DRDY pin. Default value: 0(0: disable; 1: enable)
   * bit 6: Push-pull/open drain selection on interrupt pads. Default value: 0
   *        (0: push-pull; 1: open drain)
   * bit 7: Interrupt active-high/low. Default value: 0(0: active high; 1: active low)
   */
  LPS22HB_REG_CTRL_REG3   = 0x12,
  /* 0x13 is reserved */
  /*
   * FIFO CONFIGURATION
   * FIFO control register
   *
   * bit 0-4: FIFO watermark level selection.
   * bit 5-7: FIFO mode selection. Default value: 000
   *            000 = Bypass mode
   *            001 = FIFO mode
   *            010 = Stream mode
   *            011 = Stream-to-FIFO mode
   *            100 = Bypass-to-Stream mode
   *            101 = Reserved
   *            110 = Dynamic-Stream mode
   *            111 = Bypass-to-FIFO mode
   */
  LPS22HB_REG_FIFO_CTRL   = 0x14,
  /*
   * REFERENCE PRESSURE
   * The Reference pressure value is a 24-bit data and it is composed of REF_P_H (17h),
   * REF_P_L (16h) and REF_P_XL (15h). The value is expressed as 2’s complement.
   * The reference pressure value is used when AUTOZERO or AUTORIFP function is enabled.
   * Please refer to INTERRUPT_CFG (0Bh) register description.
   */
  LPS22HB_REG_REF_P_XL    = 0x15,
  LPS22HB_REG_REF_P_L     = 0x16,
  LPS22HB_REG_REF_P_H     = 0x17,
  /*
   * PRESSURE OFFSET
   * The pressure offset value is 16-bit data that can be used to implement one-point calibration
   * (OPC) after soldering. This value is composed of RPDS_H (19h) and RPDS_L (18h). The
   * value is expressed as 2’s complement.
   */
  LPS22HB_REG_RPDS_L      = 0x18,
  LPS22HB_REG_RPDS_H      = 0x19,
  /*
   * RESERVED CONFIGURATION
   * Low-power mode configuration
   *
   * bit 0: Low current mode enable. Default 0
   *        0: Normal mode (low-noise mode); 1: Low-current mode.
   * bit 1: Reserved (value must not be changed)
   * bit 2-7: zero
   */
  LPS22HB_REG_RES_CONF    = 0x1a,
  /* 0x1b - 0x24 are reserved */
  /*
   * INTERRUPT SOURCE
   *
   * bit 0: Differential pressure High. (0: no interrupt has been generated;
   *        1: high differential pressure event has occurred).
   * bit 1: Differential pressure Low.(0: no interrupt has been generated;
   *        1: low differential pressure event has occurred).
   * bit 2: Interrupt active. (0: no interrupt has been generated;
   *        1: one or more interrupt events have been generated).
   * bit 3-6: zero
   * bit 7: If ‘1’ indicates that the Boot (Reboot) phase is running.
   */
  LPS22HB_REG_INT_SOURCE  = 0x25,
  /*
   * FIFO STATUS
   *
   * bit 0-5: FIFO stored data level. (000000: FIFO empty,
   *          100000: FIFO is full and has 32 unread samples).
   * bit 6: FIFO overrun status.(0: FIFO is not completely full;
   *        1: FIFO is full and at least one sample in the FIFO has been overwritten).
   * bit 7: FIFO watermark status.(0: FIFO filling is lower than threshold level;
   *        1: FIFO filling is equal or higher than threshold level).
   */
  LPS22HB_REG_FIFO_STATUS = 0x26,
  /*
   * STATUS REGISTER
   *
   * bit 0: Pressure data available. (0: new data for pressure is not yet available;
   *        1: a new pressure data is generated)
   * bit 1: Temperature data available.(0: new data for temperature is not yet available;
   *        1: a new temperature data is generated)
   * bit 4: Pressure data overrun. (0: no overrun has occurred;
   *        1: new data for pressure has overwritten the previous data)
   * bit 5: Temperature data overrun. (0: no overrun has occurred;
   *        1: a new data for temperature has overwritten the previous data)
   */
  LPS22HB_REG_STATUS      = 0x27,
  /*
   * PRESSURE OUTPUT
   * The pressure output value is a 24-bit data that contains the measured pressure. It is
   * composed of PRESS_OUT_H (2Ah), PRESS_OUT_L (29h) and PRESS_OUT_XL (28h).
   * The value is expressed as 2’s complement.
   * The output pressure register PRESS_OUT is provided as the difference between the
   * measured pressure and the content of the register RPDS (18h, 19h)*.
   * Please refer to Section 4.4: Interpreting pressure readings for additional info.
   * *DIFF_EN = '0', AUTOZERO = '0', AUTORIFP = '0'
   */
  LPS22HB_REG_PRESS_OUT_XL = 0x28,
  LPS22HB_REG_PRESS_OUT_L = 0x29,
  LPS22HB_REG_PRESS_OUT_H = 0x2a,
  /*
   * TEMPERATURE OUTPUT
   * The temperature output value is 16-bit data that contains the measured temperature. It is
   * composed of TEMP_OUT_H (2Ch), and TEMP_OUT_L (2Bh). The value is expressed as
   * 2’s complement.
   */
  LPS22HB_REG_TEMP_OUT_L  = 0x2b,
  LPS22HB_REG_TEMP_OUT_H  = 0x2c,
  /* 0x2d - 0x32 are reserved */
  /*
   * FILTER RESET
   * Low-pass filter reset register. If the LPFP is active, in order to avoid the transitory
   * phase, the filter can be reset by reading this register before generating pressure
   * measurements.
   */
  LPS22HB_REG_LPFP_RES    = 0x33
} LPS22HB_Register_t;

/* Structure to hold information about an LHS22HB device */
typedef struct {
  I2C_HandleTypeDef *i2cBus;          /* I2C bus handler */
  I2C_Address_t       i2cAddress;      /* Address on the I2C bus */
} LPS22HB_Handle_t;

/* Create global structure to contain the configuration data for attached ADXL355 sensors */
LPS22HB_Handle_t lps22hb;

/* LPS22HB Function prototypes */
void LPS22HB_Initialize(void);
HAL_StatusTypeDef LPS22HB_Configure(LPS22HB_Handle_t *dev);
uint8_t LPS22HB_ReadRegister(LPS22HB_Handle_t *dev,LPS22HB_Register_t reg);
void LPS22HB_WriteRegister(LPS22HB_Handle_t *dev,LPS22HB_Register_t reg,uint8_t value);

/* Pressure functions */
int32_t LPS22HB_ReadPressure(LPS22HB_Handle_t *dev);
void LPS22HB_SetRefPressure(LPS22HB_Handle_t *dev, int32_t value);
int32_t LPS22HB_ReadRefPressure(LPS22HB_Handle_t *dev);
void LPS22HB_SetOffsetPressure(LPS22HB_Handle_t *dev, int32_t value);
int32_t LPS22HB_ReadOffsetPressure(LPS22HB_Handle_t *dev);


#endif //I2CTEST_LPS22HB_H
