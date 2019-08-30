//
// Functions to test the LIS3MDL functionality
//
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lis3mdl.h"

/*
 * Parses commands that execute LIS3MDL functions
 */
void LIS3MDL_cmdparser(char *buf) {
  char *cmd = strtok(buf, " ");
  char *arg1 = strtok(NULL, " ");
  char *arg2 = strtok(NULL, " ");
  if (!cmd) {
    printf("Syntax error: lis3mdl <command> [<args>]\n");
    return;
  }
  // Convert the command to lower case to simplify checking
  for (char *p = cmd; *p; ++p) *p = tolower(*p);
  /*
   * Loop over the possible commands and execute them using the LIS3MDL API
   */
  // Read or write an LIS3MDL register
  if (!strcmp(cmd, "reg")) {
    uint8_t reg = atoi(arg1);
    if (!arg2) {
      uint8_t value = LIS3MDL_ReadRegister(&lis3mdl, reg);
      printf("LIS3MDL register 0x%02x read 0x%02x (%d)\n", reg, value, value);
    } else {
      uint8_t value = atoi(arg2);
      LIS3MDL_WriteRegister(&lis3mdl, reg, value);
      printf("LIS3MDL register 0x%02x wrote 0x%02x (%d)\n", reg, value, value);
    }
    return;
  }

  // Enable/disable temperature readout
  if (!strcmp(cmd, "tempsensor")) {
    // Convert the first argument to lower case to simplify checking
    for (char *p = arg1; *p; ++p) *p = tolower(*p);
    if (!strcmp(arg1, "on")) {
      LIS3MDL_EnableTemperature(&lis3mdl);
    } else if (!strcmp(arg1, "off")) {
      LIS3MDL_DisableTemperature(&lis3mdl);
    } else {
      printf("Syntax error: lis3mdl tempsensor [on|off]\n");
    }
    return;
  }

  // Set the range
  if (!strcmp(cmd, "range")) {
    uint8_t rng = atoi(arg1);
    LIS3MDL_SetRange(&lis3mdl, rng);
    return;
  }

  // Readout the raw temperature
  if (!strcmp(cmd, "temp")) {
    uint16_t temp = LIS3MDL_ReadTemperature(&lis3mdl);
    printf("LIS3MDL: Current temperature = %d\n", temp);
    return;
  }

  // Readout the raw magnetometer data
  if (!strcmp(cmd, "rawbfield")) {
    int16_t bx = LIS3MDL_RawXBField(&lis3mdl);
    int16_t by = LIS3MDL_RawYBField(&lis3mdl);
    int16_t bz = LIS3MDL_RawZBField(&lis3mdl);
    printf("LIS3MDL: Current raw B field = (%d, %d, %d)\n", bx, by, bz);
    return;
  }

  // Readout the magnetometer data
  if (!strcmp(cmd, "rawbfield")) {
    float_t bx = LIS3MDL_XBField(&lis3mdl);
    float_t by = LIS3MDL_YBField(&lis3mdl);
    float_t bz = LIS3MDL_ZBField(&lis3mdl);
    printf("LIS3MDL: Current B field = (%f, %f, %f) T\n", bx, by, bz);
    return;
  }
}