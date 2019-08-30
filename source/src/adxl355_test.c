//
// Functions to test the ADXL355 functionality
//
#include <ctype.h>

#include "adxl355.h"

/*
 * Parses commands that execute ADXL355 functions
 */
void ADXL355_cmdparser(const char *buf) {
  char *cmd = strtok(buf, " ");
  char *arg1 = strtok(NULL, " ");
  char *arg2 = strtok(NULL, " ");
  if (!cmd) {
    printf("Syntax error: adxl355 <command> [<args>]\n");
    return;
  }
  // Convert the command to lower case to simplify checking
  for (char *p=cmd; *p; ++p) *p = tolower(*p);
  /*
   * Loop over the possible commands and execute them using the ADXL355 API
   */
  // Read or write an ADXL355 register
  if(!strcmp(cmd,"reg")) {
    uint8_t reg = atoi(arg1);
    if (!arg2) {
      uint8_t value = ADXL355_ReadRegister(&adxl355, reg);
      printf("ADXL355 register 0x%02x read 0x%02x (%d)\n", reg, value, value);
    } else {
      uint8_t value = atoi(arg2);
      ADXL355_WriteRegister(&adxl355, reg, value);
      printf("ADXL355 register 0x%02x wrote 0x%02x (%d)\n", reg, value, value);
    }
    return;
  }

  // Set the standby mode on or off
  if(!strcmp(cmd,"standby")) {
    // Convert the first argument to lower case to simplify checking
    for (char *p=arg1; *p; ++p) *p = tolower(*p);
    if(!strcmp(arg1,"on")) {
      ADXL355_StandbyOn(&adxl355);
    } else if(!strcmp(arg1,"off")) {
      ADXL355_StandbyOff(&adxl355);
    } else {
      printf("Syntax error: adxl355 standby [on|off]\n");
    }
    return;
  }

  // Enable/disable temperature readout
  if(!strcmp(cmd,"tempsensor")) {
    // Convert the first argument to lower case to simplify checking
    for (char *p=arg1; *p; ++p) *p = tolower(*p);
    if(!strcmp(arg1,"on")) {
      ADXL355_EnableTemperature(&adxl355);
    } else if(!strcmp(arg1,"off")) {
      ADXL355_DisableTemperature(&adxl355);
    } else {
      printf("Syntax error: adxl355 tempsensor [on|off]\n");
    }
    return;
  }

  // Set the range
  if(!strcmp(cmd,"range")) {
    uint8_t rng = atoi(arg1);
    ADXL355_SetRange(&adxl355,rng);
    return;
  }

  // Readout the raw temperature
  if(!strcmp(cmd,"rawtemp")) {
    uint16_t temp=ADXL355_RawTemperature(&adxl355);
    printf("ADXL355: Current raw temperature = %d\n",temp);
    return;
  }

  // Readout the acceleration
  if(!strcmp(cmd,"accel")) {
    int32_t xaccel=ADXL355_XAcceleration(&adxl355);
    int32_t yaccel=ADXL355_YAcceleration(&adxl355);
    int32_t zaccel=ADXL355_ZAcceleration(&adxl355);
    printf("ADXL355: Cuurent acceleration = (%d, %d, %d)\n",xaccel,yaccel,zaccel);
    return;
  }

  // Readout the trim
  if(!strcmp(cmd,"trim")) {
    int32_t xtrim=ADXL355_XTrim(&adxl355);
    int32_t ytrim=ADXL355_YTrim(&adxl355);
    int32_t ztrim=ADXL355_ZTrim(&adxl355);
    printf("ADXL355: Cuurent trim = (%d, %d, %d)\n",xtrim,ytrim,ztrim);
    return;
  }

  // Sets the X trim
  if(!strcmp(cmd,"setxtrim")) {
    int32_t trim = atoi(arg1);
    ADXL355_SetXTrim(&adxl355,trim);
    printf("ADXL355: Set X trim to %d\n",trim);
    return;
  }

  // Sets the Y trim
  if(!strcmp(cmd,"setytrim")) {
    int32_t trim = atoi(arg1);
    ADXL355_SetYTrim(&adxl355,trim);
    printf("ADXL355: Set X trim to %d\n",trim);
    return;
  }

  // Sets the Z trim
  if(!strcmp(cmd,"setztrim")) {
    int32_t trim = atoi(arg1);
    ADXL355_SetZTrim(&adxl355,trim);
    printf("ADXL355: Set Z trim to %d\n",trim);
    return;
  }

}
