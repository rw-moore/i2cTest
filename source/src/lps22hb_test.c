//
// Functions to test the LPS22HB functionality
//
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lps22hb.h"

/*
 * Parses commands that execute LPS22HB functions
 */
void LPS22HB_cmdparser(char *buf) {
  char *cmd = strtok(buf, " ");
  char *arg1 = strtok(NULL, " ");
  char *arg2 = strtok(NULL, " ");
  if (!cmd) {
    printf("Syntax error: lps22hb <command> [<args>]\n");
    return;
  }
  // Convert the command to lower case to simplify checking
  for (char *p=cmd; *p; ++p) *p = tolower(*p);
  /*
   * Loop over the possible commands and execute them using the LPS22HB API
   */
  // Read/write the reference pressure
  if(!strcmp(cmd,"refp")) {
    // Check to see if a value is provided and, if so, set the reference pressure
    // if not then we just read the reference pressure
    if(arg1) {
      int32_t ref = atoi(arg1);
      LPS22HB_SetRefPressure(&lps22hb, ref);
      printf("LPS22HB: Set reference pressure to %ld\n", ref);
    } else {
      int32_t ref = LPS22HB_ReadRefPressure(&lps22hb);
      printf("LPS22HB: Reference pressure is %ld\n", ref);
    }
    return;
  }

  // Read/write the offset pressure
  if(!strcmp(cmd,"offp")) {
    // Check to see if a value is provided and, if so, set the offset pressure
    // if not then we just read the offset pressure
    if(arg1) {
      int32_t offset = atoi(arg1);
      LPS22HB_SetOffsetPressure(&lps22hb, offset);
      printf("LPS22HB: Set offset pressure to %ld\n", offset);
    } else {
      int32_t offset = LPS22HB_ReadOffsetPressure(&lps22hb);
      printf("LPS22HB: Offset pressure is %ld\n", offset);
    }
    return;
  }

  // Read the pressure
  if(!strcmp(cmd,"pressure")) {
    int32_t pressure = LPS22HB_ReadPressure(&lps22hb);
    printf("LPS22HB: Pressure is %ld\n", pressure);
    return;
  }

  // Read the temperature
  if(!strcmp(cmd,"temp")) {
    int16_t temp = LPS22HB_ReadTemperature(&lps22hb);
    printf("LPS22HB: Temperature is %d\n", temp);
    return;
  }

  // Set the autozero mode on or off
  if(!strcmp(cmd,"autozero")) {
    // Convert the first argument to lower case to simplify checking
    for (char *p=arg1; *p; ++p) *p = tolower(*p);
    if(!strcmp(arg1,"on")) {
      LPS22HB_AutoZeroOn(&lps22hb);
    } else if(!strcmp(arg1,"off")) {
      LPS22HB_AutoZeroOff(&lps22hb);
    } else {
      printf("Syntax error: lps22hb autozero [on|off]\n");
    }
    return;
  }
}