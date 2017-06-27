/**
 * @file    ic_e_alarm_cli_procedures.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   Contains CLI commands for emergency alarm control.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#include "ic_e_alarm_cli_procedures.h"
#include "ic_common_cli_procedures.h"
#include "ic_cmd_emiter.h"
#include "ic_low_level_control.h"
#include <string.h>

void cli_alarm_set(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  alarm_set(array, &len, (e_alarmType)args[0], (uint32_t)args[1], (uint16_t)args[2], (uint16_t)args[3]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_alarm_off(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  alarm_off(array, &len, (uint16_t)args[0]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

