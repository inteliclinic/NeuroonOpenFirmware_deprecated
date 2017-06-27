/**
 * @file    ic_afe4400_cli_procedures.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    January, 2017
 * @brief   Contains CLI commands for AFE4400 (pulsoximeter) control.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#include "ic_afe4400_cli_procedures.h"
#include "ic_common_cli_procedures.h"
#include "ic_afe4400.h"
#include "ic_cmd_emiter.h"
#include "ic_low_level_control.h"
#include <stdlib.h>
#include <string.h>

void cli_afe4400_hdw_init(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_hdw_init(array, &len, (uint16_t)args[0]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_afe4400_std_val_init(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_std_val_init(array, &len, (uint16_t)args[0]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_afe4400_powerdown_on(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_powerdown_on(array, &len, (uint16_t)args[0]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_afe4400_powerdown_off(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_powerdown_off(array, &len, (uint16_t)args[0]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_afe4400_get_read_selected_r(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_read_register(array, &len, (t_afe4400Register)args[0], (uint16_t)args[1]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_afe4400_set_selected_r(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_write_register(array, &len, (t_afe4400Register)args[0], (t_afe4400RegisterConf)args[1], (uint16_t)args[2]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_afe4400_self_test(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pox_self_test(array, &len, (uint16_t)args[0]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

