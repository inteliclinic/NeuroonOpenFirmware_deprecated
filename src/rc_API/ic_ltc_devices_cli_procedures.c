/**
 * @file    ic_ltc_devices_cli_procedures.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    November, 2016
 * @brief   Contains CLI procedures for LTC devices handlers and drivers testing.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#include "ic_ltc_devices_cli_procedures.h"
#include <string.h>
#include "ic_low_level_control.h"
#include "ic_cmd_emiter.h"
#include "ic_common_cli_procedures.h"

void cli_rgb_led_set_func(int c, int *args) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  rgb_led_set_func(array, &len, (e_rgbLedSide)args[0], (e_funcType)args[1], (e_rgbLedColor)args[2], (uint8_t)args[3], (uint16_t)args[4], (uint16_t)args[5], (uint16_t)args[6]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_vibra_set_func(int c, int *args) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  vibrator_set_func(array, &len, (e_funcType)args[0], (uint8_t)args[1], (uint32_t)args[3], (uint32_t)args[2], (uint16_t)args[4]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_vibra_ON(int c, int *args){
  char array[20];
  memset(array, 0, sizeof(array));
  size_t len = 20;
  vibrator_set_func(array, &len, FUN_TYPE_ON, args[0], 0, 0, 0);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_pwr_led_set_func(int c, int *args) {
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  pwr_led_set_func(array, &len, (e_funcType)args[0], (uint32_t)args[2], (uint32_t)args[1], (uint16_t)args[3]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_vibra_ramp_func(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  vibrator_set_func(array, &len, FUN_TYPE_RAMP, (uint8_t)args[0], DEV_INF_DURATION, DEV_MIN_PERIOD, (uint16_t)args[1]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}

void cli_rgb_ramp_func(int c, int *args){
  char array[CLI_BLE_FRAME_SIZE];
  memset(array, 0, sizeof(array));
  size_t len = CLI_BLE_FRAME_SIZE;
  rgb_led_set_func(array, &len, (e_rgbLedSide)args[0], FUN_TYPE_RAMP, (e_rgbLedColor)args[1], (uint8_t)args[2], DEV_INF_DURATION, DEV_MIN_PERIOD, (uint16_t)args[3]);
  push_data_CMD0Frame((uint8_t*)array, (uint16_t)len);
}
