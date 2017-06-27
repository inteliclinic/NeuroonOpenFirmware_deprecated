/**
 * @file    ic_device_handle_api.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    October, 2016
 * @brief   LTC devices interface.
 *
 *  LTC devices are RGB LEDs, vibration motor and power LEDs.
 */

#ifndef IC_DEVICE_HANDLE_API_H
#define IC_DEVICE_HANDLE_API_H


#include <stdbool.h>
#include "ic_low_level_control.h"
#include "ic_device_handle.h"

typedef enum __attribute__((packed)){
  RIGHT_RED_LED = 0,
  RIGHT_GREEN_LED = 1,
  RIGHT_BLUE_LED = 2,
  LEFT_RED_LED = 3,
  LEFT_GREEN_LED = 4,
  LEFT_BLUE_LED = 5,
  VIBRATOR = 6,
  POWER_LED = 7,
  REGISTERED_DEVICES_NUMBER
}e_descriptors;

typedef union {
  uint16_t power_led_period;
  struct __attribute__((packed)) {
    e_descriptors desc;
    deviceOutput value;
  }channels;
}u_devVal;

u_devVal dev_get_selected_output(uint8_t desc);
u_devVal dev_power_led_get_function(void);

s_devsFunc dev_get_all_functions(void);

void dev_all_handlers_register(void);
void dev_all_update_when_active(void);

bool dev_right_rgb_set_new_parameters(s_deviceCmd *device_cmd_data);
bool dev_left_rgb_set_new_parameters(s_deviceCmd *device_cmd_data);
bool dev_power_set_new_parameters(s_deviceCmd *device_cmd_data);
bool dev_vibrator_set_new_parameters(s_deviceCmd *device_cmd_data);
void dev_all_set_new_parameters(s_deviceCmd *device_cmd_data);
#endif //IC_DEVICE_HANDLE_API_H
