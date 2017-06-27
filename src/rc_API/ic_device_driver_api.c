/**
 * @file    ic_device_driver_api.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    October, 2016
 * @brief   API for LTC channels and power LED drivers.
 */

#include "ic_device_driver_api.h"
#include "ic_device_handle_api.h"
#include "ic_ltc_common.h"
#include <stdint.h>
#include "ic_timers.h"
#include "ic_cc.h"

#define DEV_DRIVER_UPDATE_DELAY 200
#define MIN_BLINK_PERIOD 200 //look into certification documentation
uint32_t timer_for_dev_update[REGISTERED_DEVICES_NUMBER];

static void priv_drv_power_led_update(void);

void drv_power_led_set_val(u_devVal device){
  timer_for_dev_update[POWER_LED] = device.power_led_period < MIN_BLINK_PERIOD ? timer_get_ms_counter() + MIN_BLINK_PERIOD : timer_get_ms_counter() + device.power_led_period;
  flash_make_nonblock();
}

void drv_dev_set_val(u_devVal device){
  timer_for_dev_update[device.channels.desc] = timer_get_ms_counter() + DEV_DRIVER_UPDATE_DELAY;
  switch (device.channels.desc) {
    case RIGHT_RED_LED:
      SET_LED_BLOCKING(RGB1_R_CHANNEL, device.channels.value);
      break;
    case RIGHT_GREEN_LED:
      SET_LED_BLOCKING(RGB1_G_CHANNEL, device.channels.value);
      break;
    case RIGHT_BLUE_LED:
      SET_LED_BLOCKING(RGB1_B_CHANNEL, device.channels.value);
      break;
    case LEFT_RED_LED:
      SET_LED_BLOCKING(RGB2_R_CHANNEL, device.channels.value);
      break;
    case LEFT_GREEN_LED:
      SET_LED_BLOCKING(RGB2_G_CHANNEL, device.channels.value);
      break;
    case LEFT_BLUE_LED:
      SET_LED_BLOCKING(RGB2_B_CHANNEL, device.channels.value);
      break;
    case VIBRATOR:
      SET_LED_BLOCKING(VIBRATOR1_CHANNEL1, device.channels.value);
      SET_LED_BLOCKING(VIBRATOR1_CHANNEL2, device.channels.value);
      break;
    default:
      break;
  }
}

void drv_all_dev_update(uint32_t timer){
  priv_drv_power_led_update();
  if ((timer >= timer_for_dev_update[RIGHT_RED_LED]) && dev_is_active(RIGHT_RED_LED))
    cc_emit(CC_UPDATE_RR_LED,(uint32_t)dev_get_selected_output);
  if ((timer >= timer_for_dev_update[RIGHT_GREEN_LED]) && dev_is_active(RIGHT_GREEN_LED))
    cc_emit(CC_UPDATE_RG_LED,(uint32_t)dev_get_selected_output);
  if ((timer >= timer_for_dev_update[RIGHT_BLUE_LED]) && dev_is_active(RIGHT_BLUE_LED))
    cc_emit(CC_UPDATE_RB_LED,(uint32_t)dev_get_selected_output);
  if ((timer >= timer_for_dev_update[LEFT_RED_LED]) && dev_is_active(LEFT_RED_LED))
    cc_emit(CC_UPDATE_LR_LED,(uint32_t)dev_get_selected_output);
  if ((timer >= timer_for_dev_update[LEFT_GREEN_LED]) && dev_is_active(LEFT_GREEN_LED))
    cc_emit(CC_UPDATE_LG_LED,(uint32_t)dev_get_selected_output);
  if ((timer >= timer_for_dev_update[LEFT_BLUE_LED]) && dev_is_active(LEFT_BLUE_LED))
    cc_emit(CC_UPDATE_LB_LED,(uint32_t)dev_get_selected_output);
  if ((timer >= timer_for_dev_update[VIBRATOR]) && dev_is_active(VIBRATOR))
    cc_emit(CC_UPDATE_VIBRATOR,(uint32_t)dev_get_selected_output);
}

static void priv_drv_power_led_update(void){
  if(get_flash_make_state())
    flash_make_nonblock();
  if ((timer_get_ms_counter() >= timer_for_dev_update[POWER_LED]) && dev_is_active(POWER_LED))
    cc_emit(CC_UPDATE_POWER,(uint32_t)dev_power_led_get_function);
}
