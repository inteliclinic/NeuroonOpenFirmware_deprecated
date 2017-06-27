/**
 * @file    ic_ltc_common.c
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    August, 2016
 * @brief   Low-level functions for LTC devices.
 *
 *  LTC devices are RGB LEDs, vibration motor and power LEDs.
 */

#include "ic_ltc_common.h"
#include "ic_cc.h"
#include "ic_timers.h"
#include "ic_device_driver_api.h"
#include "ic_low_level_control.h"

bool pwr_led_on = false;

static void hello_set_left_rgb_led(uint8_t r, uint8_t g, uint8_t b){
  SET_LED_BLOCKING(RGB1_R_CHANNEL,r);
  SET_LED_BLOCKING(RGB1_G_CHANNEL,g);
  SET_LED_BLOCKING(RGB1_B_CHANNEL,b);
}

static void hello_set_right_rgb_led(uint8_t r, uint8_t g, uint8_t b){
  SET_LED_BLOCKING(RGB2_R_CHANNEL,r);
  SET_LED_BLOCKING(RGB2_G_CHANNEL,g);
  SET_LED_BLOCKING(RGB2_B_CHANNEL,b);
}

void flash_make_nonblock(void){
  static uint32_t fm_tmp_time = 0;

  if(!pwr_led_on){
    pwr_led_on = true;
    ltc3220_set_chargepump(2);
    ltc3220_set_brightness(POWER_LED3_CHANNEL1, 0x3f);

    power_led_flash_on();
    ltc3220_set_brightness(POWER_LED3_CHANNEL1, 0x00);

    fm_tmp_time = timer_get_ms_counter();
  } else if (fm_tmp_time + 2 <= timer_get_ms_counter()) {
    power_led_flash_off();
    pwr_led_on = false;
    all_dev_driver_refresh();
  }
}

bool get_flash_make_state(void){
  return pwr_led_on;
}

void bye_procedure(void){
  hello_set_left_rgb_led(50,50,10);
  hello_set_right_rgb_led(50,50,10);
  for (int i = 1; i<=10; ++i){
    hello_set_left_rgb_led(50-i*5, 50-i*5, 10-i);
    hello_set_right_rgb_led(50-i*5, 50-i*5, 10-i);
    ic_delay_ms(30);
    WDT_RR();
  }
  dev_set_new_parameters(RIGHT_RED_LED, FUN_TYPE_ON, 0, 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(RIGHT_GREEN_LED, FUN_TYPE_ON, 0, 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(RIGHT_BLUE_LED, FUN_TYPE_ON, 0, 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(LEFT_RED_LED, FUN_TYPE_ON, 0, 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(LEFT_GREEN_LED, FUN_TYPE_ON, 0, 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(LEFT_BLUE_LED, FUN_TYPE_ON, 0, 0, 0, timer_get_ms_counter());
}

void hello_procedure(void){
  uint8_t tmp_left_rgb_val[3];
  uint8_t tmp_right_rgb_val[3];
  hello_set_left_rgb_led(0,0,0);
  hello_set_right_rgb_led(0,0,0);

  for (int i = 1; i<=10; ++i){
    tmp_left_rgb_val[0] = i*5;
    tmp_left_rgb_val[1] = i*5;
    tmp_left_rgb_val[2] = i>>1;
    tmp_right_rgb_val[0] = i*5;
    tmp_right_rgb_val[1] = i*5;
    tmp_right_rgb_val[2] = i>>1;
    hello_set_left_rgb_led(tmp_left_rgb_val[0], tmp_left_rgb_val[1], tmp_left_rgb_val[2]);
    hello_set_right_rgb_led(tmp_right_rgb_val[0], tmp_right_rgb_val[1], tmp_right_rgb_val[2]);
    ic_delay_ms(5);
    WDT_RR();
  }
  for (int i = 10; i>=1; --i){
    tmp_left_rgb_val[0] = i*5;
    tmp_left_rgb_val[1] = i*5;
    tmp_left_rgb_val[2] = i+1;
    tmp_right_rgb_val[0] = i*5;
    tmp_right_rgb_val[1] = i*5;
    tmp_right_rgb_val[2] = i+1;
    hello_set_left_rgb_led(tmp_left_rgb_val[0], tmp_left_rgb_val[1], tmp_left_rgb_val[2]);
    hello_set_right_rgb_led(tmp_right_rgb_val[0], tmp_right_rgb_val[1], tmp_right_rgb_val[2]);
    ic_delay_ms(40);
    WDT_RR();
  }
  dev_set_new_parameters(RIGHT_RED_LED, FUN_TYPE_ON, tmp_right_rgb_val[0], 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(RIGHT_GREEN_LED, FUN_TYPE_ON, tmp_right_rgb_val[1], 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(RIGHT_BLUE_LED, FUN_TYPE_ON, tmp_right_rgb_val[2], 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(LEFT_RED_LED, FUN_TYPE_ON, tmp_left_rgb_val[0], 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(LEFT_GREEN_LED, FUN_TYPE_ON, tmp_left_rgb_val[1], 0, 0, timer_get_ms_counter());
  dev_set_new_parameters(LEFT_BLUE_LED, FUN_TYPE_ON, tmp_left_rgb_val[2], 0, 0, timer_get_ms_counter());
}

void all_dev_driver_refresh(void){
  if (dev_is_active(RIGHT_RED_LED))
    SET_LED_BLOCKING(RGB1_R_CHANNEL, dev_get_selected_output(RIGHT_RED_LED).channels.value);
  if (dev_is_active(RIGHT_GREEN_LED))
    SET_LED_BLOCKING(RGB1_G_CHANNEL, dev_get_selected_output(RIGHT_GREEN_LED).channels.value);
  if (dev_is_active(RIGHT_BLUE_LED))
    SET_LED_BLOCKING(RGB1_B_CHANNEL, dev_get_selected_output(RIGHT_BLUE_LED).channels.value);
  if (dev_is_active(LEFT_RED_LED))
    SET_LED_BLOCKING(RGB2_R_CHANNEL, dev_get_selected_output(LEFT_RED_LED).channels.value);
  if (dev_is_active(LEFT_GREEN_LED))
    SET_LED_BLOCKING(RGB2_G_CHANNEL, dev_get_selected_output(LEFT_GREEN_LED).channels.value);
  if (dev_is_active(LEFT_BLUE_LED))
    SET_LED_BLOCKING(RGB2_B_CHANNEL, dev_get_selected_output(LEFT_BLUE_LED).channels.value);
  if (dev_is_active(VIBRATOR)) {
    SET_LED_BLOCKING(VIBRATOR1_CHANNEL1, dev_get_selected_output(VIBRATOR).channels.value);
    SET_LED_BLOCKING(VIBRATOR1_CHANNEL2, dev_get_selected_output(VIBRATOR).channels.value);
  }
}
