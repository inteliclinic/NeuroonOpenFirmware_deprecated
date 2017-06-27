/**
 * @file    ic_ltc_common.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    August, 2016
 * @brief   Low-level functions for LTC devices.
 *
 *  LTC devices are RGB LEDs, vibration motor and power LEDs.
 */

#ifndef IC_LTC_COMMON_H
#define IC_LTC_COMMON_H

#include <stdint.h>
#include "ic_ltc3220.h"

#define SET_LED_BLOCKING(led, level) { \
  ltc3220_set_brightness(led, level); \
}

void flash_make_nonblock(void);
bool get_flash_make_state(void);

void hello_procedure(void);
void bye_procedure(void);

void all_dev_driver_refresh(void);
#endif //IC_LTC_COMMON_H
