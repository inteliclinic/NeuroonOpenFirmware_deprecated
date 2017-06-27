/**
 * @file    ic_e_alarm_api.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Węclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   API for emergency alarm module.
 *
 *  Emergency alarm is a alarm feature used when Neuroon loses connection with application.
 *
 */

#ifndef IC_E_ALARM_H
#define IC_E_ALARM_H

#include <stdbool.h>
#include <stdint.h>
#include "ic_low_level_control.h"

void alarm_module_init(void);
void alarm_cmd_handle(s_alarmCmd *alarm_cmd_data);
void alarm_update(bool is_connected, uint32_t timer, uint32_t timer_for_device);

#endif /* !IC_E_ALARM_H */
