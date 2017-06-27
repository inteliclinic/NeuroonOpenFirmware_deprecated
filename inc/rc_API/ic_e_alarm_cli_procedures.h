/**
 * @file    ic_e_alarm_cli_procedures.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   Contains CLI commands for emergency alarm control.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#ifndef IC_E_ALARM_CLI_PROCEDURES_H
#define IC_E_ALARM_CLI_PROCEDURES_H

void cli_alarm_set(int c, int *args);
void cli_alarm_off(int c, int *args);

#endif /* !IC_E_ALARM_CLI_PROCEDURES_H */
