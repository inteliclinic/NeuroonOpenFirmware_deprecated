/**
 * @file    ic_afe4400_cli_procedures.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    January, 2017
 * @brief   Contains CLI commands for AFE4400 (pulsoximeter) control.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#ifndef IC_AFE4400_CLI_PROCEDURES_H
#define IC_AFE4400_CLI_PROCEDURES_H

void cli_afe4400_hdw_init(int c, int *args);
void cli_afe4400_std_val_init(int c, int *args);
void cli_afe4400_powerdown_on(int c, int *args);
void cli_afe4400_powerdown_off(int c, int *args);

void cli_afe4400_set_selected_r(int c, int *args);

void cli_afe4400_get_read_selected_r(int c, int *args);

void cli_afe4400_self_test(int c, int *args);

#endif /* !IC_AFE4400_CLI_PROCEDURES_H */
