/**
 * @file    ic_mixed_cli_procedures.h
 * @author  Paweł Kaźmierzewski <p.kazmierzewski@inteliclinic.com>
 * @author  Wojciech Weclewski <w.weclewski@inteliclinic.com>
 * @date    February, 2017
 * @brief   Contains CLI commands for testing all modules together.
 *
 *  For additional information about CLI, look into ic_cli module.
 */

#ifndef IC_MIXED_CLI_PROCEDURES_H
#define IC_MIXED_CLI_PROCEDURES_H

void cli_get_status(int c, int *args);

void cli_different_rsp_check_func(int c, int *args);

/**
 * @brief Different response check cycle updater.
 *
 * This function should be placed in main loop.
 */
void cli_different_rsp_check_update(void);

/**
 * @brief Start about 1 minute demo of all the LTC devices. For signals collision checking purpose.
 *
 * It shuould work by writing in terminal "dev_demo"
 *
 * @param c Nothing interesting. For additional information about CLI, look into ic_cli module.
 * @param args Nothing interesting. For additional information about CLI, look into ic_cli module.
 */
void cli_signal_collision_demo_func(int c, int *args);

/**
 * @brief Signal collision demo cycle updater.
 *
 * This function should be placed in main loop.
 */
void cli_signal_collision_demo_update(void);

#endif /* !IC_MIXED_CLI_PROCEDURES_H */
